"""Find which USB port each RealSense camera is plugged into, and whether this user can open it.

Run by run-mac.sh / run-linux.sh before the inspector starts: prints a short report and exits with
status 10 when the inspector has to run as root, 0 otherwise. Linux reads sysfs and checks the
camera's /dev nodes; macOS reads the IOKit registry and also spots a USB-C port whose data macOS
holds back until the user allows the accessory. Standard library only, so it runs before anything
else is installed.
"""

from __future__ import annotations

import glob
import os
import plistlib
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path

INTEL_VID = 0x8086
# D400 USB product IDs, from librealsense src/ds/d400/d400-private.h.
D400_MODELS = {
    0x0AD1: "D400", 0x0AD2: "D410", 0x0AD3: "D415", 0x0AD4: "D430", 0x0AD5: "D430",
    0x0AD6: "D400 (USB2)", 0x0AF6: "D420", 0x0AFE: "D420", 0x0AFF: "D410", 0x0B00: "D400",
    0x0B01: "D430", 0x0B03: "D460", 0x0B07: "D435", 0x0B0C: "DS5U", 0x0B3A: "D435i",
    0x0B49: "F416", 0x0B4B: "D430i", 0x0B52: "F416", 0x0B5B: "D405", 0x0B5C: "D455", 0x1155: "D421",
}
RECOVERY_PIDS = {0x0ADB, 0x0ADC}          # firmware-update (DFU) mode
NEEDS_ROOT = 10                           # exit status telling the launcher to use sudo

SYSFS_USB = Path("/sys/bus/usb/devices")
DEV = Path("/dev")
UDEV_RULE_DIRS = ("/etc/udev/rules.d", "/lib/udev/rules.d", "/usr/lib/udev/rules.d")
MAC_SPEED_MBPS = {0: 1.5, 1: 12, 2: 480, 3: 5000, 4: 10000}   # IOKit "Device Speed"


@dataclass
class UsbCamera:
    model: str
    pid: int
    port: str              # Linux: sysfs port path such as "2-1.4"; macOS: "Port-USB-C@1"
    speed_mbps: float
    usb_serial: str        # the camera's ASIC serial, not the S/N librealsense reports
    nodes: list = field(default_factory=list)   # Linux /dev nodes the inspector opens

    @property
    def recovery(self):
        return self.pid in RECOVERY_PIDS

    @property
    def speed(self):
        if self.speed_mbps >= 5000:
            return f"USB 3 ({self.speed_mbps / 1000:g} Gbps)"
        if self.speed_mbps >= 480:
            return "USB 2 (480 Mbps)"
        return f"USB 1 ({self.speed_mbps:g} Mbps)" if self.speed_mbps else "unknown speed"

    def accessible(self):
        return all(os.access(node, os.R_OK | os.W_OK) for node in self.nodes)


def _model(pid, product):
    if pid in RECOVERY_PIDS:
        return "RealSense D4xx in recovery mode"
    return f"RealSense {D400_MODELS[pid]}" if pid in D400_MODELS else product or f"RealSense {pid:04x}"


def _read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return ""


# --------------------------------------------------------------------------- Linux

def linux_cameras(sysfs=SYSFS_USB, dev=DEV):
    cameras = []
    for path in sorted(glob.glob(str(sysfs / "*"))):
        name = os.path.basename(path)
        if ":" in name or _read(f"{path}/idVendor").lower() != f"{INTEL_VID:04x}":
            continue   # interfaces are listed as "2-1:1.0"; only devices are wanted
        try:
            pid = int(_read(f"{path}/idProduct"), 16)
        except ValueError:
            continue
        if pid not in D400_MODELS and pid not in RECOVERY_PIDS:
            continue
        nodes = []
        bus, num = _read(f"{path}/busnum"), _read(f"{path}/devnum")
        if bus.isdigit() and num.isdigit():
            nodes.append(str(dev / "bus" / "usb" / f"{int(bus):03d}" / f"{int(num):03d}"))
        nodes += sorted(str(dev / os.path.basename(v))
                        for v in glob.glob(f"{sysfs}/{name}:*/video4linux/video*"))
        try:
            speed = float(_read(f"{path}/speed") or 0)
        except ValueError:
            speed = 0
        cameras.append(UsbCamera(_model(pid, _read(f"{path}/product")), pid, name, speed,
                                 _read(f"{path}/serial"), nodes))
    return cameras


def udev_rules_installed(dirs=UDEV_RULE_DIRS):
    """True if a udev rule opens RealSense devices, including firmware-update mode, to users."""
    for directory in dirs:
        for rules in glob.glob(os.path.join(directory, "*.rules")):
            text = _read(rules).lower()
            if "8086" in text and "0adb" in text:
                return True
    return False


# --------------------------------------------------------------------------- macOS

def _ioreg(*args):
    out = subprocess.run(["ioreg", "-a", *args, "-w0"], capture_output=True).stdout
    try:
        return plistlib.loads(out) if out.strip() else []
    except Exception:
        return []


def _walk(node):
    yield node
    for child in node.get("IORegistryEntryChildren", []):
        yield from _walk(child)


def mac_cameras():
    # Root ports know their physical USB-C port (Apple Silicon); devices behind a hub share
    # the root port's top 12 bits of locationID.
    root_ports = {p.get("locationID"): p.get("UsbIOPort", "").rsplit("/", 1)[-1]
                  for p in _ioreg("-r", "-k", "UsbIOPort")}
    tree = _ioreg("-p", "IOUSB", "-l")
    cameras = []
    for node in _walk(tree) if isinstance(tree, dict) else []:
        pid = node.get("idProduct", 0)
        if node.get("idVendor") != INTEL_VID or (pid not in D400_MODELS and pid not in RECOVERY_PIDS):
            continue
        location = node.get("locationID", 0)
        port = root_ports.get(location & 0xFFF00000) or f"USB location 0x{location:08x}"
        cameras.append(UsbCamera(_model(pid, node.get("USB Product Name", "")), pid, port,
                                 MAC_SPEED_MBPS.get(node.get("Device Speed"), 0),
                                 node.get("kUSBSerialNumberString", "")))
    return cameras


def mac_blocked_ports():
    """USB-C ports with something plugged in whose USB data macOS has not authorised."""
    blocked = []
    for port in _ioreg("-r", "-c", "IOPort", "-l"):
        usb = [c for c in port.get("IORegistryEntryChildren", [])
               if c.get("IORegistryEntryName") in ("USB2", "USB3")]
        if (port.get("ConnectionActive") and usb and not any(c.get("Active") for c in usb)
                and any(c.get("AuthorizationStatusDescription") == "Unauthorized" for c in usb)):
            blocked.append(port.get("Description") or "a USB-C port")
    return blocked


# --------------------------------------------------------------------------- common

def find_cameras():
    if sys.platform == "darwin":
        return mac_cameras()
    if sys.platform.startswith("linux"):
        return linux_cameras()
    return []


def camera_for(usb_serial=None):
    """The USB entry of a connected camera: by ASIC serial, or the only one plugged in."""
    cameras = [c for c in find_cameras() if not c.recovery]
    by_serial = [c for c in cameras if usb_serial and c.usb_serial == usb_serial]
    if by_serial:
        return by_serial[0]
    return cameras[0] if len(cameras) == 1 else None


def needs_root(cameras):
    if os.geteuid() == 0:
        return False
    if sys.platform == "darwin":
        return True   # librealsense reaches the camera through libusb, which needs root on macOS
    return not udev_rules_installed() or not all(c.accessible() for c in cameras)


def report(cameras, blocked=()):
    lines = []
    for c in cameras:
        lines.append(f"{c.model} on port {c.port}, {c.speed}"
                     + (f", USB serial {c.usb_serial}" if c.usb_serial else ""))
        if c.recovery:
            lines.append("  In firmware-update (recovery) mode: the inspector will re-flash it.")
        elif c.speed_mbps < 5000:
            lines.append("  Not USB 3: streams are limited. Use a USB 3 port and data cable for full speed.")
        if c.nodes:
            lines.append("  Devices: " + " ".join(c.nodes) + ("" if c.accessible() else "  (root needed)"))
    if not cameras:
        lines.append("No RealSense camera found on USB.")
        for port in blocked:
            lines.append(f"  {port}: something is plugged in but macOS is blocking its USB data. Unlock "
                         "the Mac, replug it and click Allow (System Settings > Privacy & Security > "
                         "Allow accessories to connect).")
        lines.append("  Check it is a USB 3 data cable (not charge-only), firmly plugged in; "
                     "try flipping the USB-C plug at the camera.")
        lines.append("  The inspector keeps waiting for the camera.")
    return lines


def main():
    cameras = find_cameras()
    blocked = mac_blocked_ports() if sys.platform == "darwin" else []
    print("RealSense USB check:")
    for line in report(cameras, blocked):
        print("  " + line)
    if sys.platform.startswith("linux") and os.geteuid() != 0 and not udev_rules_installed():
        print("  No RealSense udev rules installed: starting the inspector with sudo (see README).")
    return NEEDS_ROOT if needs_root(cameras) else 0


if __name__ == "__main__":
    sys.exit(main())
