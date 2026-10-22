"""Bring a RealSense D400 camera to the firmware the installed librealsense recommends.

Follows librealsense's own tool (tools/fw-update/rs-fw-update.cpp): back up the flash, check the
image is compatible, switch the camera into update (DFU) mode, burn the signed image and wait for it
to come back. The image is downloaded from the URL librealsense's build uses and must match the
SHA-1 recorded in librealsense's source; anything unexpected skips the update instead of forcing it.
A camera left in recovery mode by an interrupted update is re-flashed on the next run.
"""

import hashlib
import os
import re
import sys
import time
import urllib.request
from datetime import datetime

import pyrealsense2 as rs

from usb_ports import udev_rules_installed

FIRMWARE_URL = "https://librealsense.intel.com/Releases/RS4xx/FW/D4XX_FW_Image-{version}.bin"
# SHA-1 of D4XX_FW_Image-<version>.bin from librealsense's common/fw/CMakeLists.txt.
KNOWN_IMAGES = {"5.17.0.10": "e7a67224fd0bd823df03dc7f1135a59db93746fd"}   # librealsense 2.56.5
DEVICE_TIMEOUT_SEC = 30       # rs-fw-update waits 15 s; re-enumeration on macOS is slower
MIPI_RECOVERY_PID = "BBCD"    # D457 recovery needs a different procedure
NO_UPDATE_ACCESS = ("on Linux it needs root or the RealSense udev rules "
                    "(run-linux.sh uses sudo when the rules are missing)")


def version_tuple(text):
    return tuple(int(n) for n in re.findall(r"\d+", text or ""))


def _print(text, end="\n"):
    print(text, end=end, flush=True)


def info(dev, field):
    try:
        return dev.get_info(field) if dev.supports(field) else ""
    except RuntimeError:
        return ""


class FirmwareUpdater:
    def __init__(self, ctx, folder, usb_capture=None, on_file_written=None, log=_print):
        self.ctx = ctx
        self.folder = folder
        self.usb_capture = usb_capture
        self.on_file_written = on_file_written or (lambda *paths: None)
        self.log = log

    def run(self, serial=None):
        """Update or recover the camera if needed. Returns a short status; never raises."""
        try:
            return self._run(serial)
        except Exception as e:   # the camera may now be in recovery mode; the next run re-flashes it
            self.log(f"Firmware update failed: {e}")
            self.log("If the camera is stuck in recovery mode, run the launcher again to re-flash it.")
            return "failed"

    def _run(self, serial):
        devices = self._devices()
        recovery = [d for d in devices if d.is_update_device()]
        cameras = [d for d in devices if not d.is_update_device()
                   and serial in (None, info(d, rs.camera_info.serial_number))]
        if recovery and not cameras:
            return self._recover(recovery)
        if not cameras:
            return "no camera"

        dev = cameras[0]
        sn = info(dev, rs.camera_info.serial_number)
        current = info(dev, rs.camera_info.firmware_version)
        target = info(dev, rs.camera_info.recommended_firmware_version)
        if not current or not target or version_tuple(current) >= version_tuple(target):
            return "up to date"
        if info(dev, rs.camera_info.product_line) != "D400" or not dev.is_updatable():
            return self._skip(f"{info(dev, rs.camera_info.name)} is not a D400 camera this tool can update")
        if not self._can_reach_update_mode():
            return self._skip(NO_UPDATE_ACCESS)
        usb = info(dev, rs.camera_info.usb_type_descriptor)
        if not usb.startswith("3"):   # rs-fw-update also only warns here
            self.log(f"Warning: the camera is on USB {usb or '?'}. If the update fails, "
                     "connect it to a USB 3 port and run the launcher again.")
        image = self._image(target)
        if image is None:
            return "skipped"

        self.log(f"Firmware {current} is older than {target}: updating S/N {sn}. Do not unplug the camera.")
        updatable = dev.as_updatable()
        backup = updatable.create_flash_backup(self._progress("Backing up flash"))
        self.log("")
        if not backup:
            return self._skip("flash backup failed")
        path = self.folder / f"flash_backup_{sn}_{current}_{datetime.now():%Y%m%d_%H%M%S}.bin"
        path.write_bytes(bytes(backup))
        self.on_file_written(self.folder, path)
        self.log(f"Flash backup saved: {path}")
        if not updatable.check_firmware_compatibility(image):
            return self._skip(f"firmware {target} is not compatible with this camera")

        update_id = info(dev, rs.camera_info.firmware_update_id)
        updatable.enter_update_state()
        dfu = self._wait(lambda d: d.is_update_device()
                         and info(d, rs.camera_info.firmware_update_id) == update_id)
        if dfu is None:
            raise RuntimeError("the camera did not show up in update mode")
        self._flash(dfu, image)

        back = self._wait(lambda d: not d.is_update_device()
                          and info(d, rs.camera_info.serial_number) == sn)
        if back is None:
            raise RuntimeError("the camera did not come back after flashing")
        now = info(back, rs.camera_info.firmware_version)
        if version_tuple(now) != version_tuple(target):
            raise RuntimeError(f"the camera reports firmware {now} after flashing {target}")
        self.log(f"Firmware updated: {current} -> {now}")
        return "updated"

    def _recover(self, recovery):
        if len(recovery) > 1:
            return self._skip("more than one camera is in recovery mode; recover them one at a time")
        if not self._can_reach_update_mode():
            return self._skip(NO_UPDATE_ACCESS)
        dfu = recovery[0]
        if info(dfu, rs.camera_info.product_id).upper() == MIPI_RECOVERY_PID:
            return self._skip("MIPI (D457) recovery is not supported by this tool")
        version = max(KNOWN_IMAGES, key=version_tuple)
        self.log(f"A camera is in recovery mode: flashing firmware {version}. Do not unplug the camera.")
        image = self._image(version)
        if image is None:
            return "skipped"
        self._flash(dfu, image)
        back = self._wait(lambda d: not d.is_update_device())
        if back is None:
            raise RuntimeError("the camera did not come back after recovery")
        self.log(f"Camera recovered with firmware {info(back, rs.camera_info.firmware_version)}")
        return "recovered"

    @staticmethod
    def _can_reach_update_mode():
        """In update mode the camera is a new USB device; on Linux only root or udev rules can open it,
        and finding that out after enter_update_state() would leave the camera in recovery mode."""
        return not sys.platform.startswith("linux") or os.geteuid() == 0 or udev_rules_installed()

    def _flash(self, dfu, image):
        dfu.as_update_device().update(image, self._progress("Flashing firmware"))
        self.log("")

    def _image(self, version):
        """The verified image as list[int] (what pybind expects), downloading it once."""
        sha1 = KNOWN_IMAGES.get(version)
        if sha1 is None:
            self._skip(f"no verified image is known for firmware {version}")
            return None
        path = self.folder / f"D4XX_FW_Image-{version}.bin"
        if not path.exists() or hashlib.sha1(path.read_bytes()).hexdigest() != sha1:
            self.folder.mkdir(exist_ok=True)
            url = FIRMWARE_URL.format(version=version)
            self.log(f"Downloading {url}")
            try:
                with urllib.request.urlopen(url, timeout=60) as response:
                    data = response.read()
            except OSError as e:
                self._skip(f"download failed ({e})")
                return None
            if hashlib.sha1(data).hexdigest() != sha1:
                self._skip("downloaded image does not match librealsense's SHA-1")
                return None
            path.write_bytes(data)
            self.on_file_written(self.folder, path)
        return list(path.read_bytes())

    def _devices(self):
        if self.usb_capture is not None:   # a camera that just rebooted is a new USB device
            self.usb_capture.capture_new()
        found = []
        devices = self.ctx.query_devices()
        for i in range(len(devices)):
            try:
                found.append(devices[i])
            except RuntimeError:   # still enumerating, or not accessible
                pass
        return found

    def _wait(self, predicate):
        deadline = time.monotonic() + DEVICE_TIMEOUT_SEC
        while time.monotonic() < deadline:
            match = next((d for d in self._devices() if predicate(d)), None)
            if match is not None:
                return match
            time.sleep(1.0)
        return None

    def _progress(self, label):
        shown = []

        def report(fraction):   # called for every chunk; print only when the percentage changes
            percent = int(fraction * 100)
            if not shown or percent != shown[-1]:
                shown.append(percent)
                self.log(f"\r{label}: {percent:3d}%", end="")
        return report

    def _skip(self, reason):
        self.log(f"Firmware update skipped: {reason}")
        return "skipped"
