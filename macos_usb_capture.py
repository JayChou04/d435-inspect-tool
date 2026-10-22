"""Keep Intel RealSense cameras away from macOS's UVC driver for the life of the process.

librealsense powers a sensor up and down several times while it initialises. Its libusb handles
use auto-detach, and libusb's macOS backend implements every detach / re-attach as a full USB
re-enumeration, so each power-off hands the camera back to macOS's UVCAssistant, which then races
librealsense for the interfaces ("failed to claim usb interface ... RS2_USB_STATUS_ACCESS").

libusb counts captures per device: the first detach captures (one re-enumeration), every release
decrements, and only reaching zero re-enumerates. Capturing once through the *same* libusb library
that librealsense loaded and raising that count far above anything librealsense will release keeps
the camera captured. Same approach as
https://github.com/viam-modules/viam-camera-realsense/pull/182. Needs root, like librealsense itself.
"""

import ctypes
import sys
from pathlib import Path

INTEL_VID = 0x8086
MISC_DEVICE_CLASS = 0xEF     # RealSense cameras are composite (IAD) UVC devices
RECOVERY_PIDS = {0x0ADB, 0x0ADC, 0xBBCD}   # firmware-update (DFU) mode: not UVC, leave it alone
CAPTURE_HOLD = 100_000       # each librealsense interface release takes one off the count


class _DeviceDescriptor(ctypes.Structure):
    _fields_ = [
        ("bLength", ctypes.c_uint8), ("bDescriptorType", ctypes.c_uint8), ("bcdUSB", ctypes.c_uint16),
        ("bDeviceClass", ctypes.c_uint8), ("bDeviceSubClass", ctypes.c_uint8),
        ("bDeviceProtocol", ctypes.c_uint8), ("bMaxPacketSize0", ctypes.c_uint8),
        ("idVendor", ctypes.c_uint16), ("idProduct", ctypes.c_uint16), ("bcdDevice", ctypes.c_uint16),
        ("iManufacturer", ctypes.c_uint8), ("iProduct", ctypes.c_uint8),
        ("iSerialNumber", ctypes.c_uint8), ("bNumConfigurations", ctypes.c_uint8),
    ]


class UsbCapture:
    """Captures RealSense devices through pyrealsense2's own libusb and never releases them."""

    def __init__(self, libusb_path):
        lib = ctypes.CDLL(str(libusb_path))   # already loaded by pyrealsense2: same image, same state
        vp, ip = ctypes.c_void_p, ctypes.c_int
        lib.libusb_init.argtypes = [ctypes.POINTER(vp)]
        lib.libusb_get_device_list.argtypes = [vp, ctypes.POINTER(ctypes.POINTER(vp))]
        lib.libusb_get_device_list.restype = ctypes.c_ssize_t
        lib.libusb_free_device_list.argtypes = [ctypes.POINTER(vp), ip]
        lib.libusb_free_device_list.restype = None
        lib.libusb_get_device_descriptor.argtypes = [vp, ctypes.POINTER(_DeviceDescriptor)]
        lib.libusb_open.argtypes = [vp, ctypes.POINTER(vp)]
        lib.libusb_close.argtypes = [vp]
        lib.libusb_close.restype = None
        lib.libusb_detach_kernel_driver.argtypes = [vp, ip]
        lib.libusb_error_name.argtypes = [ip]
        lib.libusb_error_name.restype = ctypes.c_char_p
        self._lib = lib
        self._ctx = vp()
        rc = lib.libusb_init(ctypes.byref(self._ctx))
        if rc != 0:
            raise OSError(f"libusb_init failed: {self._error(rc)}")
        self._handles = {}   # libusb_device pointer -> open handle (kept until the process exits)

    @classmethod
    def for_pyrealsense(cls):
        """A capturer on macOS; None elsewhere, where librealsense's backends don't need it."""
        if sys.platform != "darwin":
            return None
        import pyrealsense2
        path = Path(pyrealsense2.__file__).parent / ".dylibs" / "libusb-1.0.0.dylib"
        return cls(path) if path.exists() else None

    def capture_new(self):
        """Capture every RealSense not captured yet. Returns [(vid:pid, error or None)]."""
        devices = ctypes.POINTER(ctypes.c_void_p)()
        count = self._lib.libusb_get_device_list(self._ctx, ctypes.byref(devices))
        if count < 0:
            return [("device list", self._error(count))]
        results = []
        try:
            for i in range(count):
                dev = devices[i]
                if dev in self._handles:
                    continue
                desc = _DeviceDescriptor()
                if self._lib.libusb_get_device_descriptor(dev, ctypes.byref(desc)) != 0:
                    continue
                if (desc.idVendor == INTEL_VID and desc.bDeviceClass == MISC_DEVICE_CLASS
                        and desc.idProduct not in RECOVERY_PIDS):
                    results.append((f"{desc.idVendor:04x}:{desc.idProduct:04x}", self._capture(dev)))
        finally:
            self._lib.libusb_free_device_list(devices, 1)   # our open handles keep their own refs
        return results

    def _capture(self, dev):
        handle = ctypes.c_void_p()
        rc = self._lib.libusb_open(dev, ctypes.byref(handle))
        if rc != 0:
            return self._error(rc)
        rc = self._lib.libusb_detach_kernel_driver(handle, 0)   # the one re-enumeration, with capture
        if rc != 0:
            self._lib.libusb_close(handle)
            return self._error(rc)
        for _ in range(CAPTURE_HOLD - 1):   # already captured: these only raise the count
            if self._lib.libusb_detach_kernel_driver(handle, 0) != 0:
                break
        self._handles[dev] = handle
        return None

    def _error(self, rc):
        return self._lib.libusb_error_name(rc).decode()
