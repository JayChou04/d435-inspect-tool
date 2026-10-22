#!/usr/bin/env python3
"""Intel RealSense D435 inspector: health checks plus an OpenCV dashboard.

    ./run-mac.sh  or  ./run-linux.sh              # GUI dashboard
    ./run-mac.sh --check                          # text report, exit code 1 on FAIL
    ./run-mac.sh --list                           # list connected RealSense devices

The launchers set up .venv, report the camera's USB port and add sudo when the camera needs root.
"""

from __future__ import annotations   # lets the annotations below work on Python 3.9

import sys

# Under sudo, cached bytecode would be written as root and block a plain `rm -rf` of this folder.
sys.dont_write_bytecode = True

import argparse  # noqa: E402
import json  # noqa: E402
import os  # noqa: E402
import re  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
import traceback  # noqa: E402
from collections import deque  # noqa: E402
from dataclasses import dataclass, field  # noqa: E402
from datetime import datetime  # noqa: E402
from functools import partial  # noqa: E402
from pathlib import Path  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pyrealsense2 as rs  # noqa: E402

from firmware_update import KNOWN_IMAGES, FirmwareUpdater  # noqa: E402
from macos_usb_capture import UsbCapture  # noqa: E402
from usb_ports import camera_for  # noqa: E402

# --------------------------------------------------------------------------- settings

WINDOW = "D435 Inspector"
SNAPSHOT_DIR = Path(__file__).resolve().parent / "snapshots"
FIRMWARE_DIR = Path(__file__).resolve().parent / "firmware"
DEFAULT_MODE = (640, 480, 30)
FALLBACK_MODES = [(640, 480, 30), (848, 480, 30), (640, 480, 15), (424, 240, 30),
                  (424, 240, 15), (640, 480, 6), (424, 240, 6)]

# Health thresholds. Scene-dependent ones (depth fill, brightness) only ever WARN.
WARMUP_SEC = 2.0            # fps/drop stats are not judged right after streams start
FPS_WINDOW_SEC = 2.0
FPS_TOLERANCE = 0.10        # measured fps may be 10% under target
DROP_WARN, DROP_FAIL = 0.01, 0.05
STALE_SEC, LOST_SEC = 1.0, 3.0
STARTUP_GRACE_SEC = 5.0     # hardware errors this soon after stream start only WARN
FROZEN_FRAMES = 30          # this many identical consecutive frames = frozen stream
DEPTH_FILL_WARN = 0.50
IR_BALANCE_WARN = 0.30      # relative brightness difference between left and right imager
TEMP_WARN_C, TEMP_FAIL_C = 55.0, 70.0
CALIB_GOOD, CALIB_FAIR = 0.25, 0.75   # Intel self-calibration health-check bands
DEPTH_RANGE_M = (0.2, 4.0)  # colormap range
CENTER_ROI = 10             # half size of the centre depth ROI in pixels
EXPECTED_SENSORS = {"Stereo Module", "RGB Camera"}

OCC_CONFIG = {"calib type": 0, "speed": 2, "scan parameter": 0,
              "adjust both sides": 0, "white wall mode": 0}
OCC_TIMEOUT_MS = 30000

OK, WARN, FAIL, PEND, INFO = "OK", "WARN", "FAIL", "WAIT", "INFO"
RANK = {INFO: 0, OK: 1, PEND: 2, WARN: 3, FAIL: 4}

STREAMING, CALIBRATING, RESETTING, DISCONNECTED = "STREAMING", "CALIBRATING", "RESETTING", "DISCONNECTED"

INFO_FIELDS = [
    ("name", rs.camera_info.name),
    ("serial", rs.camera_info.serial_number),
    ("firmware", rs.camera_info.firmware_version),
    ("recommended_fw", rs.camera_info.recommended_firmware_version),
    ("usb", rs.camera_info.usb_type_descriptor),
    ("product_line", rs.camera_info.product_line),
    ("product_id", rs.camera_info.product_id),
    ("port", rs.camera_info.physical_port),
    ("asic_serial", rs.camera_info.asic_serial_number),
]
LIVE_OPTIONS = [
    ("asic_temp", rs.option.asic_temperature),
    ("projector_temp", rs.option.projector_temperature),
    ("emitter", rs.option.emitter_enabled),
    ("laser_power", rs.option.laser_power),
]
METADATA_FIELDS = [
    ("exposure", rs.frame_metadata_value.actual_exposure),
    ("gain", rs.frame_metadata_value.gain_level),
    ("laser_power", rs.frame_metadata_value.frame_laser_power),
    ("emitter_mode", rs.frame_metadata_value.frame_emitter_mode),
]


@dataclass(frozen=True)
class StreamSpec:
    key: str
    label: str
    sensor: str          # "depth" (stereo module) or "color" (RGB camera)
    stream: rs.stream
    index: int
    fmt: rs.format


STREAM_SPECS = (
    StreamSpec("color", "Color", "color", rs.stream.color, 0, rs.format.bgr8),
    StreamSpec("depth", "Depth", "depth", rs.stream.depth, 0, rs.format.z16),
    StreamSpec("ir1", "IR Left", "depth", rs.stream.infrared, 1, rs.format.y8),
    StreamSpec("ir2", "IR Right", "depth", rs.stream.infrared, 2, rs.format.y8),
)


@dataclass
class Check:
    name: str
    status: str
    detail: str


@dataclass
class StreamState:
    spec: StreamSpec
    profile: rs.video_stream_profile | None
    image: np.ndarray | None
    frame_no: int | None
    fps: float
    received: int
    dropped: int
    age: float | None    # seconds since the last frame, None if no frame yet
    uptime: float        # seconds since this stream was started
    frozen: bool
    metadata: dict = field(default_factory=dict)

    @property
    def target_fps(self):
        return self.profile.fps() if self.profile else 0

    @property
    def drop_rate(self):
        total = self.received + self.dropped
        return self.dropped / total if total else 0.0

    @property
    def mode(self):
        if not self.profile:
            return "-"
        p = self.profile
        return f"{p.width()}x{p.height()} {p.format().name.upper()}"


class DeviceAccessError(RuntimeError):
    pass


ACCESS_HINT = """Cannot open the RealSense device: {err}

Start the tool with ./run-mac.sh or ./run-linux.sh: they run it with sudo when the camera needs
root (always on macOS; on Linux unless the RealSense udev rules are installed, see README).
Also close any other program using the camera (realsense-viewer, browser, video calls...)."""


# --------------------------------------------------------------------------- helpers

def safe(fn, *args):
    try:
        return fn(*args)
    except RuntimeError:
        return None


def version_tuple(text):
    return tuple(int(n) for n in re.findall(r"\d+", text or ""))


def worst(statuses):
    return max(statuses, key=RANK.__getitem__, default=INFO)


def query_devices(ctx):
    """Return [(serial, device)] for every connected RealSense device."""
    found = []
    devices = ctx.query_devices()
    for i in range(len(devices)):
        try:
            dev = devices[i]
            if dev.is_update_device():   # firmware-update mode, handled by firmware_update.py
                continue
            found.append((dev.get_info(rs.camera_info.serial_number), dev))
        except RuntimeError as e:
            raise DeviceAccessError(str(e)) from e
    return found


def read_info(dev):
    info = {key: dev.get_info(ci) for key, ci in INFO_FIELDS if dev.supports(ci)}
    usb = camera_for(info.get("asic_serial"))   # the USB serial is the camera's ASIC serial
    if usb is not None:
        info["usb_port"] = usb.port
    return info


def find_profile(profiles, spec, width, height, fps):
    for p in profiles:
        if (p.stream_type() == spec.stream and p.format() == spec.fmt
                and (spec.stream != rs.stream.infrared or p.stream_index() == spec.index)
                and (p.width(), p.height(), p.fps()) == (width, height, fps)):
            return p
    return None


def candidate_configs(sensor, specs, modes):
    """Yield (mode, [(spec, profile)]) best first: every stream at one mode, then partial sets."""
    profiles = [p.as_video_stream_profile() for p in sensor.get_stream_profiles()
                if p.is_video_stream_profile()]
    for need_all in (True, False):
        for mode in modes:
            found = [(s, p) for s in specs if (p := find_profile(profiles, s, *mode)) is not None]
            if found and (len(found) == len(specs)) == need_all:
                yield mode, found


def give_back_to_sudo_user(*paths):
    """Files written under sudo belong to root; hand them back so the folder stays deletable."""
    uid, gid = os.environ.get("SUDO_UID"), os.environ.get("SUDO_GID")
    if os.geteuid() != 0 or not uid or not gid:
        return
    for path in paths:
        try:
            os.chown(path, int(uid), int(gid))
        except OSError:
            pass


def colorize_depth(depth, scale):
    near, far = DEPTH_RANGE_M
    meters = depth.astype(np.float32) * scale
    norm = np.clip((meters - near) / (far - near), 0.0, 1.0)
    colored = cv2.applyColorMap((255 - norm * 255).astype(np.uint8), cv2.COLORMAP_JET)  # near = red
    colored[depth == 0] = 0
    return colored


# --------------------------------------------------------------------------- streaming

class StreamMonitor:
    """Collects the frames of one stream; on_frame runs on a librealsense thread."""

    def __init__(self, spec):
        self.spec = spec
        self._lock = threading.Lock()
        self.reset()

    def reset(self, profile=None):
        with self._lock:
            self.profile = profile
            self.image = None
            self.frame_no = None
            self.received = 0
            self.dropped = 0
            self.repeats = 0
            self.uniform = True
            self.metadata = {}
            self.arrivals = deque(maxlen=400)
            self.started = time.monotonic()
            self._thumb = None

    def on_frame(self, frame):
        now = time.monotonic()
        image = np.asanyarray(frame.get_data()).copy()
        number = frame.get_frame_number()
        thumb = image[::16, ::16]
        metadata = {name: frame.get_frame_metadata(m) for name, m in METADATA_FIELDS
                    if frame.supports_frame_metadata(m)}
        with self._lock:
            if self.frame_no is not None and 0 < number - self.frame_no - 1 < 1000:
                self.dropped += number - self.frame_no - 1
            same = self._thumb is not None and np.array_equal(thumb, self._thumb)
            self.repeats = self.repeats + 1 if same else 0
            self.uniform = float(thumb.std()) < 2.0
            self._thumb = thumb
            self.frame_no = number
            self.image = image
            self.metadata = metadata
            self.received += 1
            self.arrivals.append(now)

    def state(self):
        with self._lock:
            now = time.monotonic()
            recent = [t for t in self.arrivals if now - t <= FPS_WINDOW_SEC]
            fps = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 2 else 0.0
            return StreamState(
                spec=self.spec, profile=self.profile, image=self.image, frame_no=self.frame_no,
                fps=fps, received=self.received, dropped=self.dropped,
                age=now - self.arrivals[-1] if self.arrivals else None,
                uptime=now - self.started,
                frozen=self.repeats >= FROZEN_FRAMES and not self.uniform,
                metadata=dict(self.metadata),
            )


class Inspector:
    """Owns the camera: streaming, reconnection, live health values and the check list."""

    def __init__(self, ctx, serial=None, mode=DEFAULT_MODE, usb_capture=None):
        self.ctx = ctx
        self.usb_capture = usb_capture
        self.ctx.set_devices_changed_callback(self._on_devices_changed)
        self.target_serial = serial
        self.mode = mode
        self.dev = None
        self.info = {}
        self.sensor_names = []
        self.sensors = {}
        self.depth_scale = 0.001
        self.monitors = {s.key: StreamMonitor(s) for s in STREAM_SPECS}
        self.modes = {}
        self.state = DISCONNECTED
        self.message = "not connected"
        self.events = deque(maxlen=200)
        self.notification_counts = {"hardware_error": 0, "startup_error": 0, "error": 0, "warn": 0}
        self._streams_started_at = 0.0
        self.last_notification = ""
        self.live = {}
        self.analysis = {}
        self.calib = {"state": "idle"}
        self._active = []
        self._removed = False
        self._callback_error = None
        self._last_connect_error = None
        self._next_attempt = 0.0
        self._next_poll = 0.0
        self._next_analysis = 0.0
        self._next_presence_check = 0.0
        self._depth_roi = deque(maxlen=15)
        self._last_depth_no = None

    # ---- connection

    def connect(self):
        """Open the target device and start streaming; False if it is not plugged in."""
        if self.usb_capture is not None:   # a replugged or reset camera is a new USB device
            for device, error in self.usb_capture.capture_new():
                self._event(WARN if error else INFO, f"macOS USB capture {device}: {error or 'ok'}")
        match = next(((s, d) for s, d in query_devices(self.ctx)
                      if self.target_serial in (None, s)), None)
        if match is None:
            self.message = (f"device {self.target_serial} not found" if self.target_serial
                            else "no RealSense device found")
            return False
        self.target_serial, self.dev = match
        self.info = read_info(self.dev)
        self.sensor_names = [s.get_info(rs.camera_info.name) for s in self.dev.query_sensors()]
        self.sensors = {}
        for name, getter in (("depth", self.dev.first_depth_sensor),
                             ("color", self.dev.first_color_sensor)):
            sensor = safe(getter)
            if sensor is not None:
                self.sensors[name] = sensor
                sensor.set_notifications_callback(partial(self._on_notification, name))
        if "depth" in self.sensors:
            self.depth_scale = self.sensors["depth"].get_depth_scale()
        self._removed = False
        self._start_streams()
        if not self._active:
            self.dev = None
            self.message = "device found but no stream could be started"
            return False
        self.state, self.message = STREAMING, ""
        self._last_connect_error = None
        self._event(OK, f"connected {self.info.get('name', '?')} S/N {self.target_serial} "
                        f"FW {self.info.get('firmware', '?')} USB {self.info.get('usb', '?')}")
        return True

    def close(self):
        self._stop_streams()
        self.dev = None

    def _mode_order(self):
        return [self.mode] + [m for m in FALLBACK_MODES if m != self.mode]

    def _start_streams(self):
        # Querying profiles, open and close power a sensor's USB interface up or down, and on macOS
        # every such transition re-enumerates the whole camera: a stream already running breaks
        # ("Depth stream start failure") or librealsense crashes. So, like rs.pipeline: query
        # every sensor first, then open all, and only then start all.
        for monitor in self.monitors.values():
            monitor.reset()
        self._active, self.modes = [], {}
        plans = {name: list(candidate_configs(sensor, [s for s in STREAM_SPECS if s.sensor == name],
                                              self._mode_order()))
                 for name, sensor in self.sensors.items()}
        opened = []
        for name, sensor in self.sensors.items():
            for mode, chosen in plans[name]:
                try:
                    sensor.open([p for _, p in chosen])
                except RuntimeError:
                    continue
                opened.append((name, sensor, mode, chosen))
                break
            else:
                self._event(FAIL, f"{name} sensor: no stream mode could be opened")
        for name, sensor, mode, chosen in opened:
            for spec, profile in chosen:
                self.monitors[spec.key].reset(profile)
            try:
                sensor.start(self._on_frame)
            except RuntimeError as e:
                for spec, _ in chosen:
                    self.monitors[spec.key].reset()
                safe(sensor.close)
                self._event(FAIL, f"{name} sensor failed to start: {e}")
                continue
            self._active.append(sensor)
            self.modes[name] = mode
        self._streams_started_at = time.monotonic()
        self._depth_roi.clear()

    def _stop_streams(self):
        # Stop everything before closing anything, for the same reason as in _start_streams.
        for sensor in self._active:
            safe(sensor.stop)
        for sensor in self._active:
            safe(sensor.close)
        self._active = []

    # ---- librealsense callbacks (not on the GUI thread)

    def _on_frame(self, frame):
        try:
            frames = list(frame.as_frameset()) if frame.is_frameset() else [frame]
            for f in frames:
                p = f.get_profile()
                for spec in STREAM_SPECS:
                    if p.stream_type() == spec.stream and (
                            spec.stream != rs.stream.infrared or p.stream_index() == spec.index):
                        self.monitors[spec.key].on_frame(f)
                        break
        except Exception as e:  # an exception must never escape into librealsense's thread
            if self._callback_error is None:
                self._callback_error = repr(e)
                traceback.print_exc()
                self._event(WARN, f"frame callback error: {e!r}")

    def _on_notification(self, sensor_name, notification):
        severity = notification.severity.name
        category = notification.category.name
        if category == "hardware_error":
            # Firmware 5.12 on macOS can report "Depth stream start failure" and then stream
            # normally; the stream checks judge the streams, so only later errors FAIL here.
            starting = time.monotonic() - self._streams_started_at <= STARTUP_GRACE_SEC
            self.notification_counts["startup_error" if starting else "hardware_error"] += 1
            level = WARN if starting else FAIL
        elif severity in ("error", "fatal"):
            self.notification_counts["error"] += 1
            level = FAIL
        elif severity == "warn":
            self.notification_counts["warn"] += 1
            level = WARN
        else:
            level = INFO
        self.last_notification = f"{category}: {notification.description}"
        self._event(level, f"[{sensor_name}] {self.last_notification}")

    def _on_devices_changed(self, info):
        dev = self.dev
        try:
            if dev is not None and info.was_removed(dev):
                self._removed = True
        except RuntimeError:
            pass

    def _event(self, level, text):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.events.append((stamp, level, text))
        print(f"{stamp} {level:<4} {text}", flush=True)

    # ---- periodic work on the caller's thread

    def tick(self):
        now = time.monotonic()
        if self.state == STREAMING:
            if self._removed or self._looks_unplugged(now):
                self._stop_streams()
                self.dev, self.live, self.analysis = None, {}, {}
                self.state, self.message = DISCONNECTED, "device disconnected"
                self._event(FAIL, "device disconnected - waiting for it to come back")
                self._next_attempt = now + 1.0
                return
            if now >= self._next_poll:
                self._poll_options()
                self._next_poll = now + 1.0
            if now >= self._next_analysis:
                self._analyze()
                self._next_analysis = now + 0.2
        elif self.state in (DISCONNECTED, RESETTING) and now >= self._next_attempt:
            self._next_attempt = now + 2.0
            try:
                self.connect()
            except RuntimeError as e:   # e.g. device still re-enumerating
                self.report_connect_error(e)

    def report_connect_error(self, error):
        """Log each distinct connection error once, with its traceback, then keep retrying."""
        text = f"connect failed: {error}"
        if text != self._last_connect_error:
            self._last_connect_error = text
            traceback.print_exception(type(error), error, error.__traceback__)
            self._event(FAIL, text)
        self.message = f"waiting for device ({error})"

    def _looks_unplugged(self, now):
        started = [m for m in self.monitors.values() if m.profile is not None]
        if not started or now < self._next_presence_check:
            return False
        states = [m.state() for m in started]
        if not all((s.age if s.age is not None else s.uptime) > LOST_SEC for s in states):
            return False
        self._next_presence_check = now + 2.0
        return not safe(self.dev.is_connected)

    def _poll_options(self):
        depth = self.sensors.get("depth")
        live = {}
        for key, option in LIVE_OPTIONS:
            if depth is not None and depth.supports(option):
                value = safe(depth.get_option, option)
                if value is not None:
                    live[key] = value
        self.live = live

    def _analyze(self):
        a = {}
        depth = self.monitors["depth"].state()
        if depth.image is not None:
            d = depth.image
            a["depth_fill"] = np.count_nonzero(d) / d.size
            h, w = d.shape
            roi = d[h // 2 - CENTER_ROI:h // 2 + CENTER_ROI, w // 2 - CENTER_ROI:w // 2 + CENTER_ROI]
            valid = roi[roi > 0]
            if valid.size >= roi.size // 2:
                a["center_m"] = float(np.median(valid)) * self.depth_scale
            if depth.frame_no != self._last_depth_no:
                self._last_depth_no = depth.frame_no
                self._depth_roi.append(roi.astype(np.float32))
            if len(self._depth_roi) >= 5:
                stack = np.stack(self._depth_roi)
                always_valid = (stack > 0).all(axis=0)
                if always_valid.sum() >= always_valid.size // 2:
                    noise = stack[:, always_valid].std(axis=0).mean()
                    a["center_noise_mm"] = float(noise) * self.depth_scale * 1000
        for key in ("ir1", "ir2"):
            image = self.monitors[key].state().image
            if image is not None:
                a[key] = {"mean": float(image.mean()), "std": float(image.std())}
        color = self.monitors["color"].state().image
        if color is not None:
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            a["color"] = {
                "mean": float(gray.mean()),
                "std": float(gray.std()),
                "saturated": np.count_nonzero(gray >= 250) / gray.size,
                "sharpness": float(cv2.Laplacian(gray, cv2.CV_64F).var()),
            }
        self.analysis = a

    # ---- user actions

    def toggle_emitter(self):
        depth = self.sensors.get("depth")
        if self.state != STREAMING or depth is None or not depth.supports(rs.option.emitter_enabled):
            return
        on = safe(depth.get_option, rs.option.emitter_enabled)
        if on is None:
            return
        safe(depth.set_option, rs.option.emitter_enabled, 0.0 if on else 1.0)
        self._event(INFO, f"emitter {'OFF' if on else 'ON'}")
        self._poll_options()

    def hardware_reset(self):
        if self.state != STREAMING:
            return
        self._event(WARN, "hardware reset requested")
        self._stop_streams()
        try:
            self.dev.hardware_reset()
        except RuntimeError as e:
            self._event(FAIL, f"hardware reset failed: {e}")
        self.dev, self.live, self.analysis = None, {}, {}
        self.state, self.message = RESETTING, "hardware reset, waiting for device"
        self._next_attempt = time.monotonic() + 3.0

    def start_calibration_check(self):
        """Run Intel's on-chip self-calibration only to read its health number; the new table is discarded."""
        if self.state != STREAMING or "depth" not in self.sensors:
            return False
        if not self.dev.is_auto_calibrated_device():
            self.calib = {"state": "error", "error": "device does not support self-calibration"}
            return False
        self.state = CALIBRATING
        self.calib = {"state": "running", "progress": 0.0}
        self._event(INFO, "calibration health check started - keep the camera still, aim at a textured scene")
        threading.Thread(target=self._calibration_worker, daemon=True).start()
        return True

    def _calibration_profile(self, depth):
        profiles = [p.as_video_stream_profile() for p in depth.get_stream_profiles()
                    if p.is_video_stream_profile()]
        spec = self.monitors["depth"].spec
        current = self.modes.get("depth", self.mode)
        for mode in [(256, 144, 90), current, DEFAULT_MODE]:
            profile = find_profile(profiles, spec, *mode)
            if profile is not None:
                return profile
        raise RuntimeError("no depth profile usable for self-calibration")

    def _calibration_worker(self):
        depth = self.sensors["depth"]
        result = {"state": "error", "error": "unknown"}
        emitter = None
        try:
            self._stop_streams()
            acd = self.dev.as_auto_calibrated_device()
            original = acd.get_calibration_table()
            if depth.supports(rs.option.emitter_enabled):
                emitter = depth.get_option(rs.option.emitter_enabled)
                depth.set_option(rs.option.emitter_enabled, 1.0)
            depth.open(self._calibration_profile(depth))
            depth.start(lambda frame: None)
            try:
                _, health = acd.run_on_chip_calibration(
                    json.dumps(OCC_CONFIG), self._on_calibration_progress, OCC_TIMEOUT_MS)
            finally:
                safe(depth.stop)
                safe(depth.close)
                try:   # inspection only: put the original table back, nothing is written to flash
                    acd.set_calibration_table(original)
                except RuntimeError as e:
                    self._event(FAIL, f"could not restore calibration table ({e}); "
                                      "press R to reload it from flash")
            value = abs(health[0])
            result = {"state": "done", "health": value, "at": datetime.now().strftime("%H:%M:%S")}
            self._event(OK if value < CALIB_GOOD else WARN if value < CALIB_FAIR else FAIL,
                        f"calibration health-check = {value:.3f}")
        except Exception as e:
            result = {"state": "error", "error": str(e)}
            self._event(WARN, f"calibration check failed: {e}")
        finally:
            if emitter is not None:
                safe(depth.set_option, rs.option.emitter_enabled, emitter)
            self.calib = result
            try:
                self._start_streams()
                self.state = STREAMING
            except RuntimeError as e:
                self.dev = None
                self.state, self.message = DISCONNECTED, f"restart after calibration failed: {e}"

    def _on_calibration_progress(self, percent):
        self.calib = {"state": "running", "progress": float(percent)}

    # ---- results

    def stream_states(self):
        return {key: monitor.state() for key, monitor in self.monitors.items()}

    def _stream_check(self, st):
        if self.state == CALIBRATING:
            return PEND, "paused for calibration check"
        if self.state != STREAMING:
            return FAIL, "not streaming"
        if st.profile is None:
            return FAIL, "not started (no compatible mode)"
        if st.received == 0:
            return (FAIL, "no frames received") if st.uptime > LOST_SEC else (PEND, "waiting for frames")
        if st.age > LOST_SEC:
            return FAIL, f"no frames for {st.age:.1f}s"
        if st.frozen:
            return FAIL, "image frozen (identical frames)"
        if st.age > STALE_SEC:
            return WARN, f"stalled {st.age:.1f}s"
        if st.uptime < WARMUP_SEC:
            return PEND, "warming up"
        detail = f"{st.fps:.1f}/{st.target_fps} fps, drop {st.drop_rate * 100:.1f}%"
        if st.drop_rate >= DROP_FAIL:
            return FAIL, detail
        if st.drop_rate >= DROP_WARN or st.fps < st.target_fps * (1 - FPS_TOLERANCE):
            return WARN, detail
        return OK, detail

    def checks(self, states=None):
        states = states or self.stream_states()
        a = self.analysis
        out = []

        def add(name, status, detail):
            out.append(Check(name, status, detail))

        connection = {
            STREAMING: (OK, "streaming"),
            CALIBRATING: (PEND, "calibration check running"),
            RESETTING: (PEND, "hardware reset, waiting for device"),
        }.get(self.state, (FAIL, self.message or "disconnected"))
        add("Connection", *connection)

        usb = self.info.get("usb")
        if not usb:
            add("USB link", INFO, "unknown")
        elif usb.startswith("3"):
            add("USB link", OK, f"USB {usb}")
        else:
            add("USB link", WARN, f"USB {usb}: use a USB 3 port/cable")

        fw, rec = self.info.get("firmware"), self.info.get("recommended_fw")
        if not fw:
            add("Firmware", INFO, "unknown")
        elif rec and version_tuple(fw) < version_tuple(rec):
            add("Firmware", WARN, f"{fw} < recommended {rec}")
        else:
            add("Firmware", OK, fw)

        if self.sensor_names:
            missing = EXPECTED_SENSORS - set(self.sensor_names)
            add("Sensors", FAIL if missing else OK,
                f"missing {', '.join(sorted(missing))}" if missing else ", ".join(self.sensor_names))
        else:
            add("Sensors", INFO, "unknown")

        for spec in STREAM_SPECS:
            add(f"{spec.label} stream", *self._stream_check(states[spec.key]))

        live_ok = self.state == STREAMING
        fill = a.get("depth_fill") if live_ok else None
        if fill is None:
            add("Depth quality", INFO, "no data")
        elif fill < 0.01:
            add("Depth quality", FAIL, "no valid depth (lens covered / emitter off?)")
        elif fill < DEPTH_FILL_WARN:
            add("Depth quality", WARN, f"valid pixels {fill:.0%} (scene too close/far?)")
        else:
            add("Depth quality", OK, f"valid pixels {fill:.0%}")

        left, right = (a.get("ir1"), a.get("ir2")) if live_ok else (None, None)
        if not left or not right:
            add("IR imagers", INFO, "no data")
        else:
            black = [n for n, v in (("left", left), ("right", right)) if v["mean"] < 2 and v["std"] < 1.5]
            diff = abs(left["mean"] - right["mean"]) / max(left["mean"], right["mean"], 1e-6)
            balance = f"L {left['mean']:.0f} / R {right['mean']:.0f}"
            if black:
                add("IR imagers", FAIL, f"{' and '.join(black)} image black")
            elif diff > IR_BALANCE_WARN:
                add("IR imagers", WARN, f"{balance} ({diff:.0%} apart, lens dirty?)")
            else:
                add("IR imagers", OK, balance)

        color = a.get("color") if live_ok else None
        if not color:
            add("Color image", INFO, "no data")
        elif color["mean"] < 8 and color["std"] < 4:
            add("Color image", WARN, "almost black (lens covered / dark?)")
        elif color["saturated"] > 0.4:
            add("Color image", WARN, f"overexposed {color['saturated']:.0%}")
        else:
            add("Color image", OK, f"brightness {color['mean']:.0f}, sharpness {color['sharpness']:.0f}")

        temps = {k: self.live[k] for k in ("asic_temp", "projector_temp") if k in self.live}
        if not temps or not live_ok:
            add("Temperature", INFO, "not reported")
        else:
            hottest = max(temps.values())
            detail = ", ".join(f"{k.split('_')[0].upper()} {v:.0f}C" for k, v in temps.items())
            add("Temperature", FAIL if hottest >= TEMP_FAIL_C else WARN if hottest >= TEMP_WARN_C else OK, detail)

        counts = self.notification_counts
        if counts["hardware_error"]:
            add("HW notifications", FAIL, f"{counts['hardware_error']} hardware errors: {self.last_notification}")
        elif counts["startup_error"]:
            add("HW notifications", WARN, f"{counts['startup_error']} at stream start: {self.last_notification}")
        elif counts["error"] or counts["warn"]:
            add("HW notifications", WARN, f"{counts['error']} err / {counts['warn']} warn: {self.last_notification}")
        else:
            add("HW notifications", OK, "none")

        calib = self.calib
        if calib["state"] == "idle":
            add("Calibration", INFO, "press C to run health check")
        elif calib["state"] == "running":
            add("Calibration", PEND, f"running {calib['progress']:.0f}%")
        elif calib["state"] == "error":
            add("Calibration", WARN, f"check failed: {calib['error']}")
        else:
            h = calib["health"]
            if h < CALIB_GOOD:
                add("Calibration", OK, f"health {h:.2f} (good)")
            elif h < CALIB_FAIR:
                add("Calibration", WARN, f"health {h:.2f} (could be improved)")
            else:
                add("Calibration", FAIL, f"health {h:.2f} (recalibration recommended)")
        return out

    def report(self):
        states = self.stream_states()
        checks = self.checks(states)
        return {
            "time": datetime.now().isoformat(timespec="seconds"),
            "device": self.info,
            "state": self.state,
            "modes": {k: f"{w}x{h}@{fps}" for k, (w, h, fps) in self.modes.items()},
            "overall": worst(c.status for c in checks),
            "checks": [vars(c) for c in checks],
            "streams": {
                k: {"label": s.spec.label, "mode": s.mode, "fps": round(s.fps, 2), "target_fps": s.target_fps,
                    "received": s.received, "dropped": s.dropped, "drop_rate": round(s.drop_rate, 4)}
                for k, s in states.items()
            },
            "live": self.live,
            "analysis": self.analysis,
            "calibration": self.calib,
            "events": [{"time": t, "level": lv, "text": tx} for t, lv, tx in list(self.events)],
        }

    def save_snapshot(self, dashboard=None):
        folder = SNAPSHOT_DIR / datetime.now().strftime("%Y%m%d_%H%M%S")
        folder.mkdir(parents=True, exist_ok=True)
        states = self.stream_states()
        images = {
            "dashboard.png": dashboard,
            "color.png": states["color"].image,
            "depth_raw_16bit.png": states["depth"].image,
            "depth_color.png": (colorize_depth(states["depth"].image, self.depth_scale)
                                if states["depth"].image is not None else None),
            "ir_left.png": states["ir1"].image,
            "ir_right.png": states["ir2"].image,
        }
        for name, image in images.items():
            if image is not None:
                cv2.imwrite(str(folder / name), image)
        (folder / "report.json").write_text(json.dumps(self.report(), indent=2, default=str))
        give_back_to_sudo_user(SNAPSHOT_DIR, folder, *folder.iterdir())
        self._event(INFO, f"snapshot saved: {folder}")
        return folder


# --------------------------------------------------------------------------- GUI

TILE_W, TILE_H, HEADER_H = 448, 336, 26
GRID_W, GRID_H = 2 * TILE_W, 2 * (TILE_H + HEADER_H)
PANEL_W = 440
CANVAS_W, CANVAS_H = GRID_W + PANEL_W, GRID_H
TILE_POS = {"color": (0, 0), "depth": (1, 0), "ir1": (0, 1), "ir2": (1, 1)}

FONT = cv2.FONT_HERSHEY_SIMPLEX
BG, PANEL_BG, HEADER_BG, BORDER = (22, 22, 26), (34, 34, 40), (52, 52, 60), (70, 70, 78)
TEXT, DIM, SECTION = (235, 235, 235), (165, 165, 170), (230, 190, 120)
STATUS_BGR = {OK: (90, 200, 90), WARN: (40, 180, 250), FAIL: (80, 80, 240),
              PEND: (230, 170, 90), INFO: (150, 150, 150)}


def put(img, text, org, color=TEXT, scale=0.45, thickness=1):
    cv2.putText(img, text, (int(org[0]), int(org[1])), FONT, scale, color, thickness, cv2.LINE_AA)


def text_width(text, scale=0.45, thickness=1):
    return cv2.getTextSize(text, FONT, scale, thickness)[0][0]


def clip_text(text, max_width, scale=0.45):
    if text_width(text, scale) <= max_width:
        return text
    while text and text_width(text + "...", scale) > max_width:
        text = text[:-1]
    return text + "..."


def draw_badge(img, status, right, top, scale=0.42):
    """Draw a status pill whose right edge is at `right`; returns its left edge."""
    w = text_width(status, scale, 1) + 14
    left = right - w
    cv2.rectangle(img, (left, top), (right, top + 18), STATUS_BGR[status], -1)
    put(img, status, (left + 7, top + 13), (20, 20, 20), scale, 1)
    return left


def label_box(img, text, org, scale=0.42, color=TEXT):
    """Text on a dark background box so it stays readable over any image."""
    w = text_width(text, scale)
    x, y = int(org[0]), int(org[1])
    cv2.rectangle(img, (x - 4, y - 14), (x + w + 4, y + 5), (0, 0, 0), -1)
    put(img, text, (x, y), color, scale)


def fit_into(image, width, height):
    ih, iw = image.shape[:2]
    scale = min(width / iw, height / ih)
    size = (max(1, int(iw * scale)), max(1, int(ih * scale)))
    resized = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
    return resized, scale, (width - size[0]) // 2, (height - size[1]) // 2


class Dashboard:
    def __init__(self, inspector):
        self.insp = inspector
        self.mouse = None
        self.placements = {}   # stream key -> (x0, y0, scale, image_w, image_h) on the canvas
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, CANVAS_W, CANVAS_H)
        cv2.setMouseCallback(WINDOW, self._on_mouse)

    def _on_mouse(self, event, x, y, flags, param):
        self.mouse = (x, y)

    def run(self):
        while True:
            self.insp.tick()
            canvas = self.render()
            cv2.imshow(WINDOW, canvas)
            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("e"), ord("E")):
                self.insp.toggle_emitter()
            elif key in (ord("c"), ord("C")):
                self.insp.start_calibration_check()
            elif key in (ord("s"), ord("S")):
                self.insp.save_snapshot(canvas)
            elif key in (ord("r"), ord("R")):
                self.insp.hardware_reset()
            if not self._window_open():
                break
        cv2.destroyAllWindows()

    @staticmethod
    def _window_open():
        try:
            return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1
        except cv2.error:   # macOS raises once the window has been closed
            return False

    # ---- drawing

    def render(self):
        canvas = np.full((CANVAS_H, CANVAS_W, 3), BG, np.uint8)
        states = self.insp.stream_states()
        checks = self.insp.checks(states)
        by_name = {c.name: c for c in checks}
        self.placements = {}
        for spec in STREAM_SPECS:
            self._draw_tile(canvas, states[spec.key], by_name[f"{spec.label} stream"])
        self._draw_hover(canvas, states)
        self._draw_panel(canvas, checks)
        return canvas

    def _display_image(self, st):
        if st.image is None:
            return None
        if st.spec.key == "depth":
            return colorize_depth(st.image, self.insp.depth_scale)
        if st.image.ndim == 2:
            return cv2.cvtColor(st.image, cv2.COLOR_GRAY2BGR)
        return st.image

    def _tile_message(self, st):
        insp = self.insp
        if insp.state == DISCONNECTED:
            return "DISCONNECTED"
        if insp.state == RESETTING:
            return "RESETTING..."
        if insp.state == CALIBRATING:
            return f"CALIBRATION CHECK {insp.calib.get('progress', 0):.0f}%"
        if st.profile is None:
            return "NOT STREAMING"
        if st.received == 0:
            return "WAITING FOR FRAMES"
        if st.age > LOST_SEC:
            return "NO SIGNAL"
        if st.frozen:
            return "FROZEN"
        if st.age > STALE_SEC:
            return "STALLED"
        return None

    def _draw_tile(self, canvas, st, check):
        spec = st.spec
        col, row = TILE_POS[spec.key]
        x, y = col * TILE_W, row * (TILE_H + HEADER_H)
        body_y = y + HEADER_H

        cv2.rectangle(canvas, (x, y), (x + TILE_W - 1, body_y - 1), HEADER_BG, -1)
        cv2.rectangle(canvas, (x, y), (x + 4, body_y - 1), STATUS_BGR[check.status], -1)
        put(canvas, spec.label, (x + 12, y + 18), TEXT, 0.55)
        put(canvas, st.mode, (x + 100, y + 18), DIM, 0.42)
        badge_left = draw_badge(canvas, check.status, x + TILE_W - 6, y + 4)
        if st.profile is not None and self.insp.state == STREAMING:
            stats = f"{st.fps:4.1f}/{st.target_fps} fps  drop {st.dropped}"
            put(canvas, stats, (badge_left - 8 - text_width(stats, 0.42), y + 18), TEXT, 0.42)

        message = self._tile_message(st)
        image = self._display_image(st)
        if image is not None and self.insp.state == STREAMING:
            fitted, scale, ox, oy = fit_into(image, TILE_W, TILE_H)
            if message:
                fitted = (fitted * 0.3).astype(np.uint8)
            canvas[body_y + oy:body_y + oy + fitted.shape[0], x + ox:x + ox + fitted.shape[1]] = fitted
            self.placements[spec.key] = (x + ox, body_y + oy, scale, image.shape[1], image.shape[0])
            if spec.key == "depth":
                cx, cy = x + ox + fitted.shape[1] // 2, body_y + oy + fitted.shape[0] // 2
                r = max(3, int(CENTER_ROI * scale))
                cv2.rectangle(canvas, (cx - r, cy - r), (cx + r, cy + r), (255, 255, 255), 1)
            footer = self._tile_footer(st)
            if footer:
                label_box(canvas, footer, (x + 10, body_y + TILE_H - 10))
        if message:
            w = text_width(message, 0.8, 2)
            put(canvas, message, (x + (TILE_W - w) // 2, body_y + TILE_H // 2), STATUS_BGR[check.status], 0.8, 2)
            if check.detail and self.insp.state == STREAMING:
                detail = clip_text(check.detail, TILE_W - 20, 0.45)
                put(canvas, detail, (x + (TILE_W - text_width(detail)) // 2, body_y + TILE_H // 2 + 26), DIM)
        cv2.rectangle(canvas, (x, y), (x + TILE_W - 1, body_y + TILE_H - 1), BORDER, 1)

    def _tile_footer(self, st):
        a, md = self.insp.analysis, st.metadata
        parts = []
        if st.spec.key == "depth":
            if "depth_fill" in a:
                parts.append(f"fill {a['depth_fill']:.0%}")
            if "center_m" in a:
                parts.append(f"center {a['center_m']:.3f} m")
            if "center_noise_mm" in a:
                parts.append(f"noise {a['center_noise_mm']:.1f} mm")
        elif st.spec.key == "color" and "color" in a:
            parts.append(f"brightness {a['color']['mean']:.0f}")
            parts.append(f"sharpness {a['color']['sharpness']:.0f}")
        elif st.spec.key in a:
            parts.append(f"mean {a[st.spec.key]['mean']:.0f}  std {a[st.spec.key]['std']:.0f}")
        if "exposure" in md:
            parts.append(f"exp {md['exposure']}")
        if "gain" in md:
            parts.append(f"gain {md['gain']}")
        return "  ".join(parts)

    def _draw_hover(self, canvas, states):
        """Show the distance under the mouse on the depth tile (and IR left, which shares its pixels)."""
        if self.mouse is None or states["depth"].image is None:
            return
        depth = states["depth"].image
        mx, my = self.mouse
        for key in ("depth", "ir1"):
            placement = self.placements.get(key)
            if placement is None:
                continue
            x0, y0, scale, iw, ih = placement
            if (iw, ih) != (depth.shape[1], depth.shape[0]):
                continue
            u, v = int((mx - x0) / scale), int((my - y0) / scale)
            if not (0 <= u < iw and 0 <= v < ih):
                continue
            meters = depth[v, u] * self.insp.depth_scale
            cv2.drawMarker(canvas, (mx, my), (255, 255, 255), cv2.MARKER_CROSS, 14, 1)
            text = f"{meters:.3f} m" if meters > 0 else "no depth"
            label_box(canvas, f"({u},{v}) {text}", (mx + 10, my - 8))

    def _draw_panel(self, canvas, checks):
        insp, info = self.insp, self.insp.info
        x0 = GRID_W
        cv2.rectangle(canvas, (x0, 0), (CANVAS_W - 1, CANVAS_H - 1), PANEL_BG, -1)
        x, right = x0 + 16, CANVAS_W - 14
        width = right - x

        overall = worst(c.status for c in checks)
        put(canvas, "D435 INSPECTOR", (x, 30), TEXT, 0.7, 2)
        draw_badge(canvas, overall, right, 14, 0.5)

        y = 56
        lines = [
            f"{info.get('name', 'RealSense')}   S/N {info.get('serial', insp.target_serial or '-')}",
            f"FW {info.get('firmware', '-')}  (rec {info.get('recommended_fw', '-')})   USB {info.get('usb', '-')}"
            + (f"   port {info['usb_port']}" if info.get("usb_port") else ""),
            "Mode  " + "   ".join(f"{k} {w}x{h}@{f}" for k, (w, h, f) in insp.modes.items()),
            f"State {insp.state}" + (f" - {insp.message}" if insp.message else ""),
        ]
        for line in lines:
            put(canvas, clip_text(line, width, 0.45), (x, y), DIM if line.startswith("Mode") else TEXT)
            y += 19

        y = self._section(canvas, "LIVE", x, y + 6, right)
        live, a = insp.live, insp.analysis
        temps = "   ".join(f"{label} {live[k]:.0f} C" for k, label in
                           (("asic_temp", "ASIC"), ("projector_temp", "Projector")) if k in live)
        put(canvas, temps or "temperature not reported", (x, y))
        y += 19
        if "emitter" in live:
            emitter = "ON" if live["emitter"] else "OFF"
            laser = f"   laser {live['laser_power']:.0f} mW" if "laser_power" in live else ""
            put(canvas, f"Emitter {emitter}{laser}", (x, y),
                STATUS_BGR[OK] if live["emitter"] else STATUS_BGR[WARN])
        y += 19
        if "center_m" in a:
            noise = f"   noise {a['center_noise_mm']:.1f} mm" if "center_noise_mm" in a else ""
            put(canvas, f"Center depth {a['center_m']:.3f} m{noise}", (x, y))
        y += 19

        y = self._section(canvas, "CHECKS", x, y + 6, right)
        for check in checks:
            cv2.circle(canvas, (x + 5, y - 5), 5, STATUS_BGR[check.status], -1)
            put(canvas, check.name, (x + 18, y), TEXT, 0.43)
            put(canvas, clip_text(check.detail, right - (x + 160), 0.4), (x + 160, y), DIM, 0.4)
            y += 19

        keys_y = CANVAS_H - 34
        y = self._section(canvas, "EVENTS", x, y + 6, right)
        rows = max(0, (keys_y - 18 - y) // 17 + 1)
        for stamp, level, text in list(insp.events)[-rows:] if rows else []:
            put(canvas, stamp, (x, y), DIM, 0.38)
            put(canvas, clip_text(text, right - (x + 70), 0.38), (x + 70, y), STATUS_BGR[level], 0.38)
            y += 17

        cv2.line(canvas, (x, keys_y - 14), (right, keys_y - 14), BORDER, 1)
        put(canvas, "[Q] quit  [E] emitter  [C] calib check  [S] snapshot  [R] reset", (x, keys_y + 4), DIM, 0.4)
        put(canvas, "Hover over depth / IR-left to read distance", (x, keys_y + 22), DIM, 0.4)

    @staticmethod
    def _section(canvas, title, x, y, right):
        put(canvas, title, (x, y + 4), SECTION, 0.48, 1)
        cv2.line(canvas, (x + text_width(title, 0.48) + 10, y), (right, y), BORDER, 1)
        return y + 26


# --------------------------------------------------------------------------- CLI

ANSI = {OK: "\033[32m", WARN: "\033[33m", FAIL: "\033[31m", PEND: "\033[36m", INFO: "\033[90m"}


def paint(status, text):
    return f"{ANSI[status]}{text}\033[0m" if sys.stdout.isatty() else text


def print_report(report):
    dev = report["device"]
    print()
    print(f"D435 inspection report  {report['time']}")
    print(f"Device : {dev.get('name', '?')}  S/N {dev.get('serial', '?')}  "
          f"FW {dev.get('firmware', '?')} (recommended {dev.get('recommended_fw', '?')})  USB {dev.get('usb', '?')}"
          + (f"  port {dev['usb_port']}" if dev.get("usb_port") else ""))
    print(f"Modes  : {', '.join(f'{k} {v}' for k, v in report['modes'].items()) or '-'}")
    print("-" * 78)
    for s in report["streams"].values():
        print(f"  {s['label']:<9} {s['mode']:<14} {s['fps']:6.2f}/{s['target_fps']:<3} fps  "
              f"received {s['received']:<6} dropped {s['dropped']}")
    print("-" * 78)
    for c in report["checks"]:
        tag = f"[{c['status']:^4}]"
        print(f"  {paint(c['status'], tag)} {c['name']:<18} {c['detail']}")
    print("-" * 78)
    print(f"Overall: {paint(report['overall'], report['overall'])}")


def run_check(inspector, duration, with_calibration, json_path):
    try:
        connected = inspector.connect()
    except DeviceAccessError:
        raise
    except RuntimeError as e:
        connected, inspector.message = False, str(e)
    if not connected:
        print(f"FAIL: {inspector.message}")
        return 2
    print(f"Streaming for {duration:.0f}s ...", flush=True)
    end = time.monotonic() + duration
    while time.monotonic() < end:
        inspector.tick()
        time.sleep(0.05)
    if with_calibration and inspector.start_calibration_check():
        while inspector.calib["state"] == "running" or inspector.state == CALIBRATING:
            time.sleep(0.2)
        end = time.monotonic() + 2.0   # let streams settle again before the final snapshot of stats
        while time.monotonic() < end:
            inspector.tick()
            time.sleep(0.05)
    inspector.tick()
    report = inspector.report()
    print_report(report)
    if json_path:
        Path(json_path).write_text(json.dumps(report, indent=2, default=str))
        give_back_to_sudo_user(json_path)
        print(f"JSON report written to {json_path}")
    return 1 if report["overall"] == FAIL else 0


def list_devices(ctx):
    devices = query_devices(ctx)
    if not devices:
        print("No RealSense device found.")
        return 2
    for serial, dev in devices:
        print(f"S/N {serial}")
        for key, value in read_info(dev).items():
            print(f"    {key:<15} {value}")
        for sensor in dev.query_sensors():
            print(f"    sensor          {sensor.get_info(rs.camera_info.name)}")
    return 0


HELP_DESCRIPTION = """\
Inspect an Intel RealSense D400 camera (built for the D435).

Opens a live dashboard of the color, depth and both infrared streams, with 14 health checks:
USB link, firmware, frame rate, dropped or frozen frames, depth fill, IR balance, exposure,
temperature, hardware errors and calibration. Outdated firmware is updated first."""

HELP_EPILOG = """\
dashboard keys:
  Q / Esc   quit                        S   save a snapshot to snapshots/<time>/
  E         IR emitter on/off           R   hardware-reset the camera
  C         calibration health check (keep the camera still, aim at a textured scene)
  mouse     hover over Depth or IR Left to read the distance

firmware update:
  Firmware older than the version the bundled librealsense recommends ({firmware}) is
  updated before inspecting: the image is downloaded from Intel and checked against
  librealsense's SHA-1, the camera flash is backed up to firmware/, then the camera is
  flashed. It takes 1-2 minutes; don't unplug the camera. Skip it with --no-fw-update.

examples:
  %(prog)s                              open the dashboard
  %(prog)s --check                      text report, no window (also over SSH)
  %(prog)s --check --json report.json   ... and save it as JSON
  %(prog)s --list                       show connected cameras and their S/N
  %(prog)s --serial 123456789012        use this camera when several are connected"""


def parse_args(argv):
    argv = list(sys.argv[1:] if argv is None else argv)
    prog = "d435_inspect.py"
    if argv[:1] == ["--launcher"] and len(argv) > 1:   # run-mac.sh / run-linux.sh name themselves
        prog, argv = argv[1], argv[2:]
    parser = argparse.ArgumentParser(
        prog=prog, description=HELP_DESCRIPTION, formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=HELP_EPILOG.format(firmware=max(KNOWN_IMAGES, key=version_tuple)))

    mode = parser.add_argument_group("modes (default: open the dashboard)")
    mode.add_argument("--check", action="store_true",
                      help="no window: stream for --duration seconds, print a report and exit "
                           "(exit code 1 if a check fails)")
    mode.add_argument("--list", action="store_true", help="list connected cameras and exit")

    camera = parser.add_argument_group("camera")
    camera.add_argument("--serial", metavar="S/N",
                        help="camera to use when several are connected (see --list); default: the first one")
    camera.add_argument("--width", type=int, default=DEFAULT_MODE[0], metavar="PX",
                        help="stream width to try first (default: %(default)s)")
    camera.add_argument("--height", type=int, default=DEFAULT_MODE[1], metavar="PX",
                        help="stream height to try first (default: %(default)s)")
    camera.add_argument("--fps", type=int, default=DEFAULT_MODE[2],
                        help="frame rate to try first (default: %(default)s); when the camera or USB link "
                             "can't do this mode, a supported one is used")

    check = parser.add_argument_group("--check options")
    check.add_argument("--duration", type=float, default=6.0, metavar="SEC",
                       help="how long to stream before reporting (default: %(default)s)")
    check.add_argument("--calib", action="store_true",
                       help="also run the calibration health check (adds 5-15 s)")
    check.add_argument("--json", metavar="FILE", help="also write the report to FILE as JSON")

    other = parser.add_argument_group("other")
    other.add_argument("--no-fw-update", action="store_true", help="don't update outdated firmware this time")
    other.add_argument("--debug", action="store_true", help="print librealsense's internal log (verbose)")
    return parser.parse_args(argv)


def main(ctx, usb_capture=None, argv=None):
    args = parse_args(argv)
    if args.debug:
        rs.log_to_console(rs.log_severity.debug)
    inspector = None
    try:
        if args.list:
            return list_devices(ctx)
        if (not args.check and sys.platform.startswith("linux")
                and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))):
            print("No graphical display (DISPLAY / WAYLAND_DISPLAY not set): run it from the desktop "
                  "session, or use --check for a text report.", file=sys.stderr)
            return 4
        if not args.no_fw_update:
            FirmwareUpdater(ctx, FIRMWARE_DIR, usb_capture, give_back_to_sudo_user).run(args.serial)
        inspector = Inspector(ctx, args.serial, (args.width, args.height, args.fps), usb_capture)
        if args.check:
            return run_check(inspector, args.duration, args.calib, args.json)
        try:
            inspector.connect()   # without a working camera the dashboard keeps retrying
        except DeviceAccessError:
            raise
        except RuntimeError as e:
            inspector.report_connect_error(e)
        Dashboard(inspector).run()
        return 0
    except DeviceAccessError as e:
        print(ACCESS_HINT.format(err=e), file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        return 130
    finally:
        if inspector is not None:
            inspector.close()


if __name__ == "__main__":
    # On macOS take the cameras from UVCAssistant before librealsense touches them (see
    # macos_usb_capture.py); this must happen before the context starts enumerating devices.
    capture = UsbCapture.for_pyrealsense()
    if capture is not None:
        for device, error in capture.capture_new():
            print(f"macOS USB capture {device}: {error or 'ok'}", flush=True)
    # librealsense on macOS segfaults when a context is freed after a failed device open, so the
    # context lives at module level and the process exits without tearing it down.
    context = rs.context()
    code = main(context, capture)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
