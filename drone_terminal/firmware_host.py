"""Run the drone's real navigation firmware (main/nav_task.c) on the laptop.

The firmware's own C files — main/nav_task.c, main/vfh.c, main/odom.c — are
compiled unchanged into a small library together with
drone_terminal/fw_host/fw_host.c, which stands in for FreeRTOS, mavlink_task,
tof_task and wifi_task.  nav_task()'s real loop runs on its own thread in
lockstep with the simulator's clock, so trajectory playback, yaw alignment,
START checks and every other decision come from the firmware source itself.

The library is rebuilt automatically whenever any of those sources or any
header in main/ changes (it is cached in drone_terminal/.fw-host/ under a
content hash), so editing the firmware changes the simulation on the next run.
Needs a C compiler: gcc (MinGW-w64 on Windows), cc on macOS/Linux, or $CC.
"""

from __future__ import annotations

import ctypes
import hashlib
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
_MAIN = _REPO / "main"
_HOST = _HERE / "fw_host"
BUILD_DIR = _HERE / ".fw-host"
_ABI_VERSION = 1            # must match FW_ABI_VERSION in fw_host.c

# nav_state_t in main/nav_task.h
NAV_IDLE, NAV_ROTATING, NAV_FLYING, NAV_ARRIVED, NAV_STUCK, NAV_TRAJ = range(6)
NAV_STATE_NAMES = ("IDLE", "ROTATING", "FLYING", "ARRIVED", "STUCK", "TRAJ")
# setpoint types in fw_host.c (mirroring mavlink_task.c)
SP_HOLD, SP_POSITION, SP_VELOCITY, SP_VEL_XY_POS_Z, SP_POS_VEL = range(5)

# Same chunk sizes as laptop/protocol.py (TRAJ_CHUNK_PTS / TRAJ_YAW_CHUNK_PTS).
CHUNK_PTS, YAW_CHUNK_PTS = 80, 60


class FirmwareHostError(RuntimeError):
    """The firmware could not be built or loaded for the laptop."""


class Setpoint(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int32)] + [
        (name, ctypes.c_float) for name in ("x", "y", "z", "vx", "vy", "vz", "yaw", "yaw_rate")]


def _sources() -> list[Path]:
    return [_HOST / "fw_host.c", _MAIN / "vfh.c", _MAIN / "odom.c"]


def _dependencies() -> list[Path]:
    """Everything whose change must rebuild the library."""
    deps = _sources() + [_MAIN / "nav_task.c"]
    deps += sorted(_MAIN.glob("*.h")) + sorted((_HOST / "include").rglob("*.h"))
    return deps


def _compiler() -> str:
    cc = os.environ.get("CC") or ("gcc" if sys.platform == "win32" else "cc")
    found = shutil.which(cc)
    if found is None:
        hint = {"win32": "install MinGW-w64 gcc (e.g. conda install -c conda-forge gcc)",
                "darwin": "install the Xcode command line tools (xcode-select --install)"}
        raise FirmwareHostError(
            f"no C compiler ({cc!r} not on PATH): "
            f"{hint.get(sys.platform, 'install gcc (sudo apt install build-essential)')}, "
            "or set CC.")
    return found


def _digest() -> str:
    h = hashlib.sha256(f"abi{_ABI_VERSION}-{platform.machine()}".encode())
    for path in _dependencies():
        h.update(path.relative_to(_REPO).as_posix().encode())
        h.update(path.read_bytes())
    return h.hexdigest()[:12]


def build() -> Path:
    """Compile the firmware for this machine if it changed; return the library."""
    missing = [p for p in _dependencies() if not p.is_file()]
    if missing:
        raise FirmwareHostError("missing firmware sources: "
                                + ", ".join(str(p) for p in missing))
    ext = {"darwin": ".dylib", "win32": ".dll"}.get(sys.platform, ".so")
    lib = BUILD_DIR / f"libfwhost-{platform.machine()}-{_digest()}{ext}"
    if lib.is_file():
        return lib
    BUILD_DIR.mkdir(exist_ok=True)
    cmd = [_compiler(), "-O2", "-shared", "-std=gnu11", "-w",
           "-I", str(_HOST / "include"), "-I", str(_MAIN),
           *map(str, _sources()), "-o", str(lib)]
    cmd += (["-static-libgcc", "-lm"] if sys.platform == "win32"
            else ["-fPIC", "-lm", "-lpthread"])
    print(f"[firmware] compiling main/nav_task.c for the laptop -> {lib.name}")
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise FirmwareHostError("firmware build failed:\n" + result.stderr[-4000:])
    # Old builds from earlier firmware versions (ignore ones still loaded).
    for old in BUILD_DIR.glob(f"libfwhost-{platform.machine()}-*{ext}"):
        if old != lib:
            try:
                old.unlink()
            except OSError:
                pass
    return lib


class Firmware:
    """The compiled navigation firmware. One per process (it owns a thread)."""

    _instance: "Firmware | None" = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._load()
        return cls._instance

    def _load(self) -> None:
        self.path = build()
        lib = ctypes.CDLL(str(self.path))
        if lib.fw_abi() != _ABI_VERSION:
            raise FirmwareHostError(f"{self.path.name}: ABI mismatch")
        lib.fw_reset.restype = None
        lib.fw_set_state.argtypes = [ctypes.c_float] * 7 + [ctypes.c_int] * 3
        lib.fw_set_state.restype = None
        lib.fw_advance_us.argtypes = [ctypes.c_int64]
        lib.fw_advance_us.restype = ctypes.c_int
        lib.fw_get_setpoint.argtypes = [ctypes.POINTER(Setpoint)]
        lib.fw_get_setpoint.restype = None
        lib.fw_traj_progress.restype = ctypes.c_double
        lib.esp_timer_get_time.restype = ctypes.c_int64
        lib.fw_take_log.argtypes = [ctypes.c_char_p, ctypes.c_int]
        lib.nav_traj_put.argtypes = [ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint16,
                                     ctypes.c_void_p, ctypes.c_int]
        lib.nav_traj_put_yaw.argtypes = lib.nav_traj_put.argtypes
        lib.nav_traj_start.argtypes = [ctypes.c_uint8, ctypes.c_uint16]
        lib.nav_traj_start_yaw.argtypes = [ctypes.c_uint8, ctypes.c_uint16]
        for name in ("nav_traj_put", "nav_traj_put_yaw", "nav_traj_start",
                     "nav_traj_start_yaw", "nav_cancel"):
            getattr(lib, name).restype = None
        self.lib = lib
        self._sp = Setpoint()
        self._log_buf = ctypes.create_string_buffer(65536)

    # --- lifecycle / time ------------------------------------------------------
    def reset(self) -> None:
        """Fresh firmware state for a new flight."""
        self.lib.fw_reset()

    def advance(self, dt_s: float) -> int:
        """Move simulated time on; returns how many nav_task() loops ran."""
        return self.lib.fw_advance_us(round(dt_s * 1e6))

    def set_state(self, x, y, z, vx, vy, vz, heading, armed=True,
                  pos_valid=True, wifi_ok=True) -> None:
        """Telemetry the firmware sees (PX4's estimate, as over MAVLink)."""
        self.lib.fw_set_state(x, y, z, vx, vy, vz, heading,
                              int(armed), int(pos_valid), int(wifi_ok))

    # --- what the laptop sends ---------------------------------------------------
    def upload(self, traj_id: int, points) -> None:
        """Send a route in the same chunks as laptop/protocol.py.

        points: (N, 3) x,y,z offsets, or (N, 4) with absolute yaw in radians.
        """
        rows = [tuple(float(v) for v in p) for p in points]
        has_yaw = len(rows[0]) == 4
        chunk = YAW_CHUNK_PTS if has_yaw else CHUNK_PTS
        put = self.lib.nav_traj_put_yaw if has_yaw else self.lib.nav_traj_put
        for off in range(0, len(rows), chunk):
            part = rows[off:off + chunk]
            flat = (ctypes.c_float * (len(part) * len(part[0])))(
                *[v for row in part for v in row])
            put(traj_id, len(rows), off, flat, len(part))

    def start(self, traj_id: int, dt_ms: int, has_yaw: bool) -> None:
        (self.lib.nav_traj_start_yaw if has_yaw else self.lib.nav_traj_start)(traj_id, dt_ms)

    def cancel(self) -> None:
        self.lib.nav_cancel()

    # --- what the firmware does ----------------------------------------------------
    def setpoint(self) -> Setpoint:
        """The setpoint mavlink_task would be streaming to PX4 right now."""
        self.lib.fw_get_setpoint(ctypes.byref(self._sp))
        return self._sp

    def nav_state(self) -> int:
        return self.lib.fw_nav_state()

    def traj_progress(self) -> float:
        """Firmware playback position as a fractional point index (-1 if none)."""
        return self.lib.fw_traj_progress()

    def now(self) -> float:
        """Simulated firmware time in seconds (keeps counting across flights)."""
        return self.lib.esp_timer_get_time() / 1e6

    def take_log(self, since: float = 0.0) -> list[str]:
        """Firmware log lines, as 'I 12.345 [nav] ...' with time from `since`."""
        n = self.lib.fw_take_log(self._log_buf, len(self._log_buf))
        text = self._log_buf.raw[:n].decode("utf-8", "replace")
        lines = []
        for line in text.splitlines():
            level, _, rest = line.partition(" ")
            stamp, _, msg = rest.partition(" ")
            try:
                stamp = f"{float(stamp) - since:.2f}"
            except ValueError:
                pass
            lines.append(f"{level} {stamp} {msg}")
        return lines


def finite(v: float) -> bool:
    return math.isfinite(v)
