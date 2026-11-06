#!/usr/bin/env python3
"""
Fly a CSV trajectory smoothly on one drone.

The whole trajectory is uploaded first; the ESP32 then plays it on its own
20 Hz clock (position + velocity feedforward to PX4), so WiFi latency and
jitter do not set the playback timing. Fresh flight feedback is still required.

CSV: columns t,x,y,z (s, NED m); a header row and '#' comments are allowed.
The trajectory is flown relative to where the drone hovers when it starts.
Z is down-positive; constant CSV Z means no climb. TF01 telemetry enables
verified takeoff and height monitoring. Older firmware is detected automatically:
it uses --takeoff-wait (12 s by default) and verifies the horizontal endpoint,
without height, arm/mode, or PX4 sample-age feedback. Use --dry-run offline.

Example:
    python3 laptop/send_trajectory.py --drone-id 2 --takeoff --confirm \
        --trajectory trajectory/circle_traj.csv
"""

import csv
import math
import threading
import logging
import random
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from protocol import (
    CMD_HOLD,
    CMD_LAND,
    CMD_START,
    NAV_TRAJ, NAV_ARRIVED, MISSION_READY, MISSION_FLYING,
    TRAJ_MAX_PTS,
    CommandPacket,
    build_traj_packets,
    build_traj_start,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("send_trajectory")

UPLOAD_GAP_S = 0.025   # ESP drains commands at 10 Hz from a 6-packet UDP queue


class FlightCancelled(RuntimeError):
    """A HOLD/LAND request superseded the active operation."""


def check_cancelled(event):
    if event is not None and event.is_set():
        raise FlightCancelled("Flight operation cancelled by operator")


class FlightControl:
    """Serialize flight sends against HOLD/LAND so no START follows a stop."""

    def __init__(self):
        self.cancelled = threading.Event()
        self._lock = threading.Lock()

    def send_command(self, comms, drone_id, command):
        with self._lock:
            check_cancelled(self.cancelled)
            return comms.send_command(drone_id, command)

    def send_raw(self, comms, drone_id, data):
        with self._lock:
            check_cancelled(self.cancelled)
            return comms.send_raw(drone_id, data)

    def interrupt(self, comms, drone_id, command):
        with self._lock:
            self.cancelled.set()
            return comms.send_command(drone_id, CommandPacket(command))


@dataclass(frozen=True)
class FlightOptions:
    drone_id: int
    dt_ms: int = 50
    takeoff: bool = False
    takeoff_wait: float = 12.0
    takeoff_timeout: float = 40.0
    finish: str = "land"


def load_trajectory(path: str, dt: float) -> np.ndarray:
    """Strict t,x,y,z CSV; preserve the endpoint, return first-point offsets."""
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("Sample interval must be finite and positive")
    rows = []
    with open(path, newline="", encoding="utf-8-sig") as source:
        for line_number, line in enumerate(source, 1):
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            cells = next(csv.reader([line]))
            if not rows and [c.strip().lower() for c in cells] == ["t", "x", "y", "z"]:
                continue
            try:
                values = [float(c) for c in cells]
                if len(values) != 4 or not all(math.isfinite(v) for v in values):
                    raise ValueError()
            except ValueError:
                raise ValueError(f"{path}:{line_number}: expected four finite t,x,y,z values") from None
            rows.append(values)
    if len(rows) < 2:
        raise ValueError(f"{path}: need at least two t,x,y,z rows")
    data = np.asarray(rows)
    t = data[:, 0] - data[0, 0]
    if np.any(np.diff(t) <= 0):
        raise ValueError(f"{path}: time column must be strictly increasing")
    count = math.ceil(round(float(t[-1] / dt), 9)) + 1
    if count > TRAJ_MAX_PTS:
        raise ValueError(f"{count} points > {TRAJ_MAX_PTS}; use a larger --dt or shorter route")
    # Last partial interval is padded with a hold, never truncated.
    tq = np.arange(count) * dt
    pts = np.column_stack([np.interp(tq, t, data[:, c]) for c in (1, 2, 3)])
    return pts - pts[0]


class FlightFeedback:
    """Track packet arrival separately from the age of PX4 data inside it."""

    def __init__(self, drone_id, stale_timeout=1.0):
        self.drone_id = drone_id
        self.stale_timeout = stale_timeout
        self._latest = None
        self._lock = threading.Lock()
        self._extended_status = None

    def update(self, packet, _ip):
        if packet.drone_id == self.drone_id:
            with self._lock:
                self._latest = (packet, time.monotonic())

    def packet(self, cancel_event=None):
        check_cancelled(cancel_event)
        with self._lock:
            latest = self._latest
        if latest is None:
            raise ConnectionError("No drone telemetry")
        packet, received = latest
        if time.monotonic() - received > self.stale_timeout:
            raise ConnectionError("Drone telemetry is stale")
        if packet.flight_status_version not in (None, 1):
            raise RuntimeError("Unsupported flight-status telemetry version")
        extended = packet.flight_status_version == 1
        if self._extended_status is not None and extended != self._extended_status:
            raise RuntimeError("Flight-status telemetry format changed during this session")
        if extended and not packet.px4_link_ok:
            raise ConnectionError("PX4 position or heartbeat is stale")
        values = (packet.ned_x, packet.ned_y, packet.heading_rad)
        if extended:
            values += (packet.local_z, packet.local_vz)
        if not all(v is not None and math.isfinite(v) for v in values):
            raise RuntimeError("PX4 position contains invalid numbers")
        self._extended_status = extended
        return packet

    def wait_connected(self, timeout=30.0, cancel_event=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            check_cancelled(cancel_event)
            with self._lock:
                received = self._latest is not None
            if received:
                return self.packet(cancel_event)
            time.sleep(0.05)
        raise TimeoutError("No drone telemetry before connection timeout")

    def wait(self, predicate, timeout, stable_for=0.0, require_flying=False, cancel_event=None):
        deadline = time.monotonic() + timeout
        since = None
        while time.monotonic() < deadline:
            packet = self.packet(cancel_event)
            if require_flying and (not packet.px4_armed or packet.px4_custom_main_mode != 6
                                   or packet.mission_phase != MISSION_FLYING):
                raise RuntimeError("Drone left armed OFFBOARD flight; stopping trajectory control")
            if predicate(packet):
                if since is None:
                    since = time.monotonic()
                if time.monotonic() - since >= stable_for:
                    return packet
            else:
                since = None
            time.sleep(0.05)
        raise TimeoutError("Flight feedback did not reach the required state before timeout")


def at_hover(packet, expected_height=None):
    if not packet.px4_armed or packet.px4_custom_main_mode != 6 or packet.mission_phase != MISSION_FLYING:
        return False
    if abs(packet.local_vz) >= 0.10:
        return False
    if expected_height is not None:
        if packet.ground_z is None or not math.isfinite(packet.ground_z):
            return False
        return abs((packet.ground_z - packet.local_z) - expected_height) < 0.08
    return True


def run_flight(comms, feedback, pts, options, *, confirm=None, control=None, on_progress=None):
    """Run a validated route (or takeoff alone) on an existing connection.

    The caller owns the connection and any operator confirmation dialog.
    All waits remain cancellable; HOLD/LAND uses the same FlightControl lock.
    """
    args = options
    control = control or FlightControl()
    dt_ms = args.dt_ms
    duration = (len(pts) - 1) * dt_ms / 1000 if pts is not None else 0

    def stage(message):
        if on_progress is not None:
            on_progress(message)

    def check_packet():
        return feedback.packet(control.cancelled)

    def wait(predicate, timeout, **kwargs):
        return feedback.wait(predicate, timeout, cancel_event=control.cancelled, **kwargs)

    def send(cmd_type, name):
        if not control.send_command(comms, args.drone_id, CommandPacket(cmd_type)):
            raise RuntimeError(f"{name} not sent: drone {args.drone_id} IP unknown")
        log.info("%s sent", name)

    flying = False
    try:
        stage("Waiting for telemetry")
        initial = feedback.wait_connected(cancel_event=control.cancelled)
        extended = initial.flight_status_version == 1
        if not extended:
            if args.takeoff and args.takeoff_wait <= 0:
                raise RuntimeError("Legacy firmware requires a positive --takeoff-wait (default 12 s)")
            log.warning("Legacy telemetry detected: altitude, armed/mode and PX4 sample ages "
                        "are unavailable. Using timed takeoff and XY endpoint checks; no reflash required.")
            if initial.nav_state == NAV_TRAJ:
                raise RuntimeError("Drone is already playing a trajectory")
        if extended and args.takeoff and (initial.px4_armed or initial.mission_phase != MISSION_READY):
            raise RuntimeError("Takeoff requires the drone to be disarmed and firmware READY")

        if confirm is not None and not confirm():
            log.info("confirmation did not match; trajectory cancelled")
            return
        # Recheck after a potentially long confirmation prompt.
        initial = check_packet()
        if not extended and initial.nav_state == NAV_TRAJ:
            raise RuntimeError("Drone is already playing a trajectory")
        if extended and args.takeoff and (initial.px4_armed or initial.mission_phase != MISSION_READY):
            raise RuntimeError("Drone is no longer ready for takeoff")
        if extended and not args.takeoff and not at_hover(initial):
            raise RuntimeError("Without --takeoff, a stable armed OFFBOARD hover is required")

        traj_id = random.randint(1, 255)
        packets = [] if pts is None else build_traj_packets(traj_id, pts)

        def upload():
            stage("Uploading trajectory")
            for chunk in packets:
                check_packet()
                if not control.send_raw(comms, args.drone_id, chunk):
                    raise ConnectionError("Trajectory upload failed")
                time.sleep(UPLOAD_GAP_S)

        upload()  # upload on the ground before START whenever possible
        if args.takeoff:
            initial = check_packet()
            if extended and (initial.px4_armed or initial.mission_phase != MISSION_READY):
                raise RuntimeError("Drone is no longer ready for takeoff")
            flying = True  # an uncertain START transmission must still trigger cleanup
            stage("Taking off")
            send(CMD_START, "CMD_START")
            started = time.monotonic()
            if extended:
                wait(lambda p: time.monotonic() - started >= args.takeoff_wait
                              and at_hover(p, 0.5), args.takeoff_timeout, stable_for=1.0)
            else:
                log.info("Waiting %.1f s after START; confirm takeoff/hover in QGC or visually",
                         args.takeoff_wait)
                wait(lambda p: time.monotonic() - started >= args.takeoff_wait,
                              args.takeoff_timeout)
        else:
            flying = True
            if extended:
                wait(at_hover, 5.0, stable_for=1.0, require_flying=True)
            else:
                log.info("Using the existing hover; legacy telemetry cannot verify it")

        if pts is None:
            flying = False
            stage("Takeoff wait complete; holding")
            return
        stage("Starting trajectory")
        start = build_traj_start(traj_id, dt_ms)
        origin = None
        for attempt in range(3):
            origin = check_packet()
            if extended and not at_hover(origin, 0.5 if args.takeoff else None):
                raise RuntimeError("Hover lost before trajectory start")
            if not control.send_raw(comms, args.drone_id, start):
                raise ConnectionError("Trajectory START could not be sent")
            try:
                wait(lambda p: p.nav_state == NAV_TRAJ, 1.5, require_flying=extended)
                break
            except TimeoutError:
                # Only re-upload if fresh feedback proves playback never started.
                if check_packet().nav_state == NAV_ARRIVED or attempt == 2:
                    raise RuntimeError("Trajectory start was not confirmed")
                upload()
        stage("Flying trajectory")
        log.info("Playing trajectory %d (%.1f s)", traj_id, duration)
        def trajectory_done(packet):
            if packet.nav_state not in (NAV_TRAJ, NAV_ARRIVED):
                raise RuntimeError("Trajectory playback stopped before arrival")
            return packet.nav_state == NAV_ARRIVED

        wait(trajectory_done, duration + 5.0, require_flying=extended)
        stage("Checking endpoint")
        if extended:
            target = np.array([origin.ned_x, origin.ned_y, origin.local_z]) + pts[-1]
            wait(lambda p: np.linalg.norm(np.array([p.ned_x, p.ned_y, p.local_z]) - target) < 0.15
                          and abs(p.local_vz) < 0.10, 10.0, stable_for=1.0, require_flying=True)
            log.info("Trajectory endpoint reached; measured local z=%.3f", check_packet().local_z)
        else:
            target_xy = np.array([origin.ned_x, origin.ned_y]) + pts[-1, :2]
            wait(lambda p: p.nav_state == NAV_ARRIVED
                          and np.linalg.norm(np.array([p.ned_x, p.ned_y]) - target_xy) < 0.15,
                          10.0, stable_for=1.0)
            log.info("Trajectory endpoint reached in XY; altitude is unavailable on legacy firmware")
        stage("Requesting " + args.finish.upper())
        if args.finish == "land":
            for _ in range(3):
                send(CMD_LAND, "CMD_LAND")
                time.sleep(0.1)
        else:
            send(CMD_HOLD, "CMD_HOLD")
        flying = False
        stage("Complete")
    except FlightCancelled:
        raise
    except (Exception, KeyboardInterrupt):
        if flying:
            log.error("Aborted — requesting LAND (firmware respects RC mode takeover)")
            for _ in range(3):
                try:
                    control.send_command(comms, args.drone_id, CommandPacket(CMD_LAND))
                except FlightCancelled:
                    break
                except OSError:
                    log.exception("LAND could not be sent")
                time.sleep(0.1)
        raise


