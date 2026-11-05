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

import argparse
import csv
import math
import threading
import logging
import random
import time
from pathlib import Path

import numpy as np

from comms import CommsNode
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

    def packet(self):
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

    def wait_connected(self, timeout=30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                received = self._latest is not None
            if received:
                return self.packet()
            time.sleep(0.05)
        raise TimeoutError("No drone telemetry before connection timeout")

    def wait(self, predicate, timeout, stable_for=0.0, require_flying=False):
        deadline = time.monotonic() + timeout
        since = None
        while time.monotonic() < deadline:
            packet = self.packet()
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


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Upload a CSV trajectory to one drone and fly it")
    ap.add_argument("--drone-id", type=int, required=True)
    ap.add_argument("--trajectory", required=True, help="CSV with columns t,x,y,z (s, NED m)")
    ap.add_argument("--config", default=str(Path(__file__).with_name("setup.yaml")),
                    help="Fleet config (optional)")
    ap.add_argument("--dt", type=float, default=0.05,
                    help="Upload sample period (s); 0.05 = ESP nav loop rate")
    ap.add_argument("--max-speed", type=float, default=0.30,
                    help="Refuse faster trajectories (m/s); keep below MAV_CMD_SPEED_CAP_MS")
    ap.add_argument("--takeoff", action="store_true", help="Send CMD_START (arm + take off) first")
    ap.add_argument("--takeoff-wait", type=float, default=12.0,
                    help="Wait after START (default 12 s); TF01 also requires measured hover, "
                         "legacy firmware uses this timed wait with fresh telemetry")
    ap.add_argument("--finish", choices=["land", "hold"], default="land",
                    help="Action after the trajectory")
    ap.add_argument("--confirm", action="store_true",
                    help="Require typing TRAJ-<drone id> before sending any flight commands")
    ap.add_argument("--telem-port", type=int, default=5005)
    ap.add_argument("--cmd-port", type=int, default=5006)
    ap.add_argument("--takeoff-timeout", type=float, default=40.0,
                    help="Maximum takeoff wait (default 40 s)")
    ap.add_argument("--dry-run", action="store_true", help="Validate and describe the CSV without connecting")
    args = ap.parse_args(argv)
    if not all(math.isfinite(v) and v > 0 for v in (args.dt, args.max_speed, args.takeoff_timeout)):
        ap.error("dt, max speed and takeoff timeout must be finite and positive")
    if not math.isfinite(args.takeoff_wait) or args.takeoff_wait < 0 or args.takeoff_wait >= args.takeoff_timeout:
        ap.error("takeoff wait must be nonnegative and less than takeoff timeout")
    if not 0 <= args.drone_id <= 255 or not all(1 <= p <= 65535 for p in (args.telem_port, args.cmd_port)):
        ap.error("Invalid drone ID or UDP port")

    dt_ms = round(args.dt * 1000)
    if not 1 <= dt_ms <= 1000:
        raise SystemExit("--dt must be 0.001-1.0 s")
    try:
        pts = load_trajectory(args.trajectory, dt_ms / 1000)
    except ValueError as exc:
        ap.error(str(exc))
    duration = (len(pts) - 1) * dt_ms / 1000
    vmax = np.linalg.norm(np.diff(pts, axis=0), axis=1).max() * 1000 / dt_ms
    log.info("%s: %d points, %.1f s, max speed %.3f m/s",
             args.trajectory, len(pts), duration, vmax)
    if vmax > args.max_speed + 1e-6:
        raise SystemExit(f"max speed {vmax:.3f} m/s exceeds --max-speed {args.max_speed} m/s")
    if len(pts) > TRAJ_MAX_PTS:
        raise SystemExit(f"{len(pts)} points > {TRAJ_MAX_PTS}; use a larger --dt or a shorter trajectory")

    log.info("CSV Z offsets: %.3f to %.3f m (negative climbs); final offset %.3f m",
             pts[:, 2].min(), pts[:, 2].max(), pts[-1, 2])
    if args.takeoff:
        log.info("Assuming 0.50 m takeoff; nominal final height %.3f m (TF01 verifies takeoff height)",
                 0.5 - pts[-1, 2])
        if np.max(pts[:, 2]) >= 0.5 - 0.10:
            ap.error("Trajectory descends to within 0.10 m of the ground reference")
    if args.dry_run:
        return

    feedback = FlightFeedback(args.drone_id)

    def send(cmd_type: int, name: str) -> None:
        if not comms.send_command(args.drone_id, CommandPacket(cmd_type)):
            raise RuntimeError(f"{name} not sent: drone {args.drone_id} IP unknown")
        log.info("%s sent", name)

    comms = CommsNode(
        listen_port=args.telem_port,
        cmd_port=args.cmd_port,
        config_path=args.config if Path(args.config).exists() else None,
    )
    comms.on_telemetry(feedback.update)
    flying = False
    try:
        comms.start()
        initial = feedback.wait_connected()
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

        if args.confirm:
            answer = input(f"Type TRAJ-{args.drone_id} to fly {args.trajectory}: ").strip()
            if answer != f"TRAJ-{args.drone_id}":
                log.info("confirmation did not match; trajectory cancelled")
                return
        # Recheck after a potentially long confirmation prompt.
        initial = feedback.packet()
        if not extended and initial.nav_state == NAV_TRAJ:
            raise RuntimeError("Drone is already playing a trajectory")
        if extended and args.takeoff and (initial.px4_armed or initial.mission_phase != MISSION_READY):
            raise RuntimeError("Drone is no longer ready for takeoff")
        if extended and not args.takeoff and not at_hover(initial):
            raise RuntimeError("Without --takeoff, a stable armed OFFBOARD hover is required")

        traj_id = random.randint(1, 255)
        packets = build_traj_packets(traj_id, pts)

        def upload():
            for chunk in packets:
                feedback.packet()
                if not comms.send_raw(args.drone_id, chunk):
                    raise ConnectionError("Trajectory upload failed")
                time.sleep(UPLOAD_GAP_S)

        upload()  # upload on the ground before START whenever possible
        if args.takeoff:
            initial = feedback.packet()
            if extended and (initial.px4_armed or initial.mission_phase != MISSION_READY):
                raise RuntimeError("Drone is no longer ready for takeoff")
            flying = True  # an uncertain START transmission must still trigger cleanup
            send(CMD_START, "CMD_START")
            started = time.monotonic()
            if extended:
                feedback.wait(lambda p: time.monotonic() - started >= args.takeoff_wait
                              and at_hover(p, 0.5), args.takeoff_timeout, stable_for=1.0)
            else:
                log.info("Waiting %.1f s after START; confirm takeoff/hover in QGC or visually",
                         args.takeoff_wait)
                feedback.wait(lambda p: time.monotonic() - started >= args.takeoff_wait,
                              args.takeoff_timeout)
        else:
            flying = True
            if extended:
                feedback.wait(at_hover, 5.0, stable_for=1.0, require_flying=True)
            else:
                log.info("Using the existing hover; legacy telemetry cannot verify it")

        start = build_traj_start(traj_id, dt_ms)
        origin = None
        for attempt in range(3):
            origin = feedback.packet()
            if extended and not at_hover(origin, 0.5 if args.takeoff else None):
                raise RuntimeError("Hover lost before trajectory start")
            if not comms.send_raw(args.drone_id, start):
                raise ConnectionError("Trajectory START could not be sent")
            try:
                feedback.wait(lambda p: p.nav_state == NAV_TRAJ, 1.5, require_flying=extended)
                break
            except TimeoutError:
                # Only re-upload if fresh feedback proves playback never started.
                if feedback.packet().nav_state == NAV_ARRIVED or attempt == 2:
                    raise RuntimeError("Trajectory start was not confirmed")
                upload()
        log.info("Playing trajectory %d (%.1f s)", traj_id, duration)
        def trajectory_done(packet):
            if packet.nav_state not in (NAV_TRAJ, NAV_ARRIVED):
                raise RuntimeError("Trajectory playback stopped before arrival")
            return packet.nav_state == NAV_ARRIVED

        feedback.wait(trajectory_done, duration + 5.0, require_flying=extended)
        if extended:
            target = np.array([origin.ned_x, origin.ned_y, origin.local_z]) + pts[-1]
            feedback.wait(lambda p: np.linalg.norm(np.array([p.ned_x, p.ned_y, p.local_z]) - target) < 0.15
                          and abs(p.local_vz) < 0.10, 10.0, stable_for=1.0, require_flying=True)
            log.info("Trajectory endpoint reached; measured local z=%.3f", feedback.packet().local_z)
        else:
            target_xy = np.array([origin.ned_x, origin.ned_y]) + pts[-1, :2]
            feedback.wait(lambda p: p.nav_state == NAV_ARRIVED
                          and np.linalg.norm(np.array([p.ned_x, p.ned_y]) - target_xy) < 0.15,
                          10.0, stable_for=1.0)
            log.info("Trajectory endpoint reached in XY; altitude is unavailable on legacy firmware")
        if args.finish == "land":
            for _ in range(3):
                send(CMD_LAND, "CMD_LAND")
                time.sleep(0.1)
        else:
            send(CMD_HOLD, "CMD_HOLD")
        flying = False
    except (Exception, KeyboardInterrupt):
        if flying:
            log.error("Aborted — requesting LAND (firmware respects RC mode takeover)")
            for _ in range(3):
                try:
                    comms.send_command(args.drone_id, CommandPacket(CMD_LAND))
                except OSError:
                    log.exception("LAND could not be sent")
                time.sleep(0.1)
        raise
    finally:
        comms.stop()


if __name__ == "__main__":
    main()
