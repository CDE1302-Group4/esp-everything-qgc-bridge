#!/usr/bin/env python3
"""
GCS mission: take off -> visit CSV waypoints and log tags -> return -> land.

Custom UDP commands (see protocol.py):
  - CMD_START : arm + take off to the firmware's cruise altitude
  - CMD_GOTO  : fly to a map-frame (x, y); resent until the drone acts on it
  - CMD_LAND  : land in place

Waypoints are relative to the takeoff point unless --map-frame is given,
in which case they are arena coordinates and setup.yaml's start offset is sent.
With the updated nav_task firmware, waypoints preserve heading and translate
using pitch/roll. Rebuild/reflash the ESP32 to apply that firmware behavior.

Example:
    python3 laptop/simple_waypoint_mission.py --drone-id 22 --confirm \
        --waypoint 0.5,0.0 --waypoint=-0.5,0.5

    python3 laptop/simple_waypoint_mission.py --drone-id 22 --confirm \
        --waypoints-csv laptop/waypoints_example.csv

CSV columns: x,y (metres), with an optional x,y header and # comment lines.
Points are flown in file order using CMD_GOTO, not timed trajectory playback.
Time/altitude columns from send_trajectory.py are not supported by CMD_GOTO.
Camera detection uses mission_tags.py, the same detector as tag_stream.py.
Confirmed tags are saved to --log. CSV saves only
tag_id and local X/Y/Z coordinates; JSONL retains timestamps, image coordinates,
relative poses/ranges, quality details, and mission events. Rejected and
unconfirmed observations are separate diagnostic events in JSONL only.
Repeated sightings are retained. Recording is automatic; no extra flag is needed.

Exit code: 0 all waypoints reached, 1 error or waypoint timeout, 130 Ctrl+C.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import logging
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from comms import CommsNode
from tag_acceptance import add_tag_arguments, validate_tag_arguments
from protocol import (
    CMD_GOTO,
    CMD_HOLD,
    CMD_LAND,
    CMD_START,
    NAV_FLYING,
    NAV_ROTATING,
    NAV_STUCK,
    CommandPacket,
    TelemetryPacket,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(levelname)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("simple_waypoint_mission")

GOTO_RESEND_S = 2.0   # resend CMD_GOTO if the drone hasn't started on it


@dataclass
class TelemetrySnapshot:
    packet: Optional[TelemetryPacket] = None
    source_ip: Optional[str] = None
    received_at: float = 0.0


class TelemetryTracker:
    def __init__(self, drone_id: int, print_live: bool = False) -> None:
        self.drone_id = drone_id
        self._snapshot = TelemetrySnapshot()
        self._condition = threading.Condition()
        self._print_live = print_live

    def callback(self, packet: TelemetryPacket, source_ip: str) -> None:
        if packet.drone_id != self.drone_id:
            return

        with self._condition:
            self._snapshot = TelemetrySnapshot(
                packet=packet,
                source_ip=source_ip,
                received_at=time.monotonic(),
            )
            self._condition.notify_all()

            if self._print_live:
                tag_text = "none" if packet.tag_id < 0 else str(packet.tag_id)
                log.info(
                    "TELEM drone=%d pos=(%.2f, %.2f) heading=%.2f state=%s tag=%s dist=%.2f stuck=%s",
                    packet.drone_id,
                    packet.ned_x,
                    packet.ned_y,
                    packet.heading_rad,
                    packet.nav_state_name,
                    tag_text,
                    packet.tag_dist_m,
                    "yes" if packet.is_stuck else "no",
                )

    def latest(self) -> TelemetrySnapshot:
        with self._condition:
            return TelemetrySnapshot(
                packet=self._snapshot.packet,
                source_ip=self._snapshot.source_ip,
                received_at=self._snapshot.received_at,
            )

    def wait_for_first_packet(self, timeout_s: float, port: int, heard) -> TelemetrySnapshot:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._snapshot.packet is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(
                        f"No telemetry from drone {self.drone_id} on UDP {port} "
                        f"(heard drones: {sorted(heard()) or 'none'})."
                    )
                self._condition.wait(timeout=min(0.5, remaining))
            return self.latest()


def parse_waypoint(value: str) -> tuple[float, float]:
    parts = [item.strip() for item in value.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(
            "Waypoints must be given as x,y (for example 1.0,2.0)."
        )

    try:
        x_value = float(parts[0])
        y_value = float(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "Waypoint coordinates must be numeric."
        ) from exc

    if not all(math.isfinite(v) for v in (x_value, y_value)):
        raise argparse.ArgumentTypeError("Waypoint coordinates must be finite.")

    return x_value, y_value


def load_waypoints_from_csv(path: str) -> list[tuple[float, float]]:
    """Read strict x,y CSV without silently dropping malformed flight goals."""
    file_path = Path(path).expanduser()
    waypoints = []
    first_row = True
    with file_path.open(encoding="utf-8-sig", newline="") as source:
        for line_number, raw_line in enumerate(source, 1):
            if not raw_line.strip() or raw_line.lstrip().startswith("#"):
                continue
            try:
                row = [field.strip() for field in next(csv.reader([raw_line], strict=True))]
                if first_row and [field.lower() for field in row] == ["x", "y"]:
                    first_row = False
                    continue
                first_row = False
                if len(row) != 2:
                    raise ValueError("expected exactly x,y; t/z columns require send_trajectory.py")
                point = (float(row[0]), float(row[1]))
                if not all(math.isfinite(v) for v in point):
                    raise ValueError("coordinates must be finite")
            except (ValueError, csv.Error) as exc:
                raise ValueError(f"{file_path}:{line_number}: invalid waypoint: {exc}") from exc
            waypoints.append(point)
    if not waypoints:
        raise ValueError(f"{file_path}: no waypoint rows found")
    return waypoints


def load_waypoints_from_file(path: str) -> list[tuple[float, float]]:
    file_path = Path(path).expanduser()
    if file_path.suffix.lower() == ".csv":
        return load_waypoints_from_csv(path)
    if not file_path.exists():
        raise FileNotFoundError(f"Waypoint file not found: {file_path}")

    waypoints: list[tuple[float, float]] = []
    for line_number, raw_line in enumerate(
        file_path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        if "," in line:
            waypoint = parse_waypoint(line)
        else:
            parts = line.split()
            if len(parts) != 2:
                raise ValueError(
                    f"Invalid waypoint format on line {line_number}: {raw_line}"
                )
            try:
                waypoint = (float(parts[0]), float(parts[1]))
            except ValueError as exc:
                raise ValueError(
                    f"Invalid waypoint coordinates on line {line_number}: {raw_line}"
                ) from exc

        if not all(math.isfinite(v) for v in waypoint):
            raise ValueError(f"Non-finite waypoint coordinates on line {line_number}")
        waypoints.append(waypoint)

    return waypoints


def join_waypoint_values(argv: list[str]) -> list[str]:
    """Turn '--waypoint -1,0' into '--waypoint=-1,0' so argparse accepts it."""
    out, it = [], iter(argv)
    for arg in it:
        out.append(f"--waypoint={next(it, '')}" if arg == "--waypoint" else arg)
    return out


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Take off, visit CSV waypoints with AprilTag logging, return to start, and land."
    )
    parser.add_argument(
        "--drone-id",
        type=int,
        required=True,
        help="Flashed ESP32 CONFIG_DRONE_ID.",
    )
    parser.add_argument(
        "--map-frame",
        action="store_true",
        help="Treat waypoints as arena coordinates: send the drone's setup.yaml start "
             "offset. Default: waypoints are relative to the takeoff point.",
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("setup.yaml")),
        help="Path to setup.yaml, used only with --map-frame (default: laptop/setup.yaml).",
    )
    parser.add_argument(
        "--telem-port",
        type=int,
        default=5005,
        help="Laptop telemetry UDP port (default: 5005).",
    )
    parser.add_argument(
        "--cmd-port",
        type=int,
        default=5006,
        help="ESP32 command UDP port (default: 5006).",
    )
    parser.add_argument(
        "--waypoint",
        dest="waypoints",
        action="append",
        type=parse_waypoint,
        default=[],
        help="Waypoint as x,y. Repeat this option for multiple waypoints.",
    )
    files = parser.add_mutually_exclusive_group()
    files.add_argument("--waypoints-csv", help="CSV file with x,y columns, in flight order.")
    files.add_argument(
        "--waypoints-file",
        help="Waypoint file: .csv uses the CSV reader; other files accept x,y or x y lines.",
    )
    parser.add_argument(
        "--takeoff-wait",
        type=float,
        default=12.0,
        help="Seconds to wait after CMD_START before sending the first waypoint.",
    )
    parser.add_argument(
        "--arrival-timeout",
        type=float,
        default=60.0,
        help="Seconds allowed for each waypoint; a timeout ends the search and starts return.",
    )
    parser.add_argument(
        "--arrival-radius",
        type=float,
        default=0.25,
        help="Horizontal distance threshold (meters) to accept a waypoint as reached (default: 0.25).",
    )
    parser.add_argument(
        "--connect-timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for the first telemetry packet (default: 30).",
    )
    parser.add_argument(
        "--stale-timeout",
        type=float,
        default=3.0,
        help="Land if telemetry is older than this during the mission (default: 3).",
    )
    parser.add_argument(
        "--finish-action",
        choices=["hold", "land"],
        default="land",
        help="Action after returning to the start: hold or land (default: land).",
    )
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="Require a confirmation prompt before sending any flight commands.",
    )
    parser.add_argument(
        "--live-telem",
        action="store_true",
        help="Print each incoming telemetry update from the drone.",
    )
    parser.add_argument("--dwell", type=float, default=1.0,
                        help="Detection dwell at each reached outbound waypoint (s).")
    parser.add_argument("--max-search-time", type=float, default=300.0,
                        help="Outbound waypoint budget (s); return and landing are additional.")
    parser.add_argument("--camera-timeout", type=float, default=3.0)
    parser.add_argument("--pose-max-age", type=float, default=0.5)
    parser.add_argument("--assumed-altitude", type=float,
                        help="Assumed cruise height above local NED Z=0 for tag localization only.")
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--quality", type=int, default=60)
    add_tag_arguments(parser)
    parser.add_argument("--log", type=Path,
                        default=Path("waypoints_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ") + ".jsonl"),
                        help="Tag log: .csv for tag ID and local X/Y/Z only; .jsonl for detailed records. "
                             "Default: timestamped waypoints_*.jsonl in the working directory.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Validate and print the waypoints without opening sockets or flying.")
    args = parser.parse_args(join_waypoint_values(sys.argv[1:] if argv is None else argv))
    positive = (args.arrival_timeout, args.arrival_radius, args.connect_timeout,
                args.stale_timeout, args.camera_timeout, args.pose_max_age, args.max_search_time)
    if not all(math.isfinite(v) and v > 0 for v in positive):
        parser.error("Timeouts and arrival radius must be finite and positive.")
    if not all(math.isfinite(v) and v >= 0 for v in (args.takeoff_wait, args.dwell)):
        parser.error("Takeoff wait and dwell must be finite and non-negative.")
    if args.assumed_altitude is not None and (
            not math.isfinite(args.assumed_altitude) or args.assumed_altitude <= 0):
        parser.error("Assumed altitude must be finite and positive.")
    if not 0 <= args.drone_id <= 255:
        parser.error("Drone ID must be 0-255.")
    if not all(1 <= port <= 65535 for port in (args.telem_port, args.cmd_port)):
        parser.error("UDP ports must be 1-65535.")
    from protocol import build_camera_stream_command
    try:
        validate_tag_arguments(args)
        build_camera_stream_command(True, args.fps, args.quality)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def send_command(comms: CommsNode, drone_id: int, command_type: int, name: str,
                 repeat: int = 1) -> None:
    for _ in range(repeat):
        if not comms.send_command(drone_id, CommandPacket(command_type)):
            raise RuntimeError(f"Failed to send {name}: drone {drone_id} IP is unknown.")
        if repeat > 1:
            time.sleep(0.1)
    log.info("%s sent to drone %d.", name, drone_id)


def send_goto(comms: CommsNode, drone_id: int, goal_x: float, goal_y: float) -> None:
    if not comms.send_command(drone_id, CommandPacket(CMD_GOTO, goal_x=goal_x, goal_y=goal_y)):
        raise RuntimeError(f"Failed to send CMD_GOTO: drone {drone_id} IP is unknown.")


def wait_for_arrival(
    tracker: TelemetryTracker,
    resend,
    timeout_s: float,
    stale_timeout_s: float,
    goal_x: float,
    goal_y: float,
    arrival_radius_m: float,
    check_health=None,
) -> bool:
    start = last_sent = time.monotonic()
    deadline = start + timeout_s
    saw_moving = False           # drone reported ROTATING/FLYING/STUCK

    while time.monotonic() < deadline:
        if check_health is not None:
            check_health()
        now = time.monotonic()
        snapshot = tracker.latest()
        packet = snapshot.packet
        age = now - snapshot.received_at
        if packet is None or age > stale_timeout_s:
            raise ConnectionError(f"Telemetry is stale ({age:.1f} s).")

        if snapshot.received_at > start:
            dist_to_goal = math.hypot(packet.ned_x - goal_x, packet.ned_y - goal_y)

            log.info(
                "Current position: x=%.2f y=%.2f (goal x=%.2f y=%.2f, dist=%.2f m)",
                packet.ned_x,
                packet.ned_y,
                goal_x,
                goal_y,
                dist_to_goal,
            )

            if packet.nav_state in (NAV_ROTATING, NAV_FLYING, NAV_STUCK):
                saw_moving = True

            if dist_to_goal <= arrival_radius_m:
                log.info(
                    "Waypoint reached at (%.2f, %.2f), goal=(%.2f, %.2f), dist=%.2f m",
                    packet.ned_x,
                    packet.ned_y,
                    goal_x,
                    goal_y,
                    dist_to_goal,
                )
                return True

            if not saw_moving and now - last_sent >= GOTO_RESEND_S:
                log.info("Drone not moving yet (nav=%s) — resending CMD_GOTO", packet.nav_state_name)
                resend()
                last_sent = now

        time.sleep(0.2)

    packet = tracker.latest().packet
    log.warning(
        "Waypoint arrival timeout; last nav_state=%s at (%.2f, %.2f), goal=(%.2f, %.2f), dist=%.2f m",
        packet.nav_state_name,
        packet.ned_x,
        packet.ned_y,
        goal_x,
        goal_y,
        math.hypot(packet.ned_x - goal_x, packet.ned_y - goal_y),
    )
    return False


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        waypoints = list(args.waypoints)
        if args.waypoints_csv:
            waypoints.extend(load_waypoints_from_csv(args.waypoints_csv))
        if args.waypoints_file:
            waypoints.extend(load_waypoints_from_file(args.waypoints_file))
        if not waypoints:
            raise ValueError("No waypoints supplied. Use --waypoints-csv, --waypoint, or --waypoints-file.")
    except (OSError, ValueError, argparse.ArgumentTypeError) as exc:
        log.error("Invalid waypoint input: %s", exc)
        return 1

    # Import after defining the base helpers: the exploration runner and tag
    # tracker reuse them. Both entry points share the same flight lifecycle.
    from simple_exploration_mission import run_waypoint_mission
    return run_waypoint_mission(args, waypoints, metadata={
        "route_source": args.waypoints_csv or args.waypoints_file or "command_line",
    })


if __name__ == "__main__":
    sys.exit(main())
