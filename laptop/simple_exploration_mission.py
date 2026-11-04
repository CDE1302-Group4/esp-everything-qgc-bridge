#!/usr/bin/env python3
"""Take off, survey a rectangle for AprilTags, return to the start and land.

Built on simple_waypoint_mission.py. Bounds are metres relative to the
recorded starting position, along local NED X/Y (not body forward/right).
Place the UAV at its assigned start before running. See TEST_GUIDE.md.
Updated nav_task firmware holds heading during the sweep and return; the UAV
can move backward or sideways without turning. This requires an ESP32 reflash.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import logging
import math
from pathlib import Path
import sys
import time

from comms import CommsNode
from mission_tags import Journal, PoseTracker, TagRecorder
from protocol import CMD_HOLD, CMD_LAND, CMD_START
from simple_waypoint_mission import send_command, send_goto, wait_for_arrival
from tag_acceptance import add_tag_arguments, validate_tag_arguments

log = logging.getLogger("simple_exploration_mission")


def lawnmower(bounds, spacing):
    """Sweep X, alternating direction on Y rows, including all four edges."""
    xmin, xmax, ymin, ymax = bounds
    if (not all(math.isfinite(v) for v in (*bounds, spacing))
            or xmin >= xmax or ymin >= ymax or spacing <= 0):
        raise ValueError("Bounds must be finite and ordered; spacing must be positive.")
    rows = max(1, math.ceil((ymax - ymin) / spacing))
    if rows > 5000:
        raise ValueError("Search path exceeds 5000 rows; increase --lane-spacing.")
    points = []
    for row in range(rows + 1):
        y = ymin + (ymax - ymin) * row / rows
        xs = (xmin, xmax) if row % 2 == 0 else (xmax, xmin)
        points.extend((x, y) for x in xs)
    # Start at whichever of the four possible sweep corners is closest to home.
    variants = [points, list(reversed(points))]
    reflected = [(x, ymin + ymax - y) for x, y in points]
    variants.extend((reflected, list(reversed(reflected))))
    return min(variants, key=lambda route: math.hypot(*route[0]))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drone-id", required=True, type=int)
    parser.add_argument("--bounds", required=True, type=float, nargs=4,
                        metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                        help="Search rectangle relative to the starting position, NED metres.")
    parser.add_argument("--lane-spacing", type=float, default=0.5,
                        help="Maximum distance between sweep rows (m); tune to camera coverage.")
    parser.add_argument("--takeoff-wait", type=float, default=12.0)
    parser.add_argument("--arrival-timeout", type=float, default=60.0)
    parser.add_argument("--arrival-radius", type=float, default=0.2)
    parser.add_argument("--dwell", type=float, default=1.0,
                        help="Detection dwell at each reached survey waypoint (s).")
    parser.add_argument("--max-search-time", type=float, default=300.0,
                        help="Search budget after takeoff (s); return/landing are additional.")
    parser.add_argument("--connect-timeout", type=float, default=30.0)
    parser.add_argument("--stale-timeout", type=float, default=3.0)
    parser.add_argument("--camera-timeout", type=float, default=3.0)
    parser.add_argument("--pose-max-age", type=float, default=0.5,
                        help="Maximum telemetry/image receipt-time difference for localization.")
    parser.add_argument("--assumed-altitude", type=float,
                        help="Optional assumed cruise height above local NED Z=0, for estimated tag Z. "
                             "Does not set flight altitude; absent means local Z is null.")
    parser.add_argument("--fps", type=int, default=5)
    parser.add_argument("--quality", type=int, default=60)
    add_tag_arguments(parser)
    parser.add_argument("--telem-port", type=int, default=5005)
    parser.add_argument("--cmd-port", type=int, default=5006)
    parser.add_argument("--log", type=Path,
                        default=Path("exploration_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ") + ".csv"),
                        help="Tag log: .csv for tag ID and local X/Y/Z only (default); .jsonl for detailed records.")
    parser.add_argument("--confirm", action="store_true",
                        help="Require typing MISSION-<id> before starting flight.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print planned offsets and exit without opening sockets or flying.")
    parser.add_argument("--live-telem", action="store_true")
    args = parser.parse_args(argv)
    try:
        validate_tag_arguments(args)
        lawnmower(args.bounds, args.lane_spacing)
        positive = (args.takeoff_wait, args.arrival_timeout, args.arrival_radius,
                    args.max_search_time, args.connect_timeout, args.stale_timeout,
                    args.camera_timeout, args.pose_max_age)
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError("Timeouts, takeoff wait and arrival radius must be finite and positive.")
        if not math.isfinite(args.dwell) or args.dwell < 0:
            raise ValueError("Dwell must be finite and non-negative.")
        if args.assumed_altitude is not None and (
                not math.isfinite(args.assumed_altitude) or args.assumed_altitude <= 0):
            raise ValueError("Assumed altitude must be finite and positive.")
        if not 0 <= args.drone_id <= 255:
            raise ValueError("Drone ID must be 0-255.")
        if not all(1 <= port <= 65535 for port in (args.telem_port, args.cmd_port)):
            raise ValueError("UDP ports must be 1-65535.")
        from protocol import build_camera_stream_command
        build_camera_stream_command(True, args.fps, args.quality)
    except ValueError as exc:
        parser.error(str(exc))
    return args


def checked_wait(duration, check_health):
    deadline = time.monotonic() + duration
    while True:
        check_health()
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(0.1, remaining))


class Mission:
    def __init__(self, args, comms, tracker, recorder, journal):
        self.args, self.comms, self.tracker = args, comms, tracker
        self.recorder, self.journal = recorder, journal
        self.start_attempted = False
        self.land_sent = False
        self.hold_sent = False
        self.map_offset = getattr(args, "map_offset", (0.0, 0.0))

    def to_local(self, point):
        return (point[0] - self.map_offset[0], point[1] - self.map_offset[1])

    def phase(self, name, **fields):
        self.recorder.set_phase(name)
        self.journal.write("phase", phase=name, **fields)
        log.info("Mission phase: %s", name)

    def telemetry_health(self):
        snapshot = self.tracker.latest()
        if snapshot.packet is None or time.monotonic() - snapshot.received_at > self.args.stale_timeout:
            raise ConnectionError("Telemetry is stale.")
        p = snapshot.packet
        if not all(math.isfinite(v) for v in (p.ned_x, p.ned_y, p.heading_rad)):
            raise ValueError("Telemetry position/heading is non-finite.")

    def check_health(self):
        self.telemetry_health()
        self.recorder.check_health()

    def goto(self, goal, timeout):
        self.check_health()

        def resend():
            send_goto(self.comms, self.args.drone_id, *goal)

        self.journal.write("goal", local_xy_m=self.to_local(goal), command_xy_m=goal)
        resend()
        reached = wait_for_arrival(
            self.tracker, resend, timeout_s=timeout,
            stale_timeout_s=self.args.stale_timeout, goal_x=goal[0], goal_y=goal[1],
            arrival_radius_m=self.args.arrival_radius, check_health=self.check_health)
        self.journal.write("arrival", local_xy_m=self.to_local(goal), reached=reached)
        return reached

    def land(self):
        # Flight command must still run if writing the journal has failed.
        self.recorder.set_phase("landing")
        send_command(self.comms, self.args.drone_id, CMD_LAND, "CMD_LAND", repeat=3)
        self.land_sent = True
        self.journal.write("landing_command_sent", landing_confirmed=False)
        log.info("LAND requested. This protocol cannot confirm touchdown/disarm; observe QGC.")

    def fly(self, home, offsets):
        self.check_health()
        self.phase("takeoff", home_local_xy_m=self.to_local(home))
        self.start_attempted = True
        send_command(self.comms, self.args.drone_id, CMD_START, "CMD_START")
        checked_wait(self.args.takeoff_wait, self.check_health)

        self.phase("exploring")
        deadline = time.monotonic() + self.args.max_search_time
        visited = [home]
        complete = True
        for x, y in offsets:
            goal = (x, y) if getattr(self.args, "map_frame", False) else (home[0] + x, home[1] + y)
            remaining = deadline - time.monotonic()
            if remaining <= 0 or not self.goto(goal, min(self.args.arrival_timeout, remaining)):
                complete = False
                send_command(self.comms, self.args.drone_id, CMD_HOLD, "CMD_HOLD")
                self.journal.write("search_incomplete", reason="search budget or waypoint timeout")
                break
            visited.append(goal)
            checked_wait(min(self.args.dwell, max(0, deadline - time.monotonic())), self.check_health)

        self.phase("returning", search_complete=complete)
        # Retrace reached goals; after a failed leg first go back to its origin.
        for goal in reversed(visited):
            if not self.goto(goal, self.args.arrival_timeout):
                raise TimeoutError("Return waypoint timed out; landing at current position.")
        if getattr(self.args, "finish_action", "land") == "hold":
            send_command(self.comms, self.args.drone_id, CMD_HOLD, "CMD_HOLD at start")
            self.phase("holding", home_local_xy_m=self.to_local(home))
            self.hold_sent = True
        else:
            self.land()
        return 0 if complete else 1


def main(argv=None):
    args = parse_args(argv)
    offsets = lawnmower(args.bounds, args.lane_spacing)
    log.info("Search: %d goals; bounds=%s, maximum row spacing=%.2f m",
             len(offsets), args.bounds, args.lane_spacing)
    return run_waypoint_mission(args, offsets, metadata={
        "route_source": "lawnmower", "bounds_relative_to_start_m": args.bounds,
    })


def run_waypoint_mission(args, waypoints, metadata=None):
    """Shared camera/logging/flight lifecycle for generated and supplied routes."""
    config_path = args.config if getattr(args, "map_frame", False) else None
    map_offset = (0.0, 0.0)
    if config_path is not None:
        try:
            from exploration import load_drone_starts
            starts = load_drone_starts(config_path)
            if args.drone_id not in starts:
                raise ValueError(f"Drone {args.drone_id} has no starting offset in {config_path}")
            map_offset = tuple(float(v) for v in starts[args.drone_id])
            if not all(math.isfinite(v) for v in map_offset):
                raise ValueError("Configured starting offset must be finite.")
        except Exception as exc:
            log.error("Invalid map-frame configuration: %s", exc)
            return 1
    args.map_offset = map_offset
    waypoint_frame = "arena" if config_path is not None else "relative_to_recorded_start"
    if args.dry_run:
        print(f"Waypoint frame: {waypoint_frame}; units: metres (NED X/Y).")
        for index, (x, y) in enumerate(waypoints, 1):
            print(f"{index}: {x:.3f}, {y:.3f}")
        print("Return: retrace reached waypoints to recorded start, then "
              + getattr(args, "finish_action", "land").upper() + ".")
        return 0

    journal = recorder = mission = None
    tracker = PoseTracker(args.drone_id, args.live_telem, map_offset)
    comms = CommsNode(listen_port=args.telem_port, cmd_port=args.cmd_port, config_path=config_path)
    if config_path is None:
        # Match the base script's local-frame mode, clearing old arena offsets.
        comms.set_nav_tags([], {})
    comms.on_telemetry(tracker.callback)
    try:
        journal = Journal(args.log)
        log.info("Recording AprilTag observations to %s", Path(args.log).resolve())
        journal.write("mission", drone_id=args.drone_id, waypoints_m=waypoints,
                      waypoint_frame=waypoint_frame, map_offset_m=map_offset,
                      frame="PX4 local NED for detections",
                      detection_policy="exact decode, valid local pose, consistent consecutive frames",
                      tag_min_margin=args.tag_min_margin, tag_confirm_frames=args.tag_confirm_frames,
                      tag_ids=args.tag_ids,
                      timestamp_basis="laptop receipt; no synchronized capture clock",
                      attitude_assumption="roll=pitch=0", assumed_cruise_altitude_m=args.assumed_altitude,
                      **(metadata or {}))
        comms.start()
        tracker.wait_for_first_packet(args.connect_timeout, args.telem_port, comms.known_drones)
        recorder = TagRecorder(tracker.latest().source_ip, args.cmd_port, tracker, journal,
                               fps=args.fps, quality=args.quality, pose_max_age=args.pose_max_age,
                               camera_timeout=args.camera_timeout, assumed_altitude=args.assumed_altitude,
                               tag_min_margin=args.tag_min_margin, tag_confirm_frames=args.tag_confirm_frames,
                               tag_ids=args.tag_ids)
        mission = Mission(args, comms, tracker, recorder, journal)
        # Give the configured frame offset time to apply before capturing home.
        for _ in range(3):
            comms.send_nav_tags(args.drone_id)
            checked_wait(0.3, mission.telemetry_health)
        recorder.start(args.connect_timeout)
        if args.confirm:
            if input(f"Type MISSION-{args.drone_id} to take off and explore: ").strip() != f"MISSION-{args.drone_id}":
                journal.write("cancelled")
                return 0
        mission.check_health()
        packet = tracker.latest().packet
        home = (packet.ned_x, packet.ned_y)
        log.info("Recorded starting position: (%.2f, %.2f); records: %s", *home, args.log)
        return mission.fly(home, waypoints)
    except KeyboardInterrupt:
        log.warning("Mission interrupted; requesting landing if START was attempted.")
        return 130
    except Exception as exc:
        log.error("Mission failed: %s", exc)
        return 1
    finally:
        if (mission is not None and mission.start_attempted
                and not mission.land_sent and not mission.hold_sent):
            try:
                mission.land()
            except Exception as exc:
                log.error("Fallback LAND failed: %s", exc)
        if recorder is not None:
            try:
                recorder.stop()
            except Exception as exc:
                log.error("Recorder shutdown failed: %s", exc)
        comms.stop()
        if journal is not None:
            journal.close()


if __name__ == "__main__":
    sys.exit(main())
