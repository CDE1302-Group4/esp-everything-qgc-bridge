"""Headless camera detection and timestamped local-frame mission records.

Uses the same detector, camera mount, quality gate and JPEG assembly as
tag_stream.py. Telemetry and camera clocks are not synchronized: positions
are associated by laptop receipt time, with the time mismatch logged.
"""

from __future__ import annotations

from collections import deque
import csv
from dataclasses import replace
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import select
import socket
import threading
import time

from apriltag_host import Detector, POSE_ERR_LIMIT, gate_reason
from camera_stream import FrameAssembler, KEEPALIVE_S, drain_socket
from protocol import CAMERA_STREAM_PORT, build_camera_stream_command
from simple_waypoint_mission import TelemetrySnapshot, TelemetryTracker


class PoseTracker(TelemetryTracker):
    """Keep recent poses so detector runtime does not shift localization."""

    def __init__(self, drone_id, print_live=False, map_offset=(0.0, 0.0)):
        super().__init__(drone_id, print_live)
        self._history = deque(maxlen=600)
        self._map_offset = map_offset

    def callback(self, packet, source_ip):
        if packet.drone_id != self.drone_id:
            return
        with self._condition:
            super().callback(packet, source_ip)
            snapshot = self.latest()
            # Navigation consumes the original map-frame packet via latest().
            # Detection history always uses PX4 local NED, even in map mode.
            local_packet = replace(packet, ned_x=packet.ned_x - self._map_offset[0],
                                   ned_y=packet.ned_y - self._map_offset[1])
            self._history.append(replace(snapshot, packet=local_packet))

    def pose_at(self, received_at, max_age):
        with self._condition:
            # Only use telemetry received by the image's first packet, never
            # a newer pose obtained after a slow detector invocation.
            for snapshot in reversed(self._history):
                if snapshot.received_at <= received_at:
                    if received_at - snapshot.received_at <= max_age:
                        return snapshot
                    break
        return TelemetrySnapshot()


CSV_SCALAR_FIELDS = (
    "type", "timestamp_utc", "drone_id", "phase", "tag_id", "tag_family",
    "frame_id", "camera_boot_nonce", "frame_received_utc", "frame_received_monotonic_s",
    "camera_age_ms", "camera_dropped_frames", "hamming", "decision_margin",
    "quality_accepted", "quality_rejection_reason", "pose_valid", "pose_error",
    "camera_range_m", "localization_status", "uav_heading_rad", "telemetry_age_at_frame_s",
    "assumed_altitude_m", "reached", "search_complete", "landing_confirmed",
)
CSV_VECTOR_FIELDS = {
    "image_center_px": ("image_center_x_px", "image_center_y_px"),
    "image_corners_px": tuple(f"image_corner_{i}_{axis}_px" for i in range(4) for axis in ("x", "y")),
    "relative_camera_translation_m": ("camera_x_m", "camera_y_m", "camera_z_m"),
    "relative_body_translation_m": ("body_forward_m", "body_right_m", "body_down_m"),
    "local_position_ned_m": ("local_x_m", "local_y_m", "local_z_m"),
    "uav_local_xy_m": ("uav_local_x_m", "uav_local_y_m"),
    "local_xy_m": ("goal_local_x_m", "goal_local_y_m"),
    "command_xy_m": ("command_x_m", "command_y_m"),
    "home_local_xy_m": ("home_local_x_m", "home_local_y_m"),
    **{field: tuple(f"{prefix}_r{i}{j}" for i in range(3) for j in range(3))
       for field, prefix in (("relative_camera_rotation_row_major", "camera"),
                             ("relative_body_rotation_row_major", "body"),
                             ("local_rotation_row_major", "local"))},
}
CSV_FIELDS = (*CSV_SCALAR_FIELDS,
              *(column for columns in CSV_VECTOR_FIELDS.values() for column in columns),
              "details_json")


def csv_record(record):
    """Flatten poses for spreadsheets; retain other mission metadata as JSON."""
    row = {key: record[key] for key in CSV_SCALAR_FIELDS if key in record}
    for field, columns in CSV_VECTOR_FIELDS.items():
        values = record.get(field)
        if values is not None:
            if field == "image_corners_px":
                values = tuple(value for corner in values for value in corner)
            if len(values) != len(columns):
                raise ValueError(f"{field}: expected {len(columns)} values")
            row.update(zip(columns, values))
    extra = {key: value for key, value in record.items()
             if key not in CSV_SCALAR_FIELDS and key not in CSV_VECTOR_FIELDS}
    row["details_json"] = json.dumps(extra, allow_nan=False) if extra else ""
    return row


class Journal:
    """CSV for .csv paths, otherwise JSONL; flush every event/detection."""

    def __init__(self, path):
        self._file = open(path, "x", encoding="utf-8", newline="")
        self._lock = threading.Lock()
        self._csv = None
        if Path(path).suffix.lower() == ".csv":
            try:
                self._csv = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
                self._csv.writeheader()
                self._file.flush()
            except Exception:
                self._file.close()
                raise

    def write(self, kind, **fields):
        record = {"type": kind, "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                  **fields}
        line = json.dumps(record, allow_nan=False)
        row = csv_record(record) if self._csv is not None else None
        with self._lock:
            if self._csv is not None:
                self._csv.writerow(row)
            else:
                self._file.write(line + "\n")
            self._file.flush()

    def close(self):
        with self._lock:
            self._file.close()


def detection_record(det, snapshot, frame_received_at, assumed_altitude=None):
    """Record every decoded ID; only quality-approved poses get local estimates."""
    rejection = gate_reason(det)
    if not math.isfinite(det.decision_margin):
        rejection = "non_finite_decision_margin"
    pose_values = (*(det.t or ()), *(det.R or ()))
    pose_valid = (det.pose_valid and det.t is not None and det.R is not None
                  and det.pose_err is not None and 0 <= det.pose_err < POSE_ERR_LIMIT
                  and all(math.isfinite(v) for v in pose_values))
    record = dict(tag_id=det.id, tag_family="tag16h5", hamming=det.hamming,
                  decision_margin=det.decision_margin if math.isfinite(det.decision_margin) else None,
                  quality_accepted=rejection is None, quality_rejection_reason=rejection,
                  image_center_px=det.centre, image_corners_px=det.corners,
                  pose_valid=bool(pose_valid), pose_error=det.pose_err,
                  camera_range_m=math.hypot(*det.t) if pose_valid else None,
                  relative_camera_translation_m=det.t,
                  relative_camera_rotation_row_major=det.R,
                  relative_body_translation_m=det.t_body,
                  relative_body_rotation_row_major=det.R_body,
                  local_position_ned_m=None, local_rotation_row_major=None,
                  uav_local_xy_m=None, uav_heading_rad=None, telemetry_age_at_frame_s=None,
                  localization_status="invalid_pose")
    packet = snapshot.packet
    telemetry_valid = packet is not None and all(
        math.isfinite(v) for v in (packet.ned_x, packet.ned_y, packet.heading_rad))
    if telemetry_valid:
        record.update(uav_local_xy_m=(packet.ned_x, packet.ned_y),
                      uav_heading_rad=packet.heading_rad,
                      telemetry_age_at_frame_s=frame_received_at - snapshot.received_at)
    if not pose_valid:
        # Preserve the ID even if pose estimation failed; valid JSON only.
        for key in ("relative_camera_translation_m", "relative_camera_rotation_row_major",
                    "relative_body_translation_m", "relative_body_rotation_row_major"):
            record[key] = None
        if det.pose_err is not None and not math.isfinite(det.pose_err):
            record["pose_error"] = None
    if rejection is not None:
        record["localization_status"] = "rejected_quality"
        return record
    if not pose_valid:
        return record
    if packet is None:
        record["localization_status"] = "no_matching_telemetry"
        return record
    if not telemetry_valid:
        record["localization_status"] = "invalid_telemetry"
        return record
    c, s = math.cos(packet.heading_rad), math.sin(packet.heading_rad)
    fwd, right, down = det.t_body
    rotation = ((c, -s, 0), (s, c, 0), (0, 0, 1))
    body_rotation = det.R_body
    record.update(
        local_position_ned_m=(packet.ned_x + c * fwd - s * right,
                              packet.ned_y + s * fwd + c * right,
                              None if assumed_altitude is None else down - assumed_altitude),
        local_rotation_row_major=tuple(
            sum(rotation[i][k] * body_rotation[3 * k + j] for k in range(3))
            for i in range(3) for j in range(3)),
        assumed_altitude_m=assumed_altitude,
        localization_status="estimated_level_attitude",
    )
    return record


class TagRecorder:
    def __init__(self, esp_ip, cmd_port, tracker, journal, *, fps=5, quality=60,
                 pose_max_age=0.5, camera_timeout=3.0, assumed_altitude=None):
        self.esp_ip, self.cmd_port = esp_ip, cmd_port
        self.tracker, self.journal = tracker, journal
        self.pose_max_age, self.camera_timeout = pose_max_age, camera_timeout
        self.assumed_altitude = assumed_altitude
        self._phase = "preflight"
        self._start_command = build_camera_stream_command(True, fps, quality)
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._error = None
        self._last_frame = 0.0
        self._thread = threading.Thread(target=self._run, name="mission-tags", daemon=True)

    def set_phase(self, phase):
        self._phase = phase

    def start(self, timeout):
        self._thread.start()
        if not self._ready.wait(timeout):
            raise TimeoutError("No decoded camera frame before takeoff.")
        self.check_health()

    def check_health(self):
        if self._error is not None:
            raise RuntimeError(f"AprilTag recorder failed: {self._error}") from self._error
        if time.monotonic() - self._last_frame > self.camera_timeout:
            raise ConnectionError("Camera stream is stale; cannot continue searching.")

    def stop(self):
        self._stop.set()
        if self._thread.ident is not None:
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                raise RuntimeError("AprilTag recorder did not stop within 5 s.")

    def _run(self):
        try:
            import cv2
            import numpy as np

            with Detector() as detector, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as rx, \
                    socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
                # Exclusive bind: competing viewers must not steal our frames.
                rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
                rx.bind(("0.0.0.0", CAMERA_STREAM_PORT))
                rx.setblocking(False)
                assembler = FrameAssembler()
                next_keepalive = 0.0
                target = (self.esp_ip, self.cmd_port)
                try:
                    while not self._stop.is_set():
                        if time.monotonic() >= next_keepalive:
                            tx.sendto(self._start_command, target)
                            next_keepalive = time.monotonic() + KEEPALIVE_S
                        select.select([rx], [], [], 0.1)
                        frame, _, version = drain_socket(rx, assembler, self.esp_ip)
                        assembler.expire(time.monotonic())
                        if version is not None:
                            raise RuntimeError(f"Unsupported camera protocol version {version}")
                        if frame is None:
                            continue
                        phase = self._phase
                        gray = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
                        if gray is None:
                            continue
                        if gray.shape != (240, 320):
                            raise RuntimeError("Camera image must match the 320x240 detector calibration.")
                        snapshot = self.tracker.pose_at(frame.first_rx, self.pose_max_age)
                        receipt_utc = datetime.fromtimestamp(
                            time.time() - (time.monotonic() - frame.first_rx), timezone.utc).isoformat()
                        altitude = self.assumed_altitude if phase in ("exploring", "returning") else None
                        for det in detector.detect(gray):
                            self.journal.write(
                                "detection", drone_id=self.tracker.drone_id, phase=phase,
                                frame_id=frame.frame_id, camera_boot_nonce=assembler.nonce,
                                frame_received_utc=receipt_utc,
                                frame_received_monotonic_s=frame.first_rx,
                                camera_age_ms=frame.age_ms, camera_dropped_frames=assembler.dropped,
                                **detection_record(det, snapshot, frame.first_rx, altitude))
                        self._last_frame = frame.first_rx
                        self._ready.set()
                finally:
                    try:
                        tx.sendto(build_camera_stream_command(False), target)
                    except OSError:
                        pass
        except Exception as exc:
            self._error = exc
            self._ready.set()
