"""UI-independent connection, camera worker and flight controller for Drone Desk."""

from dataclasses import dataclass
import ipaddress
import logging
import math
from pathlib import Path
import queue
import select
import socket
import threading
import time
from collections import deque

import cv2
import numpy as np

from .detector import CAM_POS_M, R_BODY_CAM, Detection, Detector, passes_gate
from camera_stream import Frame, FrameAssembler, KEEPALIVE_S, drain_socket
from .transport import CommsNode
from protocol import (CAMERA_STREAM_PORT, CMD_HOLD, CMD_LAND, CommandPacket,
                      TelemetryPacket, build_camera_stream_command)
from .flight import (FlightCancelled, FlightControl, FlightFeedback,
                             FlightOptions, load_trajectory, run_flight)

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class Route:
    path: Path
    points: np.ndarray
    dt_ms: int
    duration: float
    peak_speed: float

    @classmethod
    def load(cls, path, dt=0.05, max_speed=0.30):
        if not math.isfinite(dt) or not 1 <= round(dt * 1000) <= 1000:
            raise ValueError("Sample interval must be between 0.001 and 1 second")
        dt_ms = round(dt * 1000)
        points = load_trajectory(str(path), dt_ms / 1000)
        speed = float(np.linalg.norm(np.diff(points, axis=0), axis=1).max() * 1000 / dt_ms)
        if speed > max_speed + 1e-6:
            raise ValueError(f"Peak speed {speed:.3f} m/s exceeds {max_speed:.2f} m/s")
        points.setflags(write=False)
        return cls(Path(path).resolve(), points, dt_ms, (len(points) - 1) * dt_ms / 1000, speed)


class DroneSession:
    """One CommsNode shared by the display and the cancellable flight worker.

    Workers never call Tk or Matplotlib. Camera frames are replaced in place;
    timestamped telemetry is buffered briefly for the main thread's maps.
    """

    def __init__(self, drone_id=4, esp_ip="192.168.1.204", *, config=None,
                 telem_port=5005, cmd_port=5006, demo=False):
        if not 0 <= drone_id <= 30:
            raise ValueError("Drone ID must be between 0 and 30")
        self.drone_id = drone_id
        self.esp_ip = str(ipaddress.IPv4Address(esp_ip.strip()))
        self.demo = demo
        self.config = config
        self.telem_port, self.cmd_port = telem_port, cmd_port
        self.started = time.monotonic()
        self.feedback = FlightFeedback(drone_id)
        self.comms = None
        self.events = queue.Queue(maxsize=1000)
        self._lock = threading.Lock()
        self._samples = deque(maxlen=300)
        self._packet = None
        self._packet_time = None
        self._frame = None
        self._frame_number = 0
        self._stop = threading.Event()
        self._camera_thread = None
        self._operation_thread = None
        self.control = None
        self.ever_controlled = False

    def post(self, kind, value):
        try:
            self.events.put_nowait((kind, value))
        except queue.Full:
            pass

    @property
    def busy(self):
        return self._operation_thread is not None and self._operation_thread.is_alive()

    def start(self):
        if self.demo:
            self._camera_thread = threading.Thread(target=self._demo_loop, daemon=True)
        else:
            self.comms = CommsNode(self.telem_port, self.cmd_port, self.config,
                                   reuse_address=False)
            self.comms.on_telemetry(self._on_telemetry)
            try:
                self.comms.start()
            except OSError as exc:
                raise ConnectionError(f"Cannot listen on telemetry UDP {self.telem_port}. "
                                      "Close other flight scripts using that port. " + str(exc)) from exc
            self._camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        self._camera_thread.start()

    def _on_telemetry(self, packet, source_ip):
        if packet.drone_id != self.drone_id or source_ip != self.esp_ip:
            return
        now = time.monotonic()
        self.feedback.update(packet, source_ip)
        with self._lock:
            self._packet, self._packet_time = packet, now
            self._samples.append((packet, now - self.started))

    def snapshot(self):
        with self._lock:
            samples = list(self._samples)
            self._samples.clear()
            return self._packet, self._packet_time, samples, self._frame_number, self._frame

    def _publish_frame(self, frame, gray, tags):
        with self._lock:
            self._frame = (frame, gray, tags, time.monotonic() - self.started)
            self._frame_number += 1

    def _camera_loop(self):
        rx, tx, detector = None, None, None
        try:
            rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                rx.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
            rx.bind(("0.0.0.0", CAMERA_STREAM_PORT))
            rx.setblocking(False)
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                self.post("camera_status", "Preparing AprilTag detector...")
                detector = Detector()
            except Exception as exc:
                self.post("camera_status", "Camera only: tag detector unavailable")
                log.error("Tag detector unavailable; camera will continue: %s", exc)
            assembler = FrameAssembler()
            next_keepalive = 0
            warned_version = False
            while not self._stop.is_set():
                now = time.monotonic()
                if now >= next_keepalive:
                    next_keepalive = now + KEEPALIVE_S
                    tx.sendto(build_camera_stream_command(True, 5, 60),
                              (self.esp_ip, self.cmd_port))
                select.select([rx], [], [], 0.02)
                frame, _, bad_version = drain_socket(rx, assembler, self.esp_ip)
                assembler.expire(time.monotonic())
                if bad_version is not None and not warned_version:
                    warned_version = True
                    log.error("Camera packet version %s is unsupported by this viewer", bad_version)
                    self.post("camera_status", "Camera protocol version mismatch")
                if frame is None:
                    continue
                gray = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
                if gray is None:
                    continue
                tags = []
                if detector is not None:
                    try:
                        tags = [d for d in detector.detect(gray) if passes_gate(d)]
                    except Exception as exc:
                        log.error("Tag detection stopped; camera will continue: %s", exc)
                        detector.close()
                        detector = None
                self._publish_frame(frame, gray, tags)
        except Exception as exc:
            log.error("Camera worker: %s", exc)
            self.post("camera_status", f"Camera unavailable: {exc}")
        finally:
            if tx is not None:
                try:
                    tx.sendto(build_camera_stream_command(False), (self.esp_ip, self.cmd_port))
                except OSError:
                    pass
                tx.close()
            if rx is not None:
                rx.close()
            if detector is not None:
                detector.close()

    def _check_target(self, drone_id):
        if self.demo or self.comms is None:
            raise ConnectionError("Flight commands are disabled in demo/disconnected mode")
        if drone_id != self.drone_id or self.comms.known_drones().get(drone_id) != self.esp_ip:
            raise ConnectionError("Selected drone ID/IP has not been verified by telemetry")

    def send_command(self, drone_id, command):
        self._check_target(drone_id)
        return self.comms.send_command(drone_id, command)

    def send_raw(self, drone_id, data):
        self._check_target(drone_id)
        return self.comms.send_raw(drone_id, data)

    def launch(self, route=None, *, takeoff=True, takeoff_wait=12, finish="hold"):
        if self.busy:
            raise RuntimeError("An operation is already running")
        self._check_target(self.drone_id)
        self.feedback.packet()  # check feedback before any command
        if not math.isfinite(takeoff_wait) or not 0 <= takeoff_wait < 40:
            raise ValueError("Takeoff wait must be between 0 and 40 seconds")
        if finish not in ("hold", "land"):
            raise ValueError("Finish action must be hold or land")
        if route is None and not takeoff:
            raise ValueError("Choose takeoff or load a trajectory")
        if route is not None and takeoff and np.max(route.points[:, 2]) >= 0.4:
            raise ValueError("Route descends to within 0.10 m of the assumed ground reference")
        self.control = FlightControl()
        options = FlightOptions(self.drone_id, route.dt_ms if route else 50,
                                takeoff, takeoff_wait, 40, finish)
        self.ever_controlled = True

        def run():
            try:
                run_flight(self, self.feedback, route.points if route else None, options,
                           control=self.control, on_progress=lambda text: self.post("stage", text))
                self.post("operation_done", "takeoff" if route is None else "trajectory")
            except FlightCancelled:
                self.post("stage", "Operation stopped by operator")
            except Exception as exc:
                log.error("Flight operation failed: %s", exc)
                self.post("operation_error", str(exc))

        self._operation_thread = threading.Thread(target=run, daemon=True)
        self._operation_thread.start()

    def interrupt(self, command):
        if command not in (CMD_HOLD, CMD_LAND):
            raise ValueError("Interrupt action must be HOLD or LAND")
        control = self.control or FlightControl()
        self.control = control
        # Same lock as the runner's sends: no queued START can follow HOLD/LAND.
        sent, error = False, None
        for attempt in range(3):
            try:
                sent = control.interrupt(self, self.drone_id, command) or sent
            except (ConnectionError, OSError) as exc:
                error = exc
            if attempt < 2:
                time.sleep(.06)
        if not sent:
            raise ConnectionError(str(error) if error else "Command could not be sent")
        self.post("stage", "LAND requested" if command == CMD_LAND else "HOLD requested")

    def close(self):
        if self.busy and self.control is not None and self.control.cancelled.is_set():
            self._operation_thread.join(timeout=3)
        if self.busy:
            raise RuntimeError("Stop the active operation with HOLD or LAND before disconnecting")
        self._stop.set()
        if self._camera_thread:
            self._camera_thread.join(timeout=3)
        if self.comms:
            self.comms.stop()

    def _demo_loop(self):
        """Synthetic data only: no sockets, detector build or flight commands."""
        landmarks = {5: (1.8, -0.4), 8: (0.8, 0.3), 12: (2.0, 0.4)}
        index = 0
        while not self._stop.wait(0.1):
            now = time.monotonic()
            elapsed = now - self.started
            x = 0.55 + 0.45 * math.sin(elapsed * 0.13)
            y = 0.15 * math.sin(elapsed * 0.2)
            packet = TelemetryPacket(self.drone_id, x, y, 0, 5, -1, 0,
                                     [False] * 32, False, 65535)
            self._on_telemetry(packet, self.esp_ip)
            gray = np.full((360, 480), 38, np.uint8)
            for column in range(0, 480, 30):
                cv2.line(gray, (column, 0), (column, 359), 48, 1)
            for row in range(0, 360, 30):
                cv2.line(gray, (0, row), (479, row), 48, 1)
            cv2.putText(gray, "SIMULATED CAMERA", (18, 30), cv2.FONT_HERSHEY_SIMPLEX, .65, 210, 1)
            tags = []
            for i, (tag_id, (fwd, right)) in enumerate(landmarks.items()):
                cx, cy = 100 + i * 140, 165 + round(15 * math.sin(elapsed + i))
                corners = ((cx - 27, cy - 27), (cx + 27, cy - 27),
                           (cx + 27, cy + 27), (cx - 27, cy + 27))
                cv2.rectangle(gray, corners[0], corners[2], 240, -1)
                cv2.rectangle(gray, (cx - 20, cy - 20), (cx + 20, cy + 20), 0, -1)
                cv2.rectangle(gray, (cx - 12, cy - 12), (cx + 4, cy + 4), 240, -1)
                body = np.array((fwd - x, right - y, 0.7))
                camera = np.array(R_BODY_CAM).T @ (body - CAM_POS_M)
                tags.append(Detection(tag_id, 0, 100, (cx, cy), corners, True,
                                      tuple(camera), None, 1e-6))
            self._publish_frame(Frame(index, b"", 0, 0, now), gray, tags)
            index += 1
