#!/usr/bin/env python3
"""Live drone camera with AprilTags drawn on it.

    python3 laptop/tag_stream.py --esp-ip 192.168.1.222

Each frame is checked for tags on the laptop, with the same detector code the
drone runs. Every tag the drone would accept gets a green outline, its id and
where it is from the drone centre (forward, right, down, metres; camera mount
in apriltag_host.py). Matplotlib shows the current drone-relative tags on the
left and a persistent map relative to the starting position/heading on the
right. Start this viewer before moving the drone. Mapping uses telemetry
mirrored locally by send_trajectory.py or another CommsNode program; use
--telem-port 5005 for direct telemetry when running the viewer alone.
The figure is saved on exit to --plot-output (default:
tag_stream_plot_<timestamp>.png in the current directory).
With TF01 firmware, map offsets are removed and the flight path starts on arming.
Preflight estimate drift is shown as text, not as a flown path.
Keys: q or Esc quit, s saves the camera picture and plot, r resets the map.

For tuning (rejected tags, the drone's own detections, HUD) use
tag_stream_legacy.py.
"""

import argparse
import math
import select
import socket
import time
from collections import deque
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from apriltag_host import Detector, POSE_ERR_LIMIT, passes_gate
from camera_stream import COMMAND_PORT, KEEPALIVE_S, FrameAssembler, drain_socket
from protocol import (
    CAMERA_STREAM_PORT, TELEM_MIRROR_PORT, build_camera_stream_command,
    parse_telemetry,
)

GREEN = (0, 255, 0)     # OpenCV colours are BGR
BLACK = (0, 0, 0)
FONT = cv2.FONT_HERSHEY_SIMPLEX
MAX_POSE_AGE_S = 0.35


class StartMap:
    """Top-view map in the drone's initial forward/right frame (metres).

    Telemetry provides horizontal position and yaw, so this assumes a level
    drone. Align observations to nearby telemetry rather than old positions.
    """

    def __init__(self):
        self.origin = None
        self.poses = deque(maxlen=100)
        self.drone_path = []
        self.tags = {}  # id -> averaged (forward, right), retained when unseen
        self.counts = {}
        self.has_flight_status = False
        self.armed = False
        self.ground_pose = None
        self.preflight_reference = None
        self.preflight_delta = (0.0, 0.0)

    def reset(self):
        """Explicitly discard the map; never silently move an in-flight origin."""
        self.origin = self.poses[-1][1] if self.poses else None
        self.drone_path.clear()
        self.tags.clear()
        self.counts.clear()
        self.preflight_reference = None
        if self.has_flight_status and not self.armed:
            self.origin = None

    def relative_position(self, x, y):
        x0, y0, heading0 = self.origin
        dx, dy = x - x0, y - y0
        c, s = math.cos(heading0), math.sin(heading0)
        return c * dx + s * dy, -s * dx + c * dy

    def update_pose(self, packet, elapsed_s):
        extended = getattr(packet, "flight_status_version", None) == 1
        ox, oy = (packet.map_offset_x, packet.map_offset_y) if extended else (0, 0)
        pose = (packet.ned_x - ox, packet.ned_y - oy, packet.heading_rad)
        if not all(math.isfinite(v) for v in (*pose, elapsed_s)):
            return
        if extended and not packet.px4_link_ok:
            return
        if self.poses and elapsed_s < self.poses[-1][0]:
            return
        if extended and not self.has_flight_status:
            self.reset()
            self.origin = None
            self.poses.clear()
        self.has_flight_status = extended
        if extended:
            was_armed = self.armed
            self.armed = bool(packet.px4_armed)
            if not self.armed:
                self.ground_pose = pose
                if was_armed:
                    self.preflight_reference = None
                if self.preflight_reference is None:
                    self.preflight_reference = pose
                self.preflight_delta = tuple(pose[i] - self.preflight_reference[i] for i in (0, 1))
                self.poses.append((elapsed_s, pose))
                return
            if not was_armed:
                self.reset()
                self.origin = self.ground_pose or pose
                self.poses.clear()
        if self.origin is None:
            self.origin = pose
        position = self.relative_position(*pose[:2])
        if self.poses and elapsed_s - self.poses[-1][0] > MAX_POSE_AGE_S:
            self.drone_path.append((math.nan, math.nan))
        self.poses.append((elapsed_s, pose))
        self.drone_path.append(position)

    def fresh(self, elapsed_s):
        return bool(self.poses and 0 <= elapsed_s - self.poses[-1][0] <= MAX_POSE_AGE_S)

    def observe(self, tags, frame_s):
        if not self.poses or self.origin is None or (self.has_flight_status and not self.armed):
            return
        pose_s, (x, y, heading) = min(reversed(self.poses), key=lambda p: abs(p[0] - frame_s))
        if abs(pose_s - frame_s) > MAX_POSE_AGE_S:
            return  # keep the existing map, without adding unlocated tags
        drone_fwd, drone_right = self.relative_position(x, y)
        yaw = heading - self.origin[2]
        c, s = math.cos(yaw), math.sin(yaw)
        for tag_id, (fwd, right, _) in tags.items():
            observation = (drone_fwd + c * fwd - s * right,
                           drone_right + s * fwd + c * right)
            count = self.counts.get(tag_id, 0) + 1
            previous = self.tags.get(tag_id, observation)
            self.tags[tag_id] = tuple(old + (new - old) / count
                                      for old, new in zip(previous, observation))
            self.counts[tag_id] = count


def drain_telemetry(sock, esp_ip, plot, started):
    """Accept direct drone telemetry or CommsNode's loopback copy for this IP."""
    for _ in range(2000):
        try:
            data, (source_ip, _) = sock.recvfrom(4096)
        except (BlockingIOError, InterruptedError):
            break
        if source_ip == "127.0.0.1" and len(data) >= 4:
            if socket.inet_ntoa(data[:4]) != esp_ip:
                continue
            data = data[4:]
        elif source_ip != esp_ip:
            continue
        packet = parse_telemetry(data)
        if packet is not None:
            plot.map.update_pose(packet, time.monotonic() - started)


class TagPlot:
    """Live drone-relative tags and a persistent start-relative map, by ID."""

    def __init__(self):
        self.figure, (self.position_ax, self.map_ax) = plt.subplots(
            1, 2, figsize=(13, 6), constrained_layout=True)
        self.figure.suptitle("AprilTag mapping")
        self.position_ax.set(title="Current positions (top view)",
                             xlabel="Right (m)", ylabel="Forward (m)")
        self.position_ax.set_aspect("equal", adjustable="datalim")
        self.position_ax.plot(0, 0, "k^", markersize=9, label="Drone")
        self.position_ax.set_xlim(-1, 1, auto=True)
        self.position_ax.set_ylim(-1, 1, auto=True)
        self.position_ax.margins(x=0.35, y=0.25)
        self.position_ax.legend(loc="upper left")
        self.map_ax.set(title="Persistent map: waiting for telemetry",
                        xlabel="Right from start (m)", ylabel="Forward from start (m)")
        self.map_ax.set_aspect("equal", adjustable="box")
        self.map_ax.set_xlim(-3, 3)
        self.map_ax.set_ylim(-3, 3)
        self.map_ax.set_autoscale_on(False)
        self.map_ax.plot(0, 0, "g*", markersize=12, label="Drone start (0, 0)")
        self.path_line, = self.map_ax.plot([], [], "-", color="0.6", label="Drone path")
        self.drone_marker, = self.map_ax.plot([], [], "k^", markersize=8, label="Drone now")
        self.map_ax.legend(loc="upper left")
        for ax in (self.position_ax, self.map_ax):
            ax.grid(True, alpha=0.3)
        self.map = StartMap()
        self.markers = {}
        self.map_artists = {}
        self.current = {}
        self.annotations = []
        self.last_frame_s = None
        self.next_draw_s = 0.0
        self.quit_requested = False
        self.figure.canvas.mpl_connect("key_press_event", self._on_key)
        plt.show(block=False)

    def _on_key(self, event):
        if event.key in ("q", "escape"):
            self.quit_requested = True
        elif event.key == "r":
            self.map.reset()

    def update(self, tags, elapsed_s, frame_s=None):
        """Update visible tags and accumulate located tags in the fixed map."""
        self.current = {}
        self.last_frame_s = elapsed_s
        for det in tags:
            body = det.t_body
            if (not passes_gate(det) or not det.pose_valid or body is None
                    or det.pose_err is None or not det.pose_err < POSE_ERR_LIMIT
                    or not all(math.isfinite(v) for v in body)):
                continue
            self.current[det.id] = body
        for tag_id in self.current:
            if tag_id not in self.markers:
                self.markers[tag_id], = self.position_ax.plot([], [], "o")
        self.map.observe(self.current, elapsed_s if frame_s is None else frame_s)

    def refresh(self, elapsed_s, force=False):
        """Draw at most 5 Hz, while servicing GUI events even without frames."""
        if force or elapsed_s >= self.next_draw_s:
            self.next_draw_s = elapsed_s + 0.2
            # A lost video link must not leave old positions looking current.
            if self.last_frame_s is not None and elapsed_s - self.last_frame_s > 1:
                self.update([], elapsed_s)
            for annotation in self.annotations:
                annotation.remove()
            self.annotations.clear()
            for tag_id, marker in self.markers.items():
                body = self.current.get(tag_id)
                if body is None:
                    marker.set_data([], [])
                    continue
                fwd, right, down = body
                marker.set_data([right], [fwd])
                self.annotations.append(self.position_ax.annotate(
                    f"Tag {tag_id}: {math.dist((0, 0, 0), body):.2f} m\n"
                    f"down {down:+.2f} m", (right, fwd),
                    xytext=(7, 7), textcoords="offset points",
                    color=marker.get_color()))
            self.position_ax.set_title(
                "Current positions (top view)" if self.current else "No tags visible")
            for tag_id in set(self.map_artists) - self.map.tags.keys():
                for artist in self.map_artists.pop(tag_id):
                    artist.remove()
            for tag_id, (fwd, right) in self.map.tags.items():
                if tag_id not in self.map_artists:
                    color = self.markers[tag_id].get_color()
                    point, = self.map_ax.plot([], [], "o", color=color)
                    label = self.map_ax.annotate("", (0, 0), xytext=(7, 7),
                                                 textcoords="offset points", color=color)
                    self.map_artists[tag_id] = point, label
                point, label = self.map_artists[tag_id]
                point.set_data([right], [fwd])
                label.xy = (right, fwd)
                label.set_text(f"Tag {tag_id}\n{math.hypot(fwd, right):.2f} m from start")
            if self.map.drone_path:
                path_fwd, path_right = zip(*self.map.drone_path)
                self.path_line.set_data(path_right, path_fwd)
                if self.map.fresh(elapsed_s) and (not self.map.has_flight_status or self.map.armed):
                    self.drone_marker.set_data([path_right[-1]], [path_fwd[-1]])
                else:
                    self.drone_marker.set_data([], [])
            else:
                self.path_line.set_data([], [])
                self.drone_marker.set_data([], [])
            title = "Estimated flight path (top view)"
            if not self.map.fresh(elapsed_s):
                title = "Persistent map: waiting for telemetry"
            elif self.map.has_flight_status and not self.map.armed:
                dx, dy = self.map.preflight_delta
                title = f"Disarmed: estimate drift N {dx:+.3f}, E {dy:+.3f} m"
            elif not self.map.has_flight_status:
                title = "Estimated path: flight status unavailable (r to reset)"
            self.map_ax.set_title(title)
            self.position_ax.relim()
            self.position_ax.autoscale_view()
            self.figure.canvas.draw_idle()
        self.figure.canvas.flush_events()

    def save(self, path, elapsed_s):
        self.refresh(elapsed_s, force=True)
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.figure.savefig(path, dpi=150)
        print(f"saved plot {path}")


def to_view(x, y, scale):
    """AprilTag pixel (x, y) -> pixel in the scaled window.

    AprilTag puts (0, 0) at the corner of the top-left pixel, OpenCV at its
    centre, hence the - 0.5.
    """
    return round(x * scale - 0.5), round(y * scale - 0.5)


def draw_tag(view, det, scale):
    """Green outline, plus e.g. "5  fwd 1.02 right +0.10 down 0.50 m" above it."""
    pts = [to_view(x, y, scale) for x, y in det.corners]
    cv2.polylines(view, [np.array(pts, np.int32)], True, GREEN, 2, cv2.LINE_AA)

    label = str(det.id)
    if det.t_body is not None:
        fwd, right, down = det.t_body
        label += f"  fwd {fwd:.2f} right {right:+.2f} down {down:.2f} m"
    # Keep the whole label inside the window.
    (text_w, _), _ = cv2.getTextSize(label, FONT, 0.5, 3)
    x = max(0, min(min(p[0] for p in pts), view.shape[1] - text_w))
    y = max(15, min(p[1] for p in pts) - 6)
    cv2.putText(view, label, (x, y), FONT, 0.5, BLACK, 3, cv2.LINE_AA)  # outline
    cv2.putText(view, label, (x, y), FONT, 0.5, GREEN, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description="Live drone camera with AprilTags.")
    parser.add_argument("--esp-ip", required=True,
                        help="drone IP, 192.168.1.(200 + drone id)")
    parser.add_argument("--fps", type=int, default=0,
                        help="1-15 (default 0 = firmware's 10)")
    parser.add_argument("--quality", type=int, default=0,
                        help="JPEG quality 10-90 (default 0 = firmware's 60)")
    parser.add_argument("--scale", type=float, default=2.0,
                        help="window size multiplier (default 2)")
    parser.add_argument("--telem-port", type=int, default=TELEM_MIRROR_PORT,
                        help="telemetry port (default 5010: local copy from trajectory/mission "
                             "sender; use 5005 for standalone direct telemetry)")
    parser.add_argument("--plot-output", type=Path,
                        default=Path(time.strftime("tag_stream_plot_%Y%m%d_%H%M%S.png")),
                        help="saved plot path (default: timestamped PNG in current directory)")
    parser.add_argument("--map-span", type=float, default=3.0,
                        help="Fixed map half-width in metres (default 3); use toolbar to pan/zoom")
    args = parser.parse_args()
    if not math.isfinite(args.scale) or args.scale <= 0:
        parser.error("--scale must be positive and finite")
    if not math.isfinite(args.map_span) or args.map_span <= 0:
        parser.error("--map-span must be positive and finite")
    if not 1 <= args.telem_port <= 65535:
        parser.error("--telem-port must be 1-65535")
    try:
        start_cmd = build_camera_stream_command(True, args.fps, args.quality)
    except ValueError as exc:
        parser.error(str(exc))
    esp_ip = socket.gethostbyname(args.esp_ip)  # packets carry numeric IPs

    # Frames arrive on one UDP port; start/stop commands go out on another.
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)  # 1 MB
    rx.bind(("0.0.0.0", CAMERA_STREAM_PORT))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    drone = (esp_ip, COMMAND_PORT)
    telemetry = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        bind_ip = "127.0.0.1" if args.telem_port == TELEM_MIRROR_PORT else "0.0.0.0"
        telemetry.bind((bind_ip, args.telem_port))
        telemetry.setblocking(False)
    except OSError as exc:
        telemetry.close()
        rx.close()
        tx.close()
        parser.error(f"cannot listen for telemetry on UDP {args.telem_port}: {exc}")

    detector = Detector()           # compiles the drone's C code on first run
    assembler = FrameAssembler()    # joins UDP pieces back into whole JPEGs
    window = "Drone camera + AprilTags"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    plot = TagPlot()
    plot.map_ax.set_xlim(-args.map_span, args.map_span)
    plot.map_ax.set_ylim(-args.map_span, args.map_span)
    started = time.monotonic()
    print(f"Asking {esp_ip} for video. q or Esc quits, s saves camera and plot.")
    print(f"Plot will be saved to {args.plot_output}")
    print(f"Mapping uses telemetry on UDP {args.telem_port}. TF01 firmware supplies flight status "
          "and map offsets; path starts on arming, referenced to the last ground pose. "
          "Legacy firmware uses the first position. Press r to clear/re-zero the map.")
    if args.telem_port == TELEM_MIRROR_PORT:
        print("Run send_trajectory.py or another CommsNode program alongside this viewer "
              "to supply telemetry for the persistent map.")

    next_keepalive = 0.0
    view = None
    try:
        while True:
            # The drone only streams while it keeps hearing from us.
            if time.monotonic() >= next_keepalive:
                next_keepalive = time.monotonic() + KEEPALIVE_S
                try:
                    tx.sendto(start_cmd, drone)
                except OSError as exc:      # e.g. not on the drone Wi-Fi
                    print(f"cannot reach drone: {exc}")

            select.select([rx, telemetry], [], [], 0.01)  # wait up to 10 ms for data
            drain_telemetry(telemetry, esp_ip, plot, started)
            frame, _, _ = drain_socket(rx, assembler, esp_ip)
            assembler.expire(time.monotonic())  # forget frames missing a piece

            if frame is not None:
                gray = cv2.imdecode(np.frombuffer(frame.jpeg, np.uint8),
                                    cv2.IMREAD_GRAYSCALE)   # (240, 320) uint8
                if gray is not None:
                    tags = [d for d in detector.detect(gray) if passes_gate(d)]
                    # END reports capture-to-send age; first_rx approximates arrival.
                    frame_s = frame.first_rx - started - frame.age_ms / 1000.0
                    plot.update(tags, time.monotonic() - started, frame_s=frame_s)
                    h, w = gray.shape
                    size = (round(w * args.scale), round(h * args.scale))
                    view = cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
                                      size, interpolation=cv2.INTER_NEAREST)
                    for det in tags:
                        draw_tag(view, det, args.scale)
                    cv2.imshow(window, view)

            plot.refresh(time.monotonic() - started)
            key = cv2.waitKey(1) & 0xFF
            if (key in (ord("q"), 27) or plot.quit_requested
                    or not plt.fignum_exists(plot.figure.number)):
                break
            if key == ord("s"):
                if view is not None:
                    path = time.strftime("tag_stream_%Y%m%d_%H%M%S.png")
                    cv2.imwrite(path, view)
                    print(f"saved {path}")
                plot.save(args.plot_output, time.monotonic() - started)
            if key == ord("r"):
                plot.map.reset()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            tx.sendto(build_camera_stream_command(False), drone)  # stop stream
        except OSError:
            pass
        try:
            plot.save(args.plot_output, time.monotonic() - started)
        finally:
            detector.close()
            rx.close()
            tx.close()
            telemetry.close()
            cv2.destroyAllWindows()
            plt.close(plot.figure)


if __name__ == "__main__":
    main()
