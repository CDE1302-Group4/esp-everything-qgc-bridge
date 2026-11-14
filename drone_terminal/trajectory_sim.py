#!/usr/bin/env python3
"""Real-time 2D simulation of a trajectory CSV, as the firmware would fly it.

    python -m drone_terminal.trajectory_sim trajectory/rect_spiral_2.5x1.8.csv
    python -m drone_terminal.trajectory_sim            # pick from trajectory/

The CSV goes through the same loader and sample interval the terminal
uploads, so what you see is what the ESP32 would play:

- ``t,x,y,z`` (legacy): the firmware holds the heading the drone had when
  playback started, so the triangle translates without turning. Press H to
  show the travel direction instead (a preview aid, not what the drone does).
- ``t,x,y,z,yaw_deg``: the drone first rotates in place to the first yaw
  (simulated at PX4's default 45 deg/s auto-yaw rate), then the playback
  clock starts and yaw follows the CSV through the shortest turn.

The plot is a true top-down view: North (x) to the right, East (y) down, so
a clockwise (positive NED) yaw change looks clockwise on screen.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parent.parent
for dependency in (str(PROJECT_ROOT), str(PROJECT_ROOT / "laptop")):
    if dependency not in sys.path:
        sys.path.insert(0, dependency)

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation  # noqa: E402
from matplotlib.patches import Circle, Polygon, Rectangle  # noqa: E402
import numpy as np  # noqa: E402

from drone_app.flight import load_trajectory  # noqa: E402
from drone_terminal.app import TRAJECTORY_DIR, fit_csv_dt  # noqa: E402

ALIGN_RATE_RAD_S = math.radians(45.0)  # PX4 MPC_YAWRAUTO_MAX default

# Studio 1 flying area, relative to the launch-zone centre (metres, NED).
FLIGHT_X = (-0.45, 4.95)
FLIGHT_Y = (-2.25, 2.25)
LAUNCH_XY = (-0.45, -0.45, 0.90, 0.90)
SEARCH_XY = (0.90, -1.35, 3.60, 3.15)
TAG_CANDIDATES = tuple((x, y) for y in (1.35, -0.90) for x in (0.99, 2.25, 3.51))


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def travel_heading(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Direction of travel per sample, held through stops (unwrapped)."""
    dx, dy = np.gradient(x), np.gradient(y)
    moving = np.hypot(dx, dy) > 1e-6
    if not np.any(moving):
        return np.zeros_like(x)
    idx = np.flatnonzero(moving)
    return np.interp(np.arange(len(x)), idx, np.unwrap(np.arctan2(dy[idx], dx[idx])))


class Plan:
    """The uploaded samples plus the timeline the firmware would follow."""

    def __init__(self, path: Path, start_heading: float):
        self.dt = fit_csv_dt(path)
        pts = load_trajectory(str(path), self.dt)  # validates like the uploader
        self.x, self.y = pts[:, 0], pts[:, 1]
        self.has_yaw = pts.shape[1] == 4
        self.start_heading = start_heading
        if self.has_yaw:
            self.yaw = np.unwrap(pts[:, 3])
            turn = wrap_pi(float(self.yaw[0]) - start_heading)
            self.align_s = abs(turn) / ALIGN_RATE_RAD_S
            # Unwrapped yaw continues from the start heading via the short turn.
            self.yaw += (start_heading + turn) - self.yaw[0]
        else:
            self.yaw = np.full(len(pts), start_heading)
            self.align_s = 0.0
        self.travel = travel_heading(self.x, self.y)
        self.play_s = (len(pts) - 1) * self.dt
        self.duration = self.align_s + self.play_s

    def state(self, t: float, face_travel: bool):
        """(x, y, yaw, phase) at simulation time t."""
        if t < self.align_s:
            frac = t / self.align_s
            yaw = self.start_heading + frac * (self.yaw[0] - self.start_heading)
            return float(self.x[0]), float(self.y[0]), yaw, "ALIGNING YAW"
        k = min((t - self.align_s) / self.dt, len(self.x) - 1)
        i = min(int(k), len(self.x) - 2)
        a = k - i
        lerp = lambda arr: float(arr[i] + a * (arr[i + 1] - arr[i]))  # noqa: E731
        yaw_arr = self.travel if (face_travel and not self.has_yaw) else self.yaw
        phase = "COMPLETE" if t >= self.duration else "PLAYING"
        return lerp(self.x), lerp(self.y), lerp(yaw_arr), phase

    def index_at(self, t: float) -> int:
        return int(np.clip((t - self.align_s) / self.dt, -1, len(self.x) - 1)) + 1


def triangle(x: float, y: float, yaw: float, size: float) -> np.ndarray:
    fwd = np.array((math.cos(yaw), math.sin(yaw)))
    side = np.array((-fwd[1], fwd[0]))
    pos = np.array((x, y))
    rear = pos - 0.6 * size * fwd
    return np.vstack((pos + size * fwd, rear + 0.55 * size * side, rear - 0.55 * size * side))


class Simulation:
    def __init__(self, path: Path, plan: Plan, speed: float, loop: bool):
        self.path, self.plan = path, plan
        self.speed, self.loop = speed, loop
        self.t = 0.0
        self.paused = False
        self.face_travel = False
        self.last_clock = time.monotonic()

        self.fig, self.ax = plt.subplots(figsize=(11, 8))
        self.fig.canvas.manager.set_window_title(f"Trajectory simulation - {path.name}")
        self._draw_arena()
        self.ax.plot(plan.x, plan.y, color="white", lw=1.2, alpha=0.7, label="Planned route")
        self.trail, = self.ax.plot([], [], color="#ff8c00", lw=2.2, label="Flown so far")
        span = max(np.ptp(plan.x), np.ptp(plan.y), 1.0)
        self.size = float(np.clip(span * 0.04, 0.12, 0.3))
        self.drone = Polygon(triangle(plan.x[0], plan.y[0], plan.yaw[0], self.size),
                             closed=True, fc="#1565c0", ec="white", lw=1.5, zorder=10)
        self.ax.add_patch(self.drone)
        self.info = self.ax.text(
            0.01, 0.99, "", transform=self.ax.transAxes, va="top", ha="left",
            fontsize=10, family="monospace", zorder=20,
            bbox={"boxstyle": "round,pad=0.35", "fc": "white", "alpha": 0.88})
        keys = "Space pause   R restart   +/- speed   L loop   Q close"
        if not plan.has_yaw:
            keys = "H travel-facing view   " + keys
        self.ax.text(0.99, 0.01, keys, transform=self.ax.transAxes, va="bottom",
                     ha="right", fontsize=9,
                     bbox={"fc": "white", "alpha": 0.75, "ec": "none"})
        self.ax.legend(loc="lower left")
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self.anim = FuncAnimation(self.fig, self._update, interval=33,
                                  blit=False, cache_frame_data=False)

    def _draw_arena(self) -> None:
        ax, p = self.ax, self.plan
        ax.add_patch(Rectangle((FLIGHT_X[0], FLIGHT_Y[0]), np.diff(FLIGHT_X)[0],
                               np.diff(FLIGHT_Y)[0], fc="#aaa99c", ec="#3f8fc4",
                               lw=2.0, zorder=0, label="Flying area"))
        ax.add_patch(Rectangle(LAUNCH_XY[:2], *LAUNCH_XY[2:], fc="#61a844", ec="black",
                               lw=1.5, zorder=1))
        ax.add_patch(Rectangle(SEARCH_XY[:2], *SEARCH_XY[2:], fill=False, ec="#54a83c",
                               lw=2.0, zorder=2))
        for n, (tx, ty) in enumerate(TAG_CANDIDATES, 1):
            ax.add_patch(Circle((tx, ty), 0.045, fill=False, ec="#303030", zorder=3))
            ax.annotate(str(n), (tx, ty), xytext=(5, 5), textcoords="offset points",
                        fontsize=8, color="#303030")
        ax.plot(0, 0, "o", color="#ff8c42", ms=9, zorder=5, label="Start")
        pad = 0.3
        ax.set_xlim(min(FLIGHT_X[0], p.x.min()) - pad, max(FLIGHT_X[1], p.x.max()) + pad)
        ax.set_ylim(min(FLIGHT_Y[0], p.y.min()) - pad, max(FLIGHT_Y[1], p.y.max()) + pad)
        ax.invert_yaxis()  # East down with North right = true top-down view
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x / North (m)  →")
        ax.set_ylabel("y / East (m)  ↓")
        fmt = "explicit yaw" if p.has_yaw else "fixed heading (legacy)"
        ax.set_title(f"{self.path.name} — {fmt}, dt {p.dt * 1000:.0f} ms, "
                     f"{len(p.x)} pts")
        ax.grid(True, color="white", lw=0.5, alpha=0.3)

    def _on_key(self, event) -> None:
        key = (event.key or "").lower()
        if key == " ":
            self.paused = not self.paused
        elif key == "r":
            self.t, self.paused = 0.0, False
        elif key in ("+", "="):
            self.speed = min(32.0, self.speed * 1.5)
        elif key in ("-", "_"):
            self.speed = max(0.1, self.speed / 1.5)
        elif key == "l":
            self.loop = not self.loop
        elif key == "h" and not self.plan.has_yaw:
            self.face_travel = not self.face_travel
        elif key in ("q", "escape"):
            plt.close(self.fig)
        self.last_clock = time.monotonic()

    def _update(self, _frame):
        now = time.monotonic()
        if not self.paused:
            self.t += (now - self.last_clock) * self.speed
            if self.t >= self.plan.duration:
                if self.loop:
                    self.t = 0.0
                else:
                    self.t = self.plan.duration
        self.last_clock = now

        x, y, yaw, phase = self.plan.state(self.t, self.face_travel)
        self.drone.set_xy(triangle(x, y, yaw, self.size))
        self.drone.set_facecolor("#c62828" if phase == "ALIGNING YAW" else "#1565c0")
        i = self.plan.index_at(self.t)
        self.trail.set_data(np.append(self.plan.x[:i], x), np.append(self.plan.y[:i], y))

        if self.paused and phase != "COMPLETE":
            phase = "PAUSED"
        if self.plan.has_yaw:
            yaw_note = "CSV yaw"
        else:
            yaw_note = "travel view (drone won't turn)" if self.face_travel else "held start heading"
        self.info.set_text(
            f"t   {self.t:7.2f} / {self.plan.duration:.2f} s   {phase}\n"
            f"x   {x:+6.2f} m    y {y:+6.2f} m\n"
            f"yaw {math.degrees(wrap_pi(yaw)):+6.1f} deg  ({yaw_note})\n"
            f"playback {self.speed:.2f}x   loop {'on' if self.loop else 'off'}")
        return self.drone, self.trail, self.info

    def show(self) -> None:
        plt.tight_layout()
        plt.show()


def choose_trajectory() -> Path:
    files = sorted(TRAJECTORY_DIR.glob("*.csv"), key=lambda p: p.name.lower())
    if not files:
        raise SystemExit(f"No trajectory CSV files in {TRAJECTORY_DIR}")
    for n, path in enumerate(files, 1):
        print(f"  {n}. {path.name}")
    while True:
        try:
            choice = int(input("Choose trajectory: ").strip())
        except ValueError:
            continue
        except (EOFError, KeyboardInterrupt):
            raise SystemExit(130) from None
        if 1 <= choice <= len(files):
            return files[choice - 1]


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("trajectory", nargs="?", type=Path,
                        help="t,x,y,z or t,x,y,z,yaw_deg CSV; prompts if omitted")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="playback multiplier (default 1.0 = real time)")
    parser.add_argument("--start-heading", type=float, default=0.0,
                        help="drone heading at playback start, deg (0 = North)")
    parser.add_argument("--loop", action="store_true", help="restart at the end")
    args = parser.parse_args(argv)
    if not (math.isfinite(args.speed) and 0.1 <= args.speed <= 32):
        parser.error("--speed must be between 0.1 and 32")
    path = (args.trajectory or choose_trajectory()).resolve()
    if not path.is_file():
        parser.error(f"trajectory does not exist: {path}")
    try:
        plan = Plan(path, math.radians(args.start_heading))
    except ValueError as exc:
        # Same rejection the upload would give, so the drone could not fly it either.
        parser.error(str(exc))
    Simulation(path, plan, args.speed, args.loop).show()


if __name__ == "__main__":
    main()
