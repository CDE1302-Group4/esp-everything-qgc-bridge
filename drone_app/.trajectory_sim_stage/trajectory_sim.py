#!/usr/bin/env python3
"""Real-time 2D preview for legacy and explicit-yaw trajectory CSV files."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Polygon, Rectangle
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parent.parent
TRAJECTORY_DIR = PROJECT_ROOT / "trajectory"

FLIGHT_X = (-0.45, 4.95)
FLIGHT_Y = (-2.25, 2.25)
LAUNCH_XY = (-0.45, -0.45, 0.90, 0.90)
SEARCH_XY = (0.90, -1.35, 3.60, 3.15)
TAG_CANDIDATES = tuple(
    (x, y) for y in (1.35, -0.90) for x in (0.99, 2.25, 3.51)
)


def choose_trajectory() -> Path:
    files = sorted(TRAJECTORY_DIR.glob("*.csv"), key=lambda path: path.name.lower())
    if not files:
        raise SystemExit(f"No trajectory CSV files found in {TRAJECTORY_DIR}")
    print(f"Trajectories in {TRAJECTORY_DIR}:")
    for index, path in enumerate(files, 1):
        print(f"  {index}. {path.name}")
    while True:
        try:
            choice = int(input("Choose trajectory: ").strip())
        except (EOFError, KeyboardInterrupt):
            raise SystemExit(130) from None
        except ValueError:
            choice = 0
        if 1 <= choice <= len(files):
            return files[choice - 1]
        print(f"Enter a number from 1 to {len(files)}.")


def _path_heading(t: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """Infer a smooth visual heading for a legacy CSV with no yaw column."""
    vx = np.gradient(x, t)
    vy = np.gradient(y, t)
    moving = np.hypot(vx, vy) > 1e-5
    if not np.any(moving):
        return np.zeros_like(t)
    sample_indices = np.flatnonzero(moving)
    samples = np.unwrap(np.arctan2(vy[moving], vx[moving]))
    return np.interp(np.arange(len(t)), sample_indices, samples)


def load_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool]:
    rows: list[list[float]] = []
    columns: int | None = None
    with path.open(newline="", encoding="utf-8-sig") as source:
        for line_number, line in enumerate(source, 1):
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            cells = next(csv.reader([line]))
            header = [cell.strip().lower() for cell in cells]
            if not rows and header in (["t", "x", "y", "z"],
                                       ["t", "x", "y", "z", "yaw_deg"]):
                columns = len(header)
                continue
            try:
                values = [float(cell) for cell in cells]
            except ValueError:
                raise ValueError(f"{path}:{line_number}: values must be numeric") from None
            if len(values) not in (4, 5) or not all(math.isfinite(v) for v in values):
                raise ValueError(
                    f"{path}:{line_number}: expected finite t,x,y,z values, "
                    "optionally followed by yaw_deg"
                )
            if columns is None:
                columns = len(values)
            if len(values) != columns:
                raise ValueError(f"{path}:{line_number}: inconsistent column count")
            rows.append(values)

    if len(rows) < 2:
        raise ValueError(f"{path}: at least two waypoints are required")
    data = np.asarray(rows, dtype=float)
    t = data[:, 0] - data[0, 0]
    if np.any(np.diff(t) <= 0):
        raise ValueError(f"{path}: time must be strictly increasing")
    x, y = data[:, 1], data[:, 2]
    has_yaw = data.shape[1] == 5
    yaw = (np.unwrap(np.deg2rad(data[:, 4])) if has_yaw
           else _path_heading(t, x, y))
    return t, x, y, yaw, has_yaw


def triangle_points(x: float, y: float, yaw: float) -> np.ndarray:
    forward = np.array((math.cos(yaw), math.sin(yaw)))
    right = np.array((forward[1], -forward[0]))
    position = np.array((x, y))
    tip = position + 0.22 * forward
    rear = position - 0.13 * forward
    return np.vstack((tip, rear + 0.12 * right, rear - 0.12 * right))


class TrajectorySimulation:
    def __init__(self, path: Path, speed: float = 1.0, loop: bool = False):
        self.path = path
        self.t, self.x, self.y, self.yaw, self.has_yaw = load_csv(path)
        self.duration = float(self.t[-1])
        self.playback_speed = speed
        self.loop = loop
        self.current_time = 0.0
        self.last_clock = time.monotonic()
        self.paused = False

        self.figure, self.ax = plt.subplots(figsize=(11, 8))
        self.figure.canvas.manager.set_window_title(f"Trajectory simulation - {path.name}")
        self._draw_arena()
        self.route_line, = self.ax.plot(
            self.x, self.y, color="white", linewidth=1.2, alpha=0.65,
            label="Planned route",
        )
        self.trail_line, = self.ax.plot(
            [], [], color="#ff8c00", linewidth=2.2, label="Simulated flight",
        )
        self.drone = Polygon(
            triangle_points(self.x[0], self.y[0], self.yaw[0]),
            closed=True, facecolor="#1565c0", edgecolor="white", linewidth=1.5,
            zorder=10,
        )
        self.ax.add_patch(self.drone)
        self.info = self.ax.text(
            0.01, 0.99, "", transform=self.ax.transAxes, va="top", ha="left",
            fontsize=10, color="black",
            bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "alpha": 0.88},
            zorder=20,
        )
        self.help_text = self.ax.text(
            0.99, 0.01,
            "Space: pause/resume   R: restart   +/-: speed   L: loop   Q/Esc: close",
            transform=self.ax.transAxes, va="bottom", ha="right", fontsize=9,
            color="black", bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "none"},
        )
        self.ax.legend(loc="lower left")
        self.figure.canvas.mpl_connect("key_press_event", self._on_key)
        self.animation = FuncAnimation(
            self.figure, self._update, interval=33, blit=False, cache_frame_data=False,
        )

    def _draw_arena(self) -> None:
        self.ax.add_patch(Rectangle(
            (FLIGHT_X[0], FLIGHT_Y[0]), FLIGHT_X[1] - FLIGHT_X[0],
            FLIGHT_Y[1] - FLIGHT_Y[0], facecolor="#aaa99c", edgecolor="#3f8fc4",
            linewidth=2.0, zorder=0,
        ))
        self.ax.add_patch(Rectangle(
            LAUNCH_XY[:2], LAUNCH_XY[2], LAUNCH_XY[3], facecolor="#61a844",
            edgecolor="black", linewidth=1.8, zorder=1,
        ))
        self.ax.add_patch(Rectangle(
            SEARCH_XY[:2], SEARCH_XY[2], SEARCH_XY[3], fill=False,
            edgecolor="#54a83c", linewidth=2.0, zorder=2,
        ))
        self.ax.axhline(0, color="black", linestyle="--", linewidth=1.0, alpha=0.65)
        self.ax.plot(0, 0, "o", color="#ff8c42", markersize=9, zorder=5,
                     label="Launch / return")
        for number, (tag_x, tag_y) in enumerate(TAG_CANDIDATES, 1):
            self.ax.add_patch(Circle(
                (tag_x, tag_y), 0.045, fill=False, edgecolor="#303030",
                linewidth=1.0, zorder=3,
            ))
            self.ax.annotate(
                str(number), (tag_x, tag_y), xytext=(5, 5), textcoords="offset points",
                fontsize=8, color="#303030",
            )
        self.ax.set_xlim(FLIGHT_X[0] - 0.15, FLIGHT_X[1] + 0.15)
        self.ax.set_ylim(FLIGHT_Y[0] - 0.15, FLIGHT_Y[1] + 0.15)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("Length direction x / North (m)")
        self.ax.set_ylabel("Width direction y / East (m)")
        self.ax.set_title("Flying-area trajectory preview")
        self.ax.grid(True, color="white", linewidth=0.5, alpha=0.25)

    def _on_key(self, event) -> None:
        key = (event.key or "").lower()
        if key == " ":
            self.paused = not self.paused
            self.last_clock = time.monotonic()
        elif key == "r":
            self.current_time = 0.0
            self.paused = False
            self.last_clock = time.monotonic()
        elif key in ("+", "="):
            self.playback_speed = min(16.0, self.playback_speed * 1.25)
            self.last_clock = time.monotonic()
        elif key in ("-", "_"):
            self.playback_speed = max(0.1, self.playback_speed / 1.25)
            self.last_clock = time.monotonic()
        elif key == "l":
            self.loop = not self.loop
        elif key in ("q", "escape"):
            plt.close(self.figure)

    def _update(self, _frame):
        now = time.monotonic()
        if not self.paused:
            self.current_time += (now - self.last_clock) * self.playback_speed
            if self.current_time >= self.duration:
                if self.loop:
                    self.current_time %= self.duration
                else:
                    self.current_time = self.duration
                    self.paused = True
        self.last_clock = now

        x = float(np.interp(self.current_time, self.t, self.x))
        y = float(np.interp(self.current_time, self.t, self.y))
        yaw = float(np.interp(self.current_time, self.t, self.yaw))
        self.drone.set_xy(triangle_points(x, y, yaw))
        index = int(np.searchsorted(self.t, self.current_time, side="right"))
        trail_x = np.append(self.x[:index], x)
        trail_y = np.append(self.y[:index], y)
        self.trail_line.set_data(trail_x, trail_y)

        yaw_deg = (math.degrees(yaw) + 180.0) % 360.0 - 180.0
        yaw_source = "CSV yaw" if self.has_yaw else "path-inferred yaw"
        state = "PAUSED" if self.paused and self.current_time < self.duration else "PLAYING"
        if self.current_time >= self.duration:
            state = "COMPLETE"
        self.info.set_text(
            f"{self.path.name}\n"
            f"t = {self.current_time:6.2f} / {self.duration:.2f} s   {state}\n"
            f"x = {x:+.2f} m   y = {y:+.2f} m   yaw = {yaw_deg:+.1f} deg\n"
            f"{yaw_source}   playback {self.playback_speed:.2f}x   "
            f"loop {'on' if self.loop else 'off'}"
        )
        return self.drone, self.trail_line, self.info

    def show(self) -> None:
        plt.tight_layout()
        plt.show()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description="Simulate a trajectory CSV on the 5.4 x 4.5 m flying-area map.",
    )
    parser.add_argument("trajectory", nargs="?", type=Path,
                        help="t,x,y,z or t,x,y,z,yaw_deg CSV; prompts if omitted")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="playback multiplier (default: real-time 1.0)")
    parser.add_argument("--loop", action="store_true", help="restart automatically at the end")
    args = parser.parse_args(argv)
    if not math.isfinite(args.speed) or not 0.1 <= args.speed <= 16.0:
        parser.error("--speed must be between 0.1 and 16")
    path = (args.trajectory or choose_trajectory()).resolve()
    if not path.is_file():
        parser.error(f"trajectory does not exist: {path}")
    try:
        simulation = TrajectorySimulation(path, args.speed, args.loop)
    except ValueError as exc:
        parser.error(str(exc))
    simulation.show()


if __name__ == "__main__":
    main()
