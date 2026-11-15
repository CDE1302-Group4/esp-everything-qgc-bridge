#!/usr/bin/env python3
"""Real-time 2D simulation of a trajectory CSV, flown by a simulated drone.

    python -m drone_terminal.trajectory_sim trajectory/rect_spiral_2.5x1.8.csv
    python -m drone_terminal.trajectory_sim            # pick from trajectory/
    python -m drone_terminal.trajectory_sim --ideal    # no disturbances

The CSV goes through the same loader and sample interval the terminal
uploads, and the ESP32 side is replayed as the firmware runs it:

- After take-off the drone hovers for a few seconds to settle, then START.
  Positions are offsets from where PX4 *thinks* the drone is at START.
- ``t,x,y,z`` (legacy): the setpoint heading is the heading at START.
- ``t,x,y,z,yaw_deg``: the drone holds position and turns to the first yaw;
  the playback clock starts once the yaw error is under 0.1 rad.
- Setpoints (position + velocity feed-forward + yaw) are refreshed at the
  firmware's 10 Hz nav tick and reach PX4 after a short link delay.
- After the last point the drone holds it until it has stayed within
  0.15 m for 1 s, the flight runner's landing condition.

The drone is not a perfect follower. A PX4-style cascaded controller
(position P -> velocity PID -> tilt-limited acceleration with attitude lag)
flies a point mass that is pushed around by:

- turbulence (random gusts, stronger within 0.5 m of a wall: prop wash
  recirculating off the wall), a slow steady draft, and motor vibration;
- aerodynamic drag;
- an imperfect position estimate (optical-flow drift + noise), so PX4
  controls what it believes, not where the drone really is;
- yaw disturbances and a 45 deg/s yaw-rate limit.

Walls of the flying area are solid: touching one is counted as a contact and
the drone bounces off (the firmware's ToF collision avoidance is disabled).
The constants are representative of a small indoor quad, not identified from
our drone, so treat the result as "what kind of errors to expect".

The plot is a true top-down view: North (x) to the right, East (y) down.
Keys: Space pause, R restart, N new random flight, D disturbances on/off,
+/- speed, L loop, Q close.
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

from drone_terminal.flight import load_trajectory  # noqa: E402
from drone_terminal.app import TRAJECTORY_DIR, fit_csv_dt  # noqa: E402

# Studio 1 flying area, relative to the launch-zone centre (metres, NED).
FLIGHT_X = (-0.45, 4.95)
FLIGHT_Y = (-2.25, 2.25)
LAUNCH_XY = (-0.45, -0.45, 0.90, 0.90)
SEARCH_XY = (0.90, -1.35, 3.60, 3.15)
TAG_CANDIDATES = tuple((x, y) for y in (1.35, -0.90) for x in (0.99, 2.25, 3.51))

# --- Firmware / flight-runner timing ---------------------------------------
NAV_DT = 0.10              # nav_task tick: setpoint refresh (10 Hz)
LINK_DELAY = 0.04          # ESP32 -> PX4 UART + PX4 scheduling
YAW_TOL = 0.1              # NAV_YAW_TOL_RAD: alignment done
SETTLE_BEFORE_S = 3.0      # hover after take-off before START is sent
END_TOL_M, END_HOLD_S = 0.15, 1.0   # flight runner: endpoint before landing
END_TIMEOUT_S = 10.0

# --- PX4-like controller (defaults in brackets are PX4 parameter names) -----
PHYS_DT = 0.01
KP_POS = 0.95              # MPC_XY_P
KP_VEL, KI_VEL = 1.8, 0.4  # MPC_XY_VEL_P_ACC, MPC_XY_VEL_I_ACC
VEL_MAX = 2.0              # m/s (offboard feed-forward + correction)
TILT_MAX = math.radians(35.0)
TAU_ATT = 0.12             # s, attitude loop + motor lag
KP_YAW = 2.8               # MC_YAW_P
YAW_RATE_MAX = math.radians(45.0)   # MPC_YAWRAUTO_MAX
TAU_YAW = 0.15
G = 9.81

# --- Disturbances (scaled by --wind; 0 = ideal) ----------------------------
GUST_SIGMA, GUST_TAU = 0.18, 1.5    # m/s^2, s (turbulence)
WALL_ZONE_M, WALL_GAIN = 0.5, 1.5   # gusts up to (1 + gain)x near walls
DRAFT_ACC = 0.05                    # m/s^2 steady draft (aircon, doors)
VIBRATION = 0.4                     # m/s^2 white motor/prop vibration
DRAG = 0.35                         # 1/m, quadratic drag
DRIFT = 0.005                       # m/sqrt(s), flow position drift
EST_NOISE, EST_VEL_NOISE = 0.01, 0.03
YAW_NOISE = math.radians(8.0)       # rad/s/sqrt(s) yaw disturbance
TAKEOFF_POS_ERR, TAKEOFF_VEL = 0.07, 0.15   # leftover take-off motion

DRONE_RADIUS = 0.15
WALL_RESTITUTION = 0.3

PHASES = ("STABILIZING", "ALIGNING YAW", "PLAYING", "SETTLING", "COMPLETE")


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
    """The uploaded samples, exactly as the terminal would send them."""

    def __init__(self, path: Path, start_heading: float):
        self.dt = fit_csv_dt(path)
        pts = load_trajectory(str(path), self.dt)  # validates like the uploader
        self.x, self.y = pts[:, 0], pts[:, 1]
        self.has_yaw = pts.shape[1] == 4
        self.start_heading = start_heading
        self.yaw = np.unwrap(pts[:, 3]) if self.has_yaw else None
        self.travel = travel_heading(self.x, self.y)
        self.play_s = (len(pts) - 1) * self.dt

    def sample(self, k: float):
        """Interpolated (x, y, vx, vy, yaw, yaw_rate) at fractional index k."""
        last = len(self.x) - 1
        if k >= last:
            yaw = float(self.yaw[last]) if self.has_yaw else None
            return float(self.x[last]), float(self.y[last]), 0.0, 0.0, yaw, 0.0
        i = int(k)
        a = k - i
        x = self.x[i] + a * (self.x[i + 1] - self.x[i])
        y = self.y[i] + a * (self.y[i + 1] - self.y[i])
        vx = (self.x[i + 1] - self.x[i]) / self.dt
        vy = (self.y[i + 1] - self.y[i]) / self.dt
        if not self.has_yaw:
            return float(x), float(y), float(vx), float(vy), None, 0.0
        yaw = self.yaw[i] + a * (self.yaw[i + 1] - self.yaw[i])
        rate = (self.yaw[i + 1] - self.yaw[i]) / self.dt
        return float(x), float(y), float(vx), float(vy), float(yaw), float(rate)


class Flight:
    """One simulated flight, precomputed at PHYS_DT."""

    def __init__(self, plan: Plan, seed: int, wind: float):
        self.seed, self.wind = seed, wind
        rng = np.random.default_rng(seed)
        s = wind
        sq = math.sqrt(PHYS_DT)

        pos = rng.normal(0.0, TAKEOFF_POS_ERR * s, 2)
        vel = rng.normal(0.0, TAKEOFF_VEL * s, 2)
        acc = np.zeros(2)
        yaw = plan.start_heading + rng.normal(0.0, math.radians(3.0) * s)
        yaw_rate = 0.0
        vel_int = np.zeros(2)
        gust = np.zeros(2)
        bias = np.zeros(2)
        noise_lp = np.zeros(2)
        vnoise_lp = np.zeros(2)
        angle = rng.uniform(0, 2 * math.pi)
        draft = DRAFT_ACC * s * np.array((math.cos(angle), math.sin(angle)))

        phase = 0
        org = np.zeros(2)          # firmware s_traj_org (estimated position)
        held_yaw = plan.start_heading
        t0 = 0.0
        settle_start = settle_ok = None
        cmd = (np.zeros(2), np.zeros(2), plan.start_heading, 0.0)   # active at PX4
        pending = []               # (time active, command)
        plan_xy = np.zeros(2)

        rec = {k: [] for k in ("t", "x", "y", "yaw", "ex", "ey", "sx", "sy",
                               "px", "py", "ax", "ay", "phase", "wall", "hit")}
        self.contacts = 0
        last_touch = -1e9          # scraping along a wall counts as one contact
        t = 0.0
        step = 0
        nav_every = round(NAV_DT / PHYS_DT)
        done_at = None
        t_cap = SETTLE_BEFORE_S + 30.0 + plan.play_s + END_TIMEOUT_S + 2.0

        while True:
            # --- what PX4 believes (EKF with optical flow) ---------------------
            bias += rng.normal(0.0, DRIFT * s * sq, 2)
            noise_lp += (rng.normal(0.0, EST_NOISE * s, 2) - noise_lp) * 0.2
            vnoise_lp += (rng.normal(0.0, EST_VEL_NOISE * s, 2) - vnoise_lp) * 0.2
            est = pos + bias + noise_lp
            est_vel = vel + vnoise_lp
            est_yaw = yaw + rng.normal(0.0, math.radians(0.5) * s)

            # --- ESP32 nav tick: firmware state machine + new setpoint ---------
            if step % nav_every == 0:
                if phase == 0 and t >= SETTLE_BEFORE_S:
                    org = est.copy()
                    held_yaw = est_yaw
                    if plan.has_yaw:
                        held_yaw += wrap_pi(plan.yaw[0] - est_yaw)   # short turn
                        phase = 1
                    else:
                        phase, t0 = 2, t
                elif phase == 1 and abs(wrap_pi(plan.yaw[0] - est_yaw)) <= YAW_TOL:
                    phase, t0 = 2, t
                if phase == 0:
                    sp = (np.zeros(2), np.zeros(2), plan.start_heading, 0.0)
                    plan_xy = np.zeros(2)
                elif phase == 1:
                    x0, y0 = plan.x[0], plan.y[0]
                    sp = (org + (x0, y0), np.zeros(2), held_yaw, 0.0)
                    plan_xy = np.array((x0, y0))
                else:
                    k = (t - t0) / plan.dt
                    x, y, vx, vy, pyaw, prate = plan.sample(k)
                    if plan.has_yaw:
                        pyaw += held_yaw - plan.yaw[0]
                    else:
                        pyaw, prate = held_yaw, 0.0
                    sp = (org + (x, y), np.array((vx, vy)), pyaw, prate)
                    plan_xy = np.array((x, y))
                    if phase == 2 and k >= len(plan.x) - 1:
                        phase, settle_start = 3, t
                    if phase == 3:
                        near = np.linalg.norm(est - sp[0]) < END_TOL_M
                        settle_ok = (settle_ok or t) if near else None
                        if ((settle_ok is not None and t - settle_ok >= END_HOLD_S)
                                or t - settle_start > END_TIMEOUT_S):
                            phase = 4
                pending.append((t + LINK_DELAY, sp))
            while pending and pending[0][0] <= t:
                cmd = pending.pop(0)[1]

            # --- PX4 position + velocity control ---------------------------------
            sp_pos, sp_vel, sp_yaw, sp_rate = cmd
            vel_cmd = sp_vel + KP_POS * (sp_pos - est)
            speed = np.linalg.norm(vel_cmd)
            if speed > VEL_MAX:
                vel_cmd *= VEL_MAX / speed
            err_v = vel_cmd - est_vel
            vel_int = np.clip(vel_int + err_v * PHYS_DT, -1.0, 1.0)
            acc_cmd = KP_VEL * err_v + KI_VEL * vel_int
            a_max = G * math.tan(TILT_MAX)
            norm = np.linalg.norm(acc_cmd)
            if norm > a_max:
                acc_cmd *= a_max / norm
            acc += (acc_cmd - acc) * PHYS_DT / TAU_ATT        # tilt takes time

            # --- environment ----------------------------------------------------
            gust += (-gust * PHYS_DT / GUST_TAU
                     + GUST_SIGMA * s * math.sqrt(2 * PHYS_DT / GUST_TAU)
                     * rng.normal(0.0, 1.0, 2))
            wall_d = min(pos[0] - FLIGHT_X[0], FLIGHT_X[1] - pos[0],
                         pos[1] - FLIGHT_Y[0], FLIGHT_Y[1] - pos[1]) - DRONE_RADIUS
            wall_f = 1.0 + WALL_GAIN * max(0.0, 1.0 - wall_d / WALL_ZONE_M)
            disturb = draft + gust * wall_f + rng.normal(0.0, VIBRATION * s, 2)
            drag = -DRAG * np.linalg.norm(vel) * vel
            vel += (acc + disturb + drag) * PHYS_DT
            pos += vel * PHYS_DT

            # --- solid walls ------------------------------------------------------
            touching = False
            for axis, (lo, hi) in enumerate((FLIGHT_X, FLIGHT_Y)):
                if pos[axis] < lo + DRONE_RADIUS:
                    pos[axis] = lo + DRONE_RADIUS
                    vel[axis] = abs(vel[axis]) * WALL_RESTITUTION
                    touching = True
                elif pos[axis] > hi - DRONE_RADIUS:
                    pos[axis] = hi - DRONE_RADIUS
                    vel[axis] = -abs(vel[axis]) * WALL_RESTITUTION
                    touching = True
            hit = touching and t - last_touch > 0.5
            if hit:
                self.contacts += 1
            if touching:
                last_touch = t

            # --- yaw ----------------------------------------------------------------
            rate_cmd = float(np.clip(KP_YAW * wrap_pi(sp_yaw - est_yaw) + sp_rate,
                                     -YAW_RATE_MAX, YAW_RATE_MAX))
            yaw_rate += (rate_cmd - yaw_rate) * PHYS_DT / TAU_YAW
            yaw_rate += rng.normal(0.0, YAW_NOISE * s * sq)
            yaw += yaw_rate * PHYS_DT

            for key, value in (("t", t), ("x", pos[0]), ("y", pos[1]), ("yaw", yaw),
                               ("ex", est[0]), ("ey", est[1]),
                               ("sx", sp_pos[0]), ("sy", sp_pos[1]),
                               ("px", plan_xy[0]), ("py", plan_xy[1]),
                               ("ax", acc[0]), ("ay", acc[1]),
                               ("phase", phase), ("wall", touching), ("hit", hit)):
                rec[key].append(value)

            if phase == 4 and done_at is None:
                done_at = t
            step += 1
            t = step * PHYS_DT
            # Show 1 s after the landing condition is met; hard cap as a guard.
            if (done_at is not None and t - done_at > 1.0) or t > t_cap:
                break

        for key, values in rec.items():
            setattr(self, key, np.asarray(values))
        self.duration = float(self.t[-1])
        self.error = np.hypot(self.x - self.px, self.y - self.py)
        flying = self.phase >= 2
        self.max_err = float(self.error[flying].max()) if flying.any() else 0.0
        self.rms_err = float(np.sqrt(np.mean(self.error[flying] ** 2))) if flying.any() else 0.0
        self.end_err = float(self.error[-1])

    def index_at(self, t: float) -> int:
        return int(np.clip(round(t / PHYS_DT), 0, len(self.t) - 1))


def quad_shape(x: float, y: float, yaw: float, size: float):
    """Body triangle and four rotor centres (X frame) for drawing."""
    fwd = np.array((math.cos(yaw), math.sin(yaw)))
    side = np.array((-fwd[1], fwd[0]))
    pos = np.array((x, y))
    body = np.vstack((pos + 0.55 * size * fwd,
                      pos - 0.35 * size * fwd + 0.3 * size * side,
                      pos - 0.35 * size * fwd - 0.3 * size * side))
    rotors = [pos + size * 0.75 * (math.cos(a) * fwd + math.sin(a) * side)
              for a in (math.pi / 4, 3 * math.pi / 4, -3 * math.pi / 4, -math.pi / 4)]
    return body, rotors


class Simulation:
    def __init__(self, path: Path, plan: Plan, speed: float, loop: bool,
                 seed: int, wind: float):
        self.path, self.plan = path, plan
        self.speed, self.loop = speed, loop
        self.seed, self.wind_setting = seed, wind
        self.disturbed = wind > 0
        self.t = 0.0
        self.paused = False
        self.last_clock = time.monotonic()

        self.fig = plt.figure(figsize=(12, 9))
        grid = self.fig.add_gridspec(2, 1, height_ratios=(4.2, 1), hspace=0.28)
        self.ax = self.fig.add_subplot(grid[0])
        self.err_ax = self.fig.add_subplot(grid[1])
        self.fig.canvas.manager.set_window_title(f"Trajectory simulation - {path.name}")
        self._draw_arena()
        self.ax.plot(plan.x, plan.y, color="white", lw=1.2, alpha=0.8, label="Planned route")
        self.trail, = self.ax.plot([], [], color="#ff8c00", lw=1.6, label="Flown (simulated)")
        self.est_trail, = self.ax.plot([], [], color="#7b1fa2", lw=0.8, alpha=0.6,
                                       label="PX4's estimate")
        self.contact_pts, = self.ax.plot([], [], "X", color="#b71c1c", ms=9, zorder=12,
                                         label="Wall contact")
        self.ghost, = self.ax.plot([], [], "o", mfc="none", mec="#00e5ff", mew=1.8, ms=10,
                                   zorder=9, label="Setpoint")
        self.err_line, = self.ax.plot([], [], color="#00e5ff", lw=1, ls=":", zorder=9)
        self.tilt, = self.ax.plot([], [], color="#ffeb3b", lw=2.5, zorder=11,
                                  solid_capstyle="round")
        span = max(np.ptp(plan.x), np.ptp(plan.y), 1.0)
        self.size = float(np.clip(span * 0.035, 0.12, 0.22))
        self.body = Polygon(np.zeros((3, 2)), closed=True, fc="#1565c0", ec="white",
                            lw=1.2, zorder=10)
        self.ax.add_patch(self.body)
        self.rotors = [Circle((0, 0), self.size * 0.32, fc="#90caf9", ec="#0d47a1",
                              lw=1, alpha=0.85, zorder=10) for _ in range(4)]
        for rotor in self.rotors:
            self.ax.add_patch(rotor)
        self.info = self.ax.text(
            0.01, 0.99, "", transform=self.ax.transAxes, va="top", ha="left",
            fontsize=9, family="monospace", zorder=20,
            bbox={"boxstyle": "round,pad=0.35", "fc": "white", "alpha": 0.9})
        self.fig.text(0.5, 0.005,
                      "Space pause   R restart   N new random flight   D disturbances on/off   "
                      "+/- speed   L loop   Q close", ha="center", va="bottom", fontsize=8.5)
        self.ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8,
                       borderaxespad=0)

        self.err_curve, = self.err_ax.plot([], [], color="#ff8c00", lw=1.2)
        self.err_cursor = self.err_ax.axvline(0, color="#1565c0", lw=1)
        self.err_ax.set_ylabel("off route (m)")
        self.err_ax.set_xlabel("time (s)")
        self.err_ax.grid(True, alpha=0.3)

        self.fig.canvas.mpl_connect("key_press_event", self._on_key)
        self._fly()
        self.anim = FuncAnimation(self.fig, self._update, interval=33,
                                  blit=False, cache_frame_data=False)

    def _fly(self) -> None:
        wind = self.wind_setting if self.disturbed else 0.0
        self.flight = f = Flight(self.plan, self.seed, wind)
        self.t = 0.0
        self.err_curve.set_data(f.t, f.error)
        self.err_ax.set_xlim(0, f.duration)
        self.err_ax.set_ylim(0, max(0.1, f.error.max() * 1.15))
        for span in getattr(self, "_phase_spans", []):
            span.remove()
        self._phase_spans = []
        colours = {0: "#cfd8dc", 1: "#ffcdd2", 3: "#c8e6c9"}
        for ph, colour in colours.items():
            idx = np.flatnonzero(f.phase == ph)
            if len(idx):
                self._phase_spans.append(self.err_ax.axvspan(
                    f.t[idx[0]], f.t[idx[-1]], color=colour, alpha=0.6, lw=0))
        mode = f"disturbed (seed {f.seed}, wind {f.wind:g}x)" if f.wind else "ideal (no disturbances)"
        self.err_ax.set_title(f"Distance from planned route — {mode}; grey = stabilizing, "
                              "red = yaw alignment, green = settling", fontsize=8.5)
        print(f"[sim] {mode}: RMS {f.rms_err:.3f} m, max {f.max_err:.3f} m, "
              f"end {f.end_err:.3f} m, wall contacts {f.contacts}, "
              f"total {f.duration:.1f} s")

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
        ax.set_title(f"{self.path.name} — {fmt}, dt {p.dt * 1000:.0f} ms, {len(p.x)} pts")
        ax.grid(True, color="white", lw=0.5, alpha=0.3)

    def _on_key(self, event) -> None:
        key = (event.key or "").lower()
        if key == " ":
            self.paused = not self.paused
        elif key == "r":
            self.t, self.paused = 0.0, False
        elif key == "n":
            self.seed += 1
            self.disturbed = True
            self._fly()
        elif key == "d":
            self.disturbed = not self.disturbed
            self._fly()
        elif key in ("+", "="):
            self.speed = min(32.0, self.speed * 1.5)
        elif key in ("-", "_"):
            self.speed = max(0.1, self.speed / 1.5)
        elif key == "l":
            self.loop = not self.loop
        elif key in ("q", "escape"):
            plt.close(self.fig)
        self.last_clock = time.monotonic()

    def _update(self, _frame):
        f = self.flight
        now = time.monotonic()
        if not self.paused:
            self.t += (now - self.last_clock) * self.speed
            if self.t >= f.duration:
                self.t = 0.0 if self.loop else f.duration
        self.last_clock = now

        i = f.index_at(self.t)
        x, y, yaw = f.x[i], f.y[i], f.yaw[i]
        body, rotors = quad_shape(x, y, yaw, self.size)
        self.body.set_xy(body)
        phase = PHASES[f.phase[i]]
        colour = {"STABILIZING": "#607d8b", "ALIGNING YAW": "#c62828",
                  "SETTLING": "#2e7d32"}.get(phase, "#1565c0")
        self.body.set_facecolor("#b71c1c" if f.wall[i] else colour)
        for rotor, centre in zip(self.rotors, rotors):
            rotor.center = centre
        # Tilt line: direction and size of the thrust tilt the drone is using.
        tilt = np.array((f.ax[i], f.ay[i])) / (G * math.tan(TILT_MAX)) * self.size * 2.2
        self.tilt.set_data((x, x + tilt[0]), (y, y + tilt[1]))
        self.trail.set_data(f.x[:i + 1], f.y[:i + 1])
        self.est_trail.set_data(f.ex[:i + 1], f.ey[:i + 1])
        self.ghost.set_data((f.sx[i],), (f.sy[i],))
        self.err_line.set_data((x, f.sx[i]), (y, f.sy[i]))
        hits = np.flatnonzero(f.hit[:i + 1])
        self.contact_pts.set_data(f.x[hits], f.y[hits])
        self.err_cursor.set_xdata((f.t[i], f.t[i]))

        if self.paused and phase != "COMPLETE":
            phase += " (paused)"
        tilt_deg = math.degrees(math.atan(math.hypot(f.ax[i], f.ay[i]) / G))
        est_off = math.hypot(f.ex[i] - x, f.ey[i] - y)
        self.info.set_text(
            f"t   {f.t[i]:7.2f} / {f.duration:.2f} s   {phase}\n"
            f"pos {x:+6.2f}, {y:+6.2f} m   yaw {math.degrees(wrap_pi(yaw)):+6.1f} deg\n"
            f"off route {f.error[i]:.3f} m   tilt {tilt_deg:4.1f} deg\n"
            f"estimate off by {est_off:.3f} m\n"
            f"RMS {f.rms_err:.3f} m  max {f.max_err:.3f} m  walls {f.contacts}\n"
            f"playback {self.speed:.2f}x   loop {'on' if self.loop else 'off'}")
        return ()

    def show(self) -> None:
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
                        help="drone heading at take-off, deg (0 = North)")
    parser.add_argument("--loop", action="store_true", help="restart at the end")
    parser.add_argument("--seed", type=int, default=1,
                        help="random seed for disturbances (same seed = same flight)")
    parser.add_argument("--wind", type=float, default=1.0,
                        help="disturbance scale: 0 = ideal, 1 = typical indoor, 2 = rough")
    parser.add_argument("--ideal", action="store_true", help="same as --wind 0")
    args = parser.parse_args(argv)
    if not (math.isfinite(args.speed) and 0.1 <= args.speed <= 32):
        parser.error("--speed must be between 0.1 and 32")
    if not (math.isfinite(args.wind) and 0 <= args.wind <= 5):
        parser.error("--wind must be between 0 and 5")
    path = (args.trajectory or choose_trajectory()).resolve()
    if not path.is_file():
        parser.error(f"trajectory does not exist: {path}")
    try:
        plan = Plan(path, math.radians(args.start_heading))
    except ValueError as exc:
        # Same rejection the upload would give, so the drone could not fly it either.
        parser.error(str(exc))
    wind = 0.0 if args.ideal else args.wind
    Simulation(path, plan, args.speed, args.loop, args.seed, wind).show()


if __name__ == "__main__":
    main()
