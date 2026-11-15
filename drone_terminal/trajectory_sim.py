#!/usr/bin/env python3
"""Real-time 2D simulation of a trajectory CSV, flown by a simulated drone.

    python -m drone_terminal.trajectory_sim trajectory/rect_spiral_2.5x1.8.csv
    python -m drone_terminal.trajectory_sim            # pick from trajectory/
    python -m drone_terminal.trajectory_sim --ideal    # no disturbances

The CSV goes through the same loader and sample interval the terminal
uploads. The ESP32's decisions come from the real firmware: main/nav_task.c
(with vfh.c and odom.c) is compiled for the laptop by firmware_host.py and
its own nav_task() loop runs in lockstep with the simulation, so editing the
C code changes the simulation on the next run (it rebuilds automatically):

- The route is uploaded with nav_traj_put[_yaw] in the terminal's chunks.
- After take-off the drone hovers for a few seconds, then the terminal's
  START calls nav_traj_start[_yaw]; the firmware's own checks can refuse it.
- Whatever the firmware streams through the mavlink_task setpoint API
  (position, position + velocity, hold, ...) is what simulated PX4 tracks,
  after a short link delay; firmware log lines are printed to the console.
- After the firmware reports the route done, the laptop's flight runner
  waits until the drone has stayed within 0.15 m for 1 s before landing.

The drone follows the CDE1302 multirotor model (Lecture 4) and flight
control structure (Lecture 5), with parameters from the course's identified
quadrotor (m = 0.302 kg, J, c_T, c_M, motor curve, T_m = 0.02 s):

- motor dynamics: w = (C_R * throttle + w_b) / (T_m s + 1) per rotor;
- forces/moments: X4 allocation, thrust c_T w^2, yaw torque c_M w^2;
- 6-DOF rigid body: m dv/dt = m g e3 - f R e3 + drag + disturbance,
  J dw/dt = -w x J w + tau, Euler-angle kinematics dTheta/dt = W(Theta) w;
- control, as in PX4: position P (+ velocity feed-forward from the ESP32)
  -> velocity PID -> acceleration mapped to tilt and thrust (tilt limited)
  -> attitude P -> angular-rate PI -> control allocation -> motors.

On top of that it is pushed around by:

- turbulence (random gusts), a slow steady draft, and motor vibration;
- linear aerodynamic drag and angular damping;
- an imperfect position estimate (optical-flow drift + noise), so PX4
  controls what it believes, not where the drone really is;
- small random roll/pitch/yaw torques, and a 45 deg/s yaw-rate limit.

The flying area is drawn for reference only; there are no walls, so the
drone can fly past its edges.
The disturbance levels and PX4 gains are typical values, not identified on
our drone, so treat the result as "what kind of errors to expect".

The plot is a true top-down view: North (x) to the right, East (y) down.
Keys: Space pause, R restart, N new random flight, D disturbances on/off,
S result window (also opens by itself when the flight ends),
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
from drone_terminal.firmware_host import (  # noqa: E402
    NAV_IDLE, NAV_ROTATING, NAV_TRAJ, SP_HOLD, SP_POS_VEL, SP_POSITION,
    SP_VEL_XY_POS_Z, Firmware)

# Studio 1 flying area, relative to the launch-zone centre (metres, NED).
FLIGHT_X = (-0.45, 4.95)
FLIGHT_Y = (-2.25, 2.25)
LAUNCH_XY = (-0.45, -0.45, 0.90, 0.90)
SEARCH_XY = (0.90, -1.35, 3.60, 3.15)
TAG_CANDIDATES = tuple((x, y) for y in (1.35, -0.90) for x in (0.99, 2.25, 3.51))

# --- Firmware / flight-runner timing ---------------------------------------
TRAJ_ID = 1                # upload id the terminal uses
LINK_DELAY = 0.04          # ESP32 -> PX4 UART + PX4 scheduling
SETTLE_BEFORE_S = 3.0      # hover after take-off before START is sent
END_TOL_M, END_HOLD_S = 0.15, 1.0   # flight runner: endpoint before landing
END_TIMEOUT_S = 10.0

# --- Quadrotor model: CDE1302 Lecture 4/5 identified parameters ------------
G = 9.81
MASS = 0.302                         # kg
ARM = 0.0775                         # m, body centre to rotor
J = (3.9195e-4, 4.0515e-4, 6.3890e-4)   # kg m^2 (Jxx, Jyy, Jzz)
C_T = 2.9265e-7                      # N/(rad/s)^2, thrust coefficient
C_M = 4.7345e-9                      # N m/(rad/s)^2, torque coefficient
C_R, W_B = 3480.7, 445.7799          # throttle -> motor speed: w = C_R*u + w_b
T_M = 0.02                           # s, motor + propeller time constant
LIN_DRAG = 0.25                      # 1/s, drag by linear velocity
ANG_DAMP = 0.002                     # N m s/rad, damping moment by angular rate

# --- PX4 control (parameter names in comments; typical defaults) ------------
PHYS_DT = 0.005                      # physics + attitude/rate loop (200 Hz)
POS_EVERY = 4                        # position/velocity loop at 50 Hz
REC_DT = 0.01                        # recorded / animated samples
KP_XY, KP_Z = 0.95, 1.0              # MPC_XY_P, MPC_Z_P
VEL_XY_MAX, VEL_Z_MAX = 2.0, 1.0     # m/s
KV_XY = (1.8, 0.4, 0.2)              # MPC_XY_VEL_P/I/D_ACC
KV_Z = (4.0, 2.0, 0.0)               # MPC_Z_VEL_P/I/D_ACC
TILT_MAX = math.radians(35.0)        # MPC_TILTMAX_AIR
THR_MAX = 0.9                        # MPC_THR_MAX (fraction of max thrust)
K_ATT = (6.5, 6.5, 2.8)              # MC_ROLL_P, MC_PITCH_P, MC_YAW_P
RATE_MAX = (math.radians(220.0), math.radians(220.0), math.radians(45.0))
K_RATE_P = (20.0, 20.0, 10.0)        # angular-rate P (1/s), scaled by J
K_RATE_I = (5.0, 5.0, 2.0)
CRUISE_ALT = 0.5                     # m, main.c CRUISE_ALT_M
YAW_RATE_MAX = RATE_MAX[2]           # MPC_YAWRAUTO_MAX

# --- Disturbances (scaled by --wind; 0 = ideal) ----------------------------
GUST_SIGMA, GUST_TAU = 0.18, 1.5    # m/s^2, s (turbulence)
DRAFT_ACC = 0.05                    # m/s^2 steady draft (aircon, doors)
VIBRATION = 0.4                     # m/s^2 white motor/prop vibration
DRIFT = 0.005                       # m/sqrt(s), flow position drift
EST_NOISE, EST_VEL_NOISE = 0.01, 0.03
ATT_NOISE = 0.25                    # rad/s/sqrt(s) roll/pitch torque noise
YAW_NOISE = math.radians(8.0)       # rad/s/sqrt(s) yaw torque noise
TAKEOFF_POS_ERR, TAKEOFF_VEL = 0.07, 0.15   # leftover take-off motion


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
        self.pts = pts            # what the terminal uploads to the firmware


def setpoint_to_cmd(sp, est_yaw: float):
    """What PX4 tracks for a firmware setpoint (types as in mavlink_task.c).

    Returns (pos_ctl, x, y, vx_ff, vy_ff, z or None, vz_ff, yaw or None, yaw_rate).
    """
    yaw = sp.yaw if math.isfinite(sp.yaw) else est_yaw   # HOLD keeps the heading
    if sp.type == SP_POS_VEL:
        return (True, sp.x, sp.y, sp.vx, sp.vy, sp.z, sp.vz, yaw, 0.0)
    if sp.type in (SP_POSITION, SP_HOLD):
        return (True, sp.x, sp.y, 0.0, 0.0, sp.z, 0.0, yaw, 0.0)
    if sp.type == SP_VEL_XY_POS_Z:
        return (False, 0.0, 0.0, sp.vx, sp.vy, sp.z, 0.0, yaw, 0.0)
    return (False, 0.0, 0.0, sp.vx, sp.vy, None, sp.vz, None, sp.yaw_rate)


def _allocation():
    """X4 allocation (Lecture 4): [f, tx, ty, tz] = M [w1^2 .. w4^2]."""
    k = ARM * C_T * math.sqrt(2) / 2
    m = np.array([[C_T, C_T, C_T, C_T],
                  [-k, k, k, -k],
                  [k, -k, k, -k],
                  [C_M, C_M, -C_M, -C_M]])
    return m, np.linalg.inv(m)


ALLOC, ALLOC_INV = _allocation()
W_MAX = C_R + W_B
W_HOVER = math.sqrt(MASS * G / (4 * C_T))


class Flight:
    """One simulated flight, precomputed at PHYS_DT, recorded at REC_DT."""

    def __init__(self, plan: Plan, seed: int, wind: float):
        self.seed, self.wind = seed, wind
        rng = np.random.default_rng(seed)
        s = wind
        dt = PHYS_DT
        sq = math.sqrt(dt)
        cos, sin, tan = math.cos, math.sin, math.tan
        Jx, Jy, Jz = J
        M, Minv = ALLOC.tolist(), ALLOC_INV.tolist()
        gust_k = GUST_SIGMA * math.sqrt(2 * dt / GUST_TAU)
        f_max = THR_MAX * 4 * C_T * W_MAX ** 2

        t_cap = SETTLE_BEFORE_S + 30.0 + plan.play_s + END_TIMEOUT_S + 2.0
        n_steps = int(t_cap / dt) + 2
        # All random numbers up front (much faster than drawing per step).
        noise = (rng.normal(0.0, 1.0, (n_steps, 16)) * s).tolist()

        # --- true state (NED) ---
        px, py = (float(v) for v in rng.normal(0.0, TAKEOFF_POS_ERR * s, 2))
        pz = -CRUISE_ALT
        vx, vy = (float(v) for v in rng.normal(0.0, TAKEOFF_VEL * s, 2))
        vz = 0.0
        phi = theta = 0.0
        psi = plan.start_heading + float(rng.normal(0.0, math.radians(3.0) * s))
        wp = wq = wr = 0.0                       # body angular rates p, q, r
        rotor = [W_HOVER] * 4
        # --- environment ---
        gx = gy = 0.0
        ang = rng.uniform(0, 2 * math.pi)
        draft_x, draft_y = DRAFT_ACC * s * cos(ang), DRAFT_ACC * s * sin(ang)
        # --- estimator errors ---
        bx = by = 0.0
        nx = ny = nvx = nvy = 0.0
        lp = min(1.0, 0.2 * dt / 0.01)
        # --- controller state ---
        ix = iy = iz = 0.0                       # velocity integrators
        irp = irq = irr = 0.0                    # rate integrators
        prev_v = (vx, vy, vz)
        acc_f = [0.0, 0.0, 0.0]                  # filtered acceleration (D term)
        phi_d = theta_d = 0.0
        f_d = MASS * G

        # --- the real firmware (main/nav_task.c), compiled for the laptop ---
        fw = Firmware()
        fw.reset()
        fw_t0 = fw.now()
        fw.upload(TRAJ_ID, plan.pts)            # same chunks as the terminal
        dt_ms = round(plan.dt * 1000)
        started = False
        start_t = 0.0
        # --- laptop flight runner state ---
        phase = 0
        settle_start = settle_ok = None
        end_target = (0.0, 0.0)
        self.refused = False
        # Before START, mission_task holds the take-off point.
        hover = (True, 0.0, 0.0, 0.0, 0.0, -CRUISE_ALT, 0.0, plan.start_heading, 0.0)
        cmd = hover                              # setpoint active at PX4
        pending = []
        plan_x = plan_y = 0.0
        last = len(plan.x) - 1

        rec = {k: [] for k in ("t", "x", "y", "z", "yaw", "roll", "pitch", "ex", "ey",
                               "sx", "sy", "px", "py", "ax", "ay", "phase")}
        rec_every = round(REC_DT / dt)
        done_at = None
        step = 0
        t = 0.0

        while step < n_steps:
            r = noise[step]
            # --- what PX4 believes (EKF2 with optical flow) --------------------
            bx += DRIFT * sq * r[0]
            by += DRIFT * sq * r[1]
            nx += (EST_NOISE * r[2] - nx) * lp
            ny += (EST_NOISE * r[3] - ny) * lp
            nvx += (EST_VEL_NOISE * r[4] - nvx) * lp
            nvy += (EST_VEL_NOISE * r[5] - nvy) * lp
            ex, ey, ez = px + bx + nx, py + by + ny, pz
            evx, evy, evz = vx + nvx, vy + nvy, vz
            epsi = psi + 0.0087 * r[6]

            # --- ESP32: the firmware sees PX4's estimate over MAVLink -----------
            fw.set_state(ex, ey, ez, evx, evy, evz, wrap_pi(epsi))
            if not started and t >= SETTLE_BEFORE_S:
                fw.start(TRAJ_ID, dt_ms, plan.has_yaw)     # the terminal's START
                started, start_t = True, t
            if fw.advance(dt) and started and phase < 4:   # a nav_task() loop ran
                state = fw.nav_state()
                if state == NAV_ROTATING:
                    phase = 1
                    plan_x, plan_y = float(plan.x[0]), float(plan.y[0])
                elif state == NAV_TRAJ:
                    phase = 2
                    k = min(max(fw.traj_progress(), 0.0), last)
                    i = min(int(k), last - 1)
                    a = k - i
                    plan_x = float(plan.x[i] + a * (plan.x[i + 1] - plan.x[i]))
                    plan_y = float(plan.y[i] + a * (plan.y[i + 1] - plan.y[i]))
                elif state == NAV_IDLE and phase == 0 and t - start_t > 0.5:
                    self.refused = True                    # firmware refused START
                    phase = 4
                elif phase >= 1:
                    # Playback over (ARRIVED); the laptop flight runner waits
                    # for the drone to settle on the end point before landing.
                    if phase < 3:
                        phase, settle_start = 3, t
                        plan_x, plan_y = float(plan.x[last]), float(plan.y[last])
                    sp_now = fw.setpoint()
                    if math.isfinite(sp_now.x):
                        end_target = (sp_now.x, sp_now.y)
                    near = math.hypot(ex - end_target[0], ey - end_target[1]) < END_TOL_M
                    settle_ok = (settle_ok or t) if near else None
                    if ((settle_ok is not None and t - settle_ok >= END_HOLD_S)
                            or t - settle_start > END_TIMEOUT_S):
                        phase = 4
                if phase >= 1:
                    pending.append((t + LINK_DELAY, setpoint_to_cmd(fw.setpoint(), epsi)))
            while pending and pending[0][0] <= t:
                cmd = pending.pop(0)[1]
            pos_ctl, spx, spy, ffx, ffy, sp_z, ffz, sp_yaw, sp_rate = cmd

            # --- PX4 position + velocity control (50 Hz) -------------------------
            if step % POS_EVERY == 0:
                dtp = dt * POS_EVERY
                # Position loop only for setpoints that carry a position
                # (PX4 bypasses it for velocity setpoints).
                cvx = ffx + (KP_XY * (spx - ex) if pos_ctl else 0.0)
                cvy = ffy + (KP_XY * (spy - ey) if pos_ctl else 0.0)
                mag = math.hypot(cvx, cvy)
                if mag > VEL_XY_MAX:
                    cvx, cvy = cvx * VEL_XY_MAX / mag, cvy * VEL_XY_MAX / mag
                cvz = ffz + (KP_Z * (sp_z - ez) if sp_z is not None else 0.0)
                cvz = max(-VEL_Z_MAX, min(VEL_Z_MAX, cvz))
                e_vx, e_vy, e_vz = cvx - evx, cvy - evy, cvz - evz
                ix = max(-1.0, min(1.0, ix + e_vx * dtp))
                iy = max(-1.0, min(1.0, iy + e_vy * dtp))
                iz = max(-1.0, min(1.0, iz + e_vz * dtp))
                acc_f[0] += ((evx - prev_v[0]) / dtp - acc_f[0]) * 0.3
                acc_f[1] += ((evy - prev_v[1]) / dtp - acc_f[1]) * 0.3
                acc_f[2] += ((evz - prev_v[2]) / dtp - acc_f[2]) * 0.3
                prev_v = (evx, evy, evz)
                adx = KV_XY[0] * e_vx + KV_XY[1] * ix - KV_XY[2] * acc_f[0]
                ady = KV_XY[0] * e_vy + KV_XY[1] * iy - KV_XY[2] * acc_f[1]
                adz = KV_Z[0] * e_vz + KV_Z[1] * iz - KV_Z[2] * acc_f[2]
                # Acceleration -> thrust vector (Lecture 5): A = a_d - g e3.
                Ax, Ay, Az = adx, ady, min(adz - G, -0.2 * G)
                h = math.hypot(Ax, Ay)
                h_max = -Az * tan(TILT_MAX)
                if h > h_max:
                    Ax, Ay = Ax * h_max / h, Ay * h_max / h
                An = math.sqrt(Ax * Ax + Ay * Ay + Az * Az)
                f_d = min(MASS * An, f_max)
                # Thrust direction -> roll/pitch setpoints in the yaw frame.
                cpsi, spsi = cos(epsi), sin(epsi)
                abx = cpsi * Ax + spsi * Ay
                aby = -spsi * Ax + cpsi * Ay
                theta_d = math.atan2(-abx, -Az)
                phi_d = math.asin(max(-1.0, min(1.0, aby / An)))

            # --- attitude P -> angular-rate PI (200 Hz) ---------------------------
            ephi = phi + 0.0035 * r[7]
            etheta = theta + 0.0035 * r[8]
            rp_d = max(-RATE_MAX[0], min(RATE_MAX[0], K_ATT[0] * (phi_d - ephi)))
            rq_d = max(-RATE_MAX[1], min(RATE_MAX[1], K_ATT[1] * (theta_d - etheta)))
            yaw_cmd = K_ATT[2] * wrap_pi(sp_yaw - epsi) if sp_yaw is not None else sp_rate
            rr_d = max(-RATE_MAX[2], min(RATE_MAX[2], yaw_cmd))
            e_p, e_q, e_r = rp_d - wp, rq_d - wq, rr_d - wr
            irp = max(-0.5, min(0.5, irp + e_p * dt))
            irq = max(-0.5, min(0.5, irq + e_q * dt))
            irr = max(-0.5, min(0.5, irr + e_r * dt))
            u0 = f_d
            u1 = Jx * (K_RATE_P[0] * e_p + K_RATE_I[0] * irp)
            u2 = Jy * (K_RATE_P[1] * e_q + K_RATE_I[1] * irq)
            u3 = Jz * (K_RATE_P[2] * e_r + K_RATE_I[2] * irr)

            # --- control allocation -> throttle -> motor dynamics ----------------
            for i in range(4):
                mi = Minv[i]
                w2 = mi[0] * u0 + mi[1] * u1 + mi[2] * u2 + mi[3] * u3
                w_cmd = math.sqrt(w2) if w2 > 0.0 else 0.0
                throttle = (w_cmd - W_B) / C_R
                throttle = 0.0 if throttle < 0.0 else (1.0 if throttle > 1.0 else throttle)
                rotor[i] += (C_R * throttle + W_B - rotor[i]) * dt / T_M
            a0, a1, a2, a3 = (w * w for w in rotor)
            f = M[0][0] * a0 + M[0][1] * a1 + M[0][2] * a2 + M[0][3] * a3
            tx = M[1][0] * a0 + M[1][1] * a1 + M[1][2] * a2 + M[1][3] * a3
            ty = M[2][0] * a0 + M[2][1] * a1 + M[2][2] * a2 + M[2][3] * a3
            tz = M[3][0] * a0 + M[3][1] * a1 + M[3][2] * a2 + M[3][3] * a3

            # --- rotational dynamics: J dw/dt = -w x Jw + tau ---------------------
            dp = (tx - (Jz - Jy) * wq * wr - ANG_DAMP * wp) / Jx
            dq = (ty - (Jx - Jz) * wp * wr - ANG_DAMP * wq) / Jy
            dr = (tz - (Jy - Jx) * wp * wq - ANG_DAMP * wr) / Jz
            wp += dp * dt + ATT_NOISE * sq * r[9]
            wq += dq * dt + ATT_NOISE * sq * r[10]
            wr += dr * dt + YAW_NOISE * sq * r[11]
            # Euler kinematics: dTheta/dt = W(Theta) w
            cphi, sphi, cth, tth = cos(phi), sin(phi), cos(theta), tan(theta)
            phi += (wp + sphi * tth * wq + cphi * tth * wr) * dt
            theta += (cphi * wq - sphi * wr) * dt
            psi += ((sphi * wq + cphi * wr) / cth) * dt

            # --- translational dynamics: m dv/dt = m g e3 - f R e3 + ... ---------
            cphi, sphi = cos(phi), sin(phi)
            cth, sth = cos(theta), sin(theta)
            cps, sps = cos(psi), sin(psi)
            fm = f / MASS
            fax = -fm * (cphi * sth * cps + sphi * sps)
            fay = -fm * (cphi * sth * sps - sphi * cps)
            faz = -fm * cphi * cth
            gx += -gx * dt / GUST_TAU + gust_k * r[12]
            gy += -gy * dt / GUST_TAU + gust_k * r[13]
            vx += (fax - LIN_DRAG * vx + draft_x + gx + VIBRATION * r[14]) * dt
            vy += (fay - LIN_DRAG * vy + draft_y + gy + VIBRATION * r[15]) * dt
            vz += (G + faz - LIN_DRAG * vz + 0.35 * VIBRATION * (r[14] - r[15])) * dt
            px += vx * dt
            py += vy * dt
            pz += vz * dt

            if step % rec_every == 0:
                for key, value in (("t", t), ("x", px), ("y", py), ("z", pz), ("yaw", psi),
                                   ("roll", phi), ("pitch", theta),
                                   ("ex", ex), ("ey", ey),
                                   ("sx", spx if pos_ctl else ex), ("sy", spy if pos_ctl else ey),
                                   ("px", plan_x), ("py", plan_y), ("ax", fax), ("ay", fay),
                                   ("phase", phase)):
                    rec[key].append(value)

            if phase == 4 and done_at is None:
                done_at = t
            step += 1
            t = step * dt
            if done_at is not None and t - done_at > 1.0:
                break

        self.fw_log = fw.take_log(since=fw_t0)
        self.fw_name = fw.path.name
        for key, values in rec.items():
            setattr(self, key, np.asarray(values))
        self.duration = float(self.t[-1])
        self.error = np.hypot(self.x - self.px, self.y - self.py)
        flying = self.phase >= 2
        self.max_err = float(self.error[flying].max()) if flying.any() else 0.0
        self.rms_err = float(np.sqrt(np.mean(self.error[flying] ** 2))) if flying.any() else 0.0
        self.end_err = float(self.error[-1])

    def index_at(self, t: float) -> int:
        return int(np.clip(round(t / REC_DT), 0, len(self.t) - 1))


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
                      "Space pause   R restart   N new random flight   D disturbances on/off   S result window   "
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
        self._summary_shown = False
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
        for line in f.fw_log[:20]:
            print(f"[firmware] {line}")
        if f.refused:
            print("[sim] the firmware refused START; see its log above")
        print(f"[sim] {mode}: RMS {f.rms_err:.3f} m, max {f.max_err:.3f} m, "
              f"end {f.end_err:.3f} m, "
              f"total {f.duration:.1f} s  (firmware build {f.fw_name})")

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
            self._summary_shown = False
        elif key == "s":
            self._show_summary()
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
                if not self._summary_shown:
                    self._summary_shown = True
                    self._show_summary()
                self.t = 0.0 if self.loop else f.duration
        self.last_clock = now

        i = f.index_at(self.t)
        x, y, yaw = f.x[i], f.y[i], f.yaw[i]
        body, rotors = quad_shape(x, y, yaw, self.size)
        self.body.set_xy(body)
        phase = PHASES[f.phase[i]]
        colour = {"STABILIZING": "#607d8b", "ALIGNING YAW": "#c62828",
                  "SETTLING": "#2e7d32"}.get(phase, "#1565c0")
        self.body.set_facecolor(colour)
        for rotor, centre in zip(self.rotors, rotors):
            rotor.center = centre
        # Tilt line: direction and size of the thrust tilt the drone is using.
        tilt = np.array((f.ax[i], f.ay[i])) / (G * math.tan(TILT_MAX)) * self.size * 2.2
        self.tilt.set_data((x, x + tilt[0]), (y, y + tilt[1]))
        self.trail.set_data(f.x[:i + 1], f.y[:i + 1])
        self.est_trail.set_data(f.ex[:i + 1], f.ey[:i + 1])
        self.ghost.set_data((f.sx[i],), (f.sy[i],))
        self.err_line.set_data((x, f.sx[i]), (y, f.sy[i]))
        self.err_cursor.set_xdata((f.t[i], f.t[i]))

        if self.paused and phase != "COMPLETE":
            phase += " (paused)"
        roll, pitch = math.degrees(f.roll[i]), math.degrees(f.pitch[i])
        est_off = math.hypot(f.ex[i] - x, f.ey[i] - y)
        self.info.set_text(
            f"t   {f.t[i]:7.2f} / {f.duration:.2f} s   {phase}\n"
            f"pos {x:+6.2f}, {y:+6.2f} m   alt {-f.z[i]:.2f} m\n"
            f"roll {roll:+5.1f}  pitch {pitch:+5.1f}  yaw {math.degrees(wrap_pi(yaw)):+6.1f} deg\n"
            f"off route {f.error[i]:.3f} m   estimate off by {est_off:.3f} m\n"
            f"RMS {f.rms_err:.3f} m  max {f.max_err:.3f} m\n"
            f"playback {self.speed:.2f}x   loop {'on' if self.loop else 'off'}")
        return ()

    def _show_summary(self):
        """Open a clean window with just the arena, the route and the flown path."""
        f, p = self.flight, self.plan
        fig, ax = plt.subplots(figsize=(8.5, 7))
        fig.canvas.manager.set_window_title(f"Flight result - {self.path.name}")
        ax.add_patch(Rectangle((FLIGHT_X[0], FLIGHT_Y[0]), np.diff(FLIGHT_X)[0],
                               np.diff(FLIGHT_Y)[0], fc="#f5f5f5", ec="#616161", lw=1.8,
                               zorder=0))
        for tx, ty in TAG_CANDIDATES:
            ax.add_patch(Circle((tx, ty), 0.05, fill=False, ec="#424242", zorder=2))
        ax.plot(p.x, p.y, color="#9e9e9e", lw=1.3, ls="--", label="Planned route", zorder=3)
        ax.plot(f.x, f.y, color="#ff8c00", lw=1.5, label="Flown (simulated)", zorder=4)
        ax.plot(f.x[0], f.y[0], "o", color="#2e7d32", ms=8, zorder=6, label="Start")
        body, rotors = quad_shape(f.x[-1], f.y[-1], f.yaw[-1], self.size)
        ax.add_patch(Polygon(body, closed=True, fc="#1565c0", ec="white", lw=1.2, zorder=8,
                             label="Drone (end)"))
        for centre in rotors:
            ax.add_patch(Circle(centre, self.size * 0.32, fc="#90caf9", ec="#0d47a1",
                                lw=1, zorder=7))
        pad = 0.2
        ax.set_xlim(min(FLIGHT_X[0], p.x.min()) - pad, max(FLIGHT_X[1], p.x.max()) + pad)
        ax.set_ylim(min(FLIGHT_Y[0], p.y.min()) - pad, max(FLIGHT_Y[1], p.y.max()) + pad)
        ax.invert_yaxis()  # same top-down view as the main window
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x / North (m)  →")
        ax.set_ylabel("y / East (m)  ↓")
        ax.grid(True, color="#e0e0e0", lw=0.8)
        mode = f"seed {f.seed}, wind {f.wind:g}x" if f.wind else "no disturbances"
        ax.set_title(f"{self.path.name} ({mode})\n"
                     f"{f.duration:.1f} s, RMS {f.rms_err:.2f} m, max {f.max_err:.2f} m",
                     fontsize=10.5)
        ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=8.5, borderaxespad=0)
        fig.tight_layout()
        fig.show()
        self._summary_figs = getattr(self, "_summary_figs", []) + [fig]
        return fig

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
