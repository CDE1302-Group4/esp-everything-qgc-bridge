#!/usr/bin/env python3
"""Draw the camera-to-map transform figure for the report.

    python report_assets/make_camera_to_map.py

(a) side view: camera frame -> drone frame (45 deg pitch, 0.02 m offset)
(b) top view: drone frame -> map frame (yaw psi, plus drone position)
Numbers follow laptop/apriltag_host.py camera_to_body() and
laptop/tag_stream.py StartMap.observe().
"""

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Arc, FancyArrowPatch, Rectangle  # noqa: E402

OUT = Path(__file__).with_name("camera_to_map.png")

INK, MUTED = "#1f2a3a", "#5b6676"
BODY, CAM, TAG, MAPC = "#2f6fdb", "#e07b00", "#d62728", "#2ca02c"

P = math.radians(45.0)
CAM_F = 0.02
# Worked example from apriltag_host.camera_to_body's docstring.
XC, YC, ZC = 0.0, -0.354, 1.061
F = CAM_F - YC * math.sin(P) + ZC * math.cos(P)
D = YC * math.cos(P) + ZC * math.sin(P)


def vec(ax, x0, y0, x1, y1, color, lw=2.0, ls="-", head=True):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), color=color, lw=lw, linestyle=ls,
                                 arrowstyle="-|>" if head else "-", mutation_scale=14,
                                 shrinkA=0, shrinkB=0))


fig, (a, b) = plt.subplots(1, 2, figsize=(12.5, 5.6), gridspec_kw={"width_ratios": [1.05, 1]})

# ---- (a) side view: x = forward, y = up (so down is drawn downward) -------
a.add_patch(Rectangle((-0.16, -0.035), 0.32, 0.07, fc="#dbe4f5", ec=BODY, lw=1.2))
for px in (-0.2, 0.2):
    a.plot([px - 0.08, px + 0.08], [0.06, 0.06], color=BODY, lw=2.5)
    a.plot([px, px], [0.035, 0.06], color=BODY, lw=1.2)
a.text(-0.3, 0.12, "drone (camera 0.02 m ahead of centre)", fontsize=8.5, color=MUTED)

vec(a, 0, 0, 0, -0.42, BODY)
a.text(-0.03, -0.44, "d (down)", color=BODY, fontsize=10, va="top", ha="right")
vec(a, 0, 0, 1.25, 0, BODY, lw=1.2)
a.text(1.27, 0.0, "f (forward)", color=BODY, fontsize=10, va="center")

# Camera at (0.02, 0); z_c along the 45 deg optical axis, y_c = image down.
cx, cy = CAM_F, 0.0
zc_dir = (math.cos(P), -math.sin(P))
yc_dir = (-math.sin(P), -math.cos(P))
a.plot(cx, cy, "s", color=CAM, ms=7, zorder=5)
vec(a, cx, cy, cx + 0.38 * zc_dir[0], cy + 0.38 * zc_dir[1], CAM)
a.text(cx + 0.30, cy - 0.33, r"$z_c$ (optical axis)", color=CAM, fontsize=10)
vec(a, cx, cy, cx + 0.24 * yc_dir[0], cy + 0.24 * yc_dir[1], CAM)
a.text(cx + 0.24 * yc_dir[0] - 0.03, cy + 0.24 * yc_dir[1] - 0.02, r"$y_c$",
       color=CAM, fontsize=10, ha="right", va="top")
a.text(-0.3, -0.62, r"$x_c$ points into the page and equals r (right)",
       color=CAM, fontsize=8.5)
a.add_patch(Arc((cx, cy), 0.56, 0.56, theta1=-45, theta2=0, color=CAM, lw=1.2))
a.text(cx + 0.3, cy - 0.12, r"$p = 45^\circ$", color=CAM, fontsize=10)

# Line of sight to the tag and its drone-frame components.
tx, ty = F, -D
a.plot([cx, tx], [cy, ty], color=CAM, lw=1.2, ls="--")
a.plot([tx, tx], [0, ty], color=BODY, lw=1.1, ls="--")
a.plot([0, tx], [ty, ty], color=BODY, lw=1.1, ls="--")
a.text(tx / 2 + 0.05, ty - 0.035, f"f = {F:.2f} m", color=BODY, fontsize=9.5,
       ha="center", va="top")
a.text(tx + 0.03, ty / 2, f"d = {D:.2f} m", color=BODY, fontsize=9.5, va="center")
a.plot(tx, ty, "D", color=TAG, ms=9, zorder=6)
a.text(tx + 0.05, ty - 0.02, "tag", color=TAG, fontsize=10, fontweight="bold", va="center")
a.text(0.5, -0.17, "camera measures\n" + rf"$(x_c, y_c, z_c)$ = ({XC:g}, {YC:g}, {ZC:g})",
       color=CAM, fontsize=8.5, rotation=-26, rotation_mode="anchor")

a.set_title("(a) Side view: camera frame → drone frame", fontweight="bold", fontsize=11)
a.set_xlim(-0.32, 1.6)
a.set_ylim(-0.68, 0.22)
a.set_aspect("equal")
a.axis("off")

# ---- (b) top view: x = East, y = North ------------------------------------
psi = math.radians(30)
dn, de = 1.5, 0.5
f2, r2 = 1.0, 0.35
fwd = (math.sin(psi), math.cos(psi))        # (E, N)
rgt = (math.cos(psi), -math.sin(psi))
dN = f2 * math.cos(psi) - r2 * math.sin(psi)
dE = f2 * math.sin(psi) + r2 * math.cos(psi)
tn, te = dn + dN, de + dE

b.grid(True, color="#e3e3e3", lw=0.8)
b.plot(0, 0, "o", color=MAPC, ms=8)
b.text(0.05, -0.12, "map origin (start)", color=MAPC, fontsize=9)
b.text(de, tn + 0.08, "N", color=MUTED, fontsize=9, ha="center")
b.add_patch(Arc((de, dn), 0.6, 0.6, theta1=90 - math.degrees(psi), theta2=90, color=INK, lw=1.2))
b.text(de + 0.07, dn + 0.33, r"$\psi$", color=INK, fontsize=12)

vec(b, 0, 0, de, dn, MUTED, lw=1.2, ls="--")
b.text(de / 2 + 0.05, dn / 2 - 0.05, r"drone ($N_d$, $E_d$)", color=MUTED, fontsize=9)
b.plot(de, dn, "o", color=BODY, ms=7, zorder=5)
vec(b, de, dn, de + 0.5 * fwd[0], dn + 0.5 * fwd[1], BODY)
b.text(de + 0.5 * fwd[0] - 0.05, dn + 0.5 * fwd[1] + 0.06, "f", color=BODY, fontsize=11)
vec(b, de, dn, de + 0.4 * rgt[0], dn + 0.4 * rgt[1], BODY)
b.text(de + 0.4 * rgt[0] + 0.04, dn + 0.4 * rgt[1] - 0.06, "r", color=BODY, fontsize=11)

mid = (de + f2 * fwd[0], dn + f2 * fwd[1])
vec(b, de, dn, mid[0], mid[1], BODY, lw=1.1, ls="--", head=False)
vec(b, mid[0], mid[1], te, tn, BODY, lw=1.1, ls="--", head=False)
vec(b, de, dn, de, tn, TAG, lw=1.1, ls=":", head=False)
vec(b, de, tn, te, tn, TAG, lw=1.1, ls=":", head=False)
b.text(de - 0.05, (dn + tn) / 2, r"$\Delta N$", color=TAG, fontsize=11, ha="right")
b.text((de + te) / 2 + 0.22, tn - 0.06, r"$\Delta E$", color=TAG, fontsize=11, ha="center", va="top")
b.plot(te, tn, "D", color=TAG, ms=9, zorder=6)
b.text(te + 0.06, tn - 0.02, "tag", color=TAG, fontsize=10, fontweight="bold")

b.set_title("(b) Top view: drone frame → map frame", fontweight="bold", fontsize=11)
b.set_xlabel("East (m)")
b.set_ylabel("North (m)")
b.set_xlim(-0.3, 1.9)
b.set_ylim(-0.3, 2.75)
b.set_aspect("equal")

fig.tight_layout()
fig.savefig(OUT, dpi=300, facecolor="white")
print(f"Wrote {OUT}  (example: f={F:.3f}, d={D:.3f})")
