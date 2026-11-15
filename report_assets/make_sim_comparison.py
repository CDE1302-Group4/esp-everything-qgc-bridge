#!/usr/bin/env python3
"""Compare the 239.5 s and 400 s six-circle routes in the trajectory simulator.

    python report_assets/make_sim_comparison.py

Both routes follow the same path; only the speed differs. Each is flown
SEEDS times with typical indoor disturbances (same seeds for both, so both
routes see the same random conditions), plus once with no disturbances.
Writes sim_comparison.png and prints a summary table.
"""

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT), str(ROOT / "laptop")]

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402
import numpy as np  # noqa: E402

from drone_terminal.trajectory_sim import (  # noqa: E402
    FLIGHT_X, FLIGHT_Y, TAG_CANDIDATES, Flight, Plan)

OUT = Path(__file__).with_name("sim_comparison.png")
SEEDS = range(1, 21)
ROUTES = [
    ("Faster (239.5 s)", ROOT / "trajectory" / "flying_area_search_5p4x4p5_circles_239s.csv",
     "#e65100"),
    ("Slower (400 s)", ROOT / "trajectory" / "flying_area_search_5p4x4p5_circles_400s.csv",
     "#1565c0"),
]
EXAMPLE_SEED = 3
NEAR_TAG_M = 0.6


def stats(flight):
    flying = flight.phase == 2              # route playback only
    err = flight.error[flying]
    speed = np.hypot(np.diff(flight.x[flying]), np.diff(flight.y[flying])) / 0.01
    # Seconds spent within NEAR_TAG_M of each candidate tag (true position),
    # averaged over the six tags: the time the camera has to see each one.
    x, y = flight.x[flying], flight.y[flying]
    dwell = np.mean([np.sum(np.hypot(x - tx, y - ty) < NEAR_TAG_M) * 0.01
                     for tx, ty in TAG_CANDIDATES])
    return dict(rms=np.sqrt(np.mean(err ** 2)), p95=np.percentile(err, 95),
                max=err.max(), peak_speed=np.percentile(speed, 99),
                dwell=dwell)


results = {}
for name, path, _ in ROUTES:
    plan = Plan(path, 0.0)
    ideal = Flight(plan, 1, 0.0)
    runs = [stats(Flight(plan, seed, 1.0)) for seed in SEEDS]
    results[name] = dict(plan=plan, ideal=stats(ideal), runs=runs,
                         example=Flight(plan, EXAMPLE_SEED, 1.0))

print(f"{'route':18} {'ideal RMS':>9} {'RMS mean':>9} {'RMS range':>13} "
      f"{'95th %':>7} {'max':>6} {'s near each tag':>16}")
for name, r in results.items():
    rms = np.array([x["rms"] for x in r["runs"]])
    p95 = np.mean([x["p95"] for x in r["runs"]])
    mx = np.mean([x["max"] for x in r["runs"]])
    print(f"{name:18} {r['ideal']['rms']:9.3f} {rms.mean():9.3f} "
          f"{rms.min():6.3f}-{rms.max():.3f} {p95:7.3f} {mx:6.3f} "
          f"{np.mean([x['dwell'] for x in r['runs']]):16.1f}")

fig = plt.figure(figsize=(13, 5.6))
grid = fig.add_gridspec(1, 3, width_ratios=(1, 1, 0.8), wspace=0.28)

for col, (name, _, colour) in enumerate(ROUTES):
    ax = fig.add_subplot(grid[col])
    r = results[name]
    f, plan = r["example"], r["plan"]
    ax.add_patch(Rectangle((FLIGHT_Y[0], FLIGHT_X[0]), np.diff(FLIGHT_Y)[0],
                           np.diff(FLIGHT_X)[0], fill=False, ec="#555555", lw=1.5))
    for tx, ty in TAG_CANDIDATES:
        ax.add_patch(Circle((ty, tx), 0.05, fill=False, ec="#333333"))
    ax.plot(plan.y, plan.x, color="#9e9e9e", lw=1.2, ls="--", label="Planned")
    ax.plot(f.y, f.x, color=colour, lw=1.1, label="Simulated flight")
    s = stats(f)
    ax.set_title(f"{name}\nRMS {s['rms']:.2f} m, max {s['max']:.2f} m",
                 fontsize=10.5, fontweight="bold")
    ax.set_xlabel("Right / East (m)")
    ax.set_ylabel("Forward / North (m)")
    ax.set_aspect("equal")
    ax.set_xlim(FLIGHT_Y[0] - 0.1, FLIGHT_Y[1] + 0.1)
    ax.set_ylim(FLIGHT_X[0] - 0.1, FLIGHT_X[1] + 0.1)
    ax.grid(True, color="#e5e5e5")
    ax.legend(loc="upper right", fontsize=7.5)

ax = fig.add_subplot(grid[2])
names = [n for n, _, _ in ROUTES]
data = [[x["rms"] for x in results[n]["runs"]] for n in names]
box = ax.boxplot(data, widths=0.5, patch_artist=True, showfliers=False,
                 medianprops={"color": "black"})
for patch, (_, _, colour) in zip(box["boxes"], ROUTES):
    patch.set_facecolor(colour)
    patch.set_alpha(0.35)
for i, values in enumerate(data, 1):
    ax.scatter(np.full(len(values), i) + np.linspace(-0.12, 0.12, len(values)), values,
               s=12, color=ROUTES[i - 1][2], zorder=3)
    ax.plot([i - 0.3, i + 0.3], [results[names[i - 1]]["ideal"]["rms"]] * 2,
            color="#2e7d32", lw=2, label="No disturbances" if i == 1 else None)
ax.set_xticks([1, 2], [n.replace(" (", "\n(") for n in names])
ax.set_ylabel("RMS distance from route (m)")
ax.set_title(f"Tracking error over {len(SEEDS)} simulated flights",
             fontsize=10.5, fontweight="bold")
ax.set_ylim(bottom=0)
ax.grid(True, axis="y", color="#e5e5e5")
ax.legend(loc="upper right", fontsize=8)

fig.suptitle("Simulated six-circle route: same path, two speeds "
             "(typical indoor disturbances)", fontsize=13, fontweight="bold")
fig.savefig(OUT, dpi=300, facecolor="white", bbox_inches="tight")
print(f"Wrote {OUT}")
