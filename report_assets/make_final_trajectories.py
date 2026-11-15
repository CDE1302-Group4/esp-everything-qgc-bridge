#!/usr/bin/env python3
"""Draw the 'Final non-yaw trajectory designs' figure for the report.

    python report_assets/make_final_trajectories.py

Plots each CSV top-down (East to the right, North up) and writes
final_trajectories_circles_under200.png next to this script. Titles take the duration
from the last CSV timestamp.
"""

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = Path(__file__).with_name("final_trajectories_circles_under200.png")
ROUTES = [
    (ROOT / "trajectory" / "flying_area_search_5p4x4p5_under200s.csv", "search route"),
    (ROOT / "trajectory" / "flying_area_search_5p4x4p5_circles_239s.csv", "six-circle route"),
]


def load(path):
    t, north, east = [], [], []
    with path.open(newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"]))
            north.append(float(row["x"]))
            east.append(float(row["y"]))
    return t, north, east


fig, axes = plt.subplots(1, 2, figsize=(12, 5.2))
for ax, (path, name) in zip(axes, ROUTES):
    t, north, east = load(path)
    ax.plot(east, north, color="#2f6fdb", lw=1.4)
    ax.plot(east[0], north[0], "o", color="#2ca02c", ms=8, label="Start", zorder=3)
    ax.plot(east[-1], north[-1], "x", color="#d62728", ms=9, mew=2, label="End", zorder=4)
    ax.set_title(f"{t[-1]:g} s {name}", fontweight="bold", fontsize=11)
    ax.set_xlabel("Right / East (m)")
    ax.set_ylabel("Forward / North (m)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, color="#dddddd", lw=0.8)
    ax.legend(loc="upper right", fontsize=8)

fig.suptitle("Final non-yaw trajectory designs", fontweight="bold", fontsize=14)
fig.tight_layout()
fig.savefig(OUT, dpi=300, facecolor="white")
print(f"Wrote {OUT}")
