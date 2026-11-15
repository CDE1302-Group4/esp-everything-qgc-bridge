#!/usr/bin/env python3
"""Draw the system architecture figure for the report.

    python report_assets/make_architecture_diagram.py

Writes system_architecture.png (300 dpi) next to this script.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch  # noqa: E402

OUT = Path(__file__).with_name("system_architecture.png")

INK = "#1f2a3a"
MUTED = "#5b6676"
BLUE = "#2f6fdb"
GREY = "#4a5566"
LAPTOP_FILL, LAPTOP_EDGE = "#f3f7fe", "#3b6fd6"
DRONE_FILL, DRONE_EDGE = "#f4f5f7", "#9aa3b0"
BOX_EDGE = "#a9bde6"

fig = plt.figure(figsize=(12.8, 10.0))
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 16)
ax.set_ylim(0, 12.5)
ax.axis("off")


def panel(x, y, w, h, fill, edge, lw=1.6):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0,rounding_size=0.18",
                                fc=fill, ec=edge, lw=lw))


def box(x, y, w, h, title, lines=(), footer=None, subtitle=None):
    panel(x, y, w, h, "white", BOX_EDGE, lw=1.3)
    top = y + h
    ax.text(x + 0.25, top - 0.38, title, fontsize=12.5, fontweight="bold",
            color=INK, va="center")
    if subtitle:
        ax.text(x + 0.25, top - 0.85, subtitle, fontsize=9.5, color=MUTED, va="center")
    for i, line in enumerate(lines):
        ax.text(x + 0.25, top - 0.88 - i * 0.40, "•  " + line, fontsize=9.5,
                color=INK, va="center")
    if footer:
        ax.text(x + 0.25, y + 0.32, footer, fontsize=8.5, fontweight="bold",
                color=INK, va="center")


def arrow(x0, y0, x1, y1, color=GREY):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>", mutation_scale=16,
                                 color=color, lw=1.8, shrinkA=0, shrinkB=0))


def label(x, y, text, ha="left"):
    ax.text(x, y, text, fontsize=8.5, fontweight="bold", color=INK, ha=ha, va="center",
            linespacing=1.3)


# ---- Laptop -------------------------------------------------------------
panel(0.3, 8.2, 15.4, 4.0, LAPTOP_FILL, LAPTOP_EDGE)
ax.text(0.6, 11.85, "Laptop: Drone Terminal", fontsize=13.5, fontweight="bold",
        color=INK, va="center")
ax.text(4.75, 11.85, "drone_terminal/ menu; opens tag_stream.py alongside it",
        fontsize=9.5, color=MUTED, va="center")

box(0.8, 8.6, 6.3, 2.95, "Drone Terminal (app.py + flight.py)", [
    "Numbered menu: connect, take off, fly, hold, land",
    "Loads and checks the route CSV, then uploads it",
    "Sends take-off, start, hold and land commands",
    "Checks telemetry before each step",
], footer="Route and mission control")

box(8.9, 8.6, 6.3, 2.95, "tag_stream.py", [
    "Shows the live camera feed with detected tags",
    "Detects AprilTags and estimates their poses",
    "Matches each detection with the drone pose",
    "Updates a persistent map of tag IDs and positions",
], footer="Vision and tag mapping")

arrow(7.1, 10.25, 8.9, 10.25)
label(8.0, 10.62, "drone pose\n(UDP 5010)", ha="center")

# ---- Wi-Fi gap ----------------------------------------------------------
ax.add_patch(FancyBboxPatch((11.1, 7.32), 4.1, 0.5, boxstyle="round,pad=0,rounding_size=0.25",
                            fc="#e9f0fd", ec=BOX_EDGE, lw=1.2))
ax.text(13.15, 7.57, "←  all three links: Wi-Fi (UDP)", fontsize=9, fontweight="bold",
        color=INK, ha="center", va="center")

# ---- Drone --------------------------------------------------------------
panel(0.3, 0.3, 15.4, 6.6, DRONE_FILL, DRONE_EDGE)
ax.text(0.6, 6.6, "Drone", fontsize=13.5, fontweight="bold", color=INK, va="center")

box(0.8, 3.95, 9.0, 2.35, "ESP32 companion", [
    "Plays the uploaded trajectory on its own timing loop",
    "Sends position + velocity setpoints to the flight controller",
    "Reads the ToF range sensors over I2C",
    "Relays telemetry and streams camera frames to the laptop",
])

box(10.8, 5.0, 4.4, 1.3, "Forward camera", subtitle="Tilted 45° down, 320 × 240 grayscale")
box(10.8, 3.35, 4.4, 1.3, "ToF sensors", subtitle="8× VL53L5CX via I2C mux")

box(0.8, 0.7, 9.0, 2.3, "Flight controller (PX4)", [
    "Estimates position, velocity and attitude (EKF2)",
    "Runs the position, attitude and rate control loops",
    "Mixes thrust and torques into a command for each motor",
])

box(10.8, 1.9, 4.4, 1.1, "PX4 sensors", subtitle="IMU and onboard flight sensors")
box(10.8, 0.6, 4.4, 1.1, "ESCs + motors", subtitle="4 brushless motors")

# Laptop <-> ESP32 over Wi-Fi
arrow(2.2, 8.6, 2.2, 6.3, color=BLUE)
label(2.4, 7.55, "route, take-off,\nstart, hold, land")
arrow(5.6, 6.3, 5.6, 8.6)
label(5.8, 7.55, "telemetry: position,\nheading, flight state")
arrow(9.4, 6.3, 9.4, 8.6)
label(9.2, 7.55, "camera frames\n(JPEG)", ha="right")

# Sensors -> ESP32
arrow(10.8, 5.65, 9.8, 5.65)
label(10.3, 5.95, "images", ha="center")
arrow(10.8, 4.3, 9.8, 4.3)
label(10.3, 4.6, "ranges", ha="center")

# ESP32 <-> PX4 over MAVLink
arrow(2.2, 3.95, 2.2, 3.0, color=BLUE)
label(2.4, 3.47, "position + velocity\nsetpoints")
arrow(5.6, 3.0, 5.6, 3.95)
label(5.8, 3.47, "position, heading,\nflight state")
ax.text(9.6, 3.47, "wired MAVLink (UART)", fontsize=8.5, color=MUTED, ha="right",
        va="center")

# PX4 sensors / motors
arrow(10.8, 2.45, 9.8, 2.45)
arrow(9.8, 1.15, 10.8, 1.15)

fig.savefig(OUT, dpi=300, facecolor="white")
print(f"Wrote {OUT}")
