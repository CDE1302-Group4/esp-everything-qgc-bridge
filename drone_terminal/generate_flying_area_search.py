#!/usr/bin/env python3
"""Create a slow, multi-angle search trajectory for the Studio 1 flying area.

Coordinates are relative to the launch/landing-zone centre: +x is the 5.4 m
length direction and y spans the 4.5 m width. The route raster-scans the area,
circles all six illustrated candidate AprilTag positions, then returns home.
"""

import csv
import math
from pathlib import Path


DT = 0.10  # seconds; use send_trajectory.py --dt 0.10 to retain this point count
MAX_POINTS = 2400
# Written to trajectory/, where the Drone Terminal route picker looks.
OUTPUT = (Path(__file__).resolve().parent.parent / "trajectory"
          / "flying_area_search_5p4x4p5_under200s.csv")

# Flight boundary relative to the green launch-zone centre. The 0.2-0.25 m
# inset leaves a little room at the drawn 5.4 x 4.5 m boundary.
X_MIN, X_MAX = -0.20, 4.75
Y_MIN, Y_MAX = -2.00, 2.00


def build_route():
    """Return spatial samples and speed caps along a continuous route."""
    points = [(0.0, 0.0)]
    caps = [0.16]

    def line(end, speed, spacing=0.05):
        x0, y0 = points[-1]
        x1, y1 = end
        distance = math.hypot(x1 - x0, y1 - y0)
        steps = max(1, math.ceil(distance / spacing))
        for i in range(1, steps + 1):
            u = i / steps
            points.append((x0 + (x1 - x0) * u, y0 + (y1 - y0) * u))
            caps.append(speed)

    def arc(cx, cy, radius, a0, a1, speed=0.24):
        arc_len = abs(a1 - a0) * radius
        steps = max(2, math.ceil(arc_len / 0.05))
        # The first arc point is connected from the previous route segment.
        first = (cx + radius * math.cos(a0), cy + radius * math.sin(a0))
        line(first, 0.30)
        for i in range(1, steps + 1):
            a = a0 + (a1 - a0) * i / steps
            points.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
            caps.append(speed)

    # Five long raster lanes cover the 4.95 m usable length at 1 m spacing.
    # Turn connectors have a lower cap; the speed planner also ramps before
    # and after each corner.
    line((X_MIN, -2.0), 0.20)
    lane_y = (-2.0, -1.0, 0.0, 1.0, 2.0)
    for row, y in enumerate(lane_y):
        line((X_MAX if row % 2 == 0 else X_MIN, y), 0.30)
        if row + 1 < len(lane_y):
            line((X_MAX if row % 2 == 0 else X_MIN, lane_y[row + 1]), 0.18)

    # The six illustrated candidate positions. A slow western half-circle at
    # each gives several forward-camera viewpoints while keeping travel short.
    candidate_x = (0.99, 2.25, 3.51)
    candidate_y = (1.35, -0.90)
    radius = 0.45
    for cy in candidate_y:
        xs = candidate_x if cy > 0 else tuple(reversed(candidate_x))
        for cx in xs:
            # North point, around the west side, to the south point.
            arc(cx, cy, radius, math.pi / 2, 3 * math.pi / 2, speed=0.24)

    line((0.0, 0.0), 0.20)
    return points, caps


def sample_route(points, caps):
    """Time-parameterize the path with acceleration and corner speed limits."""
    distances = [0.0]
    for a, b in zip(points, points[1:]):
        distances.append(distances[-1] + math.dist(a, b))

    speeds = list(caps)
    # Slow at sharp corners, then ramp smoothly under a forward/backward
    # acceleration bound. Curved tag scans are sampled densely and stay smooth.
    for i in range(1, len(points) - 1):
        ax, ay = points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]
        bx, by = points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1]
        na, nb = math.hypot(ax, ay), math.hypot(bx, by)
        if na and nb:
            turn = math.acos(max(-1.0, min(1.0, (ax * bx + ay * by) / (na * nb))))
            if turn > math.radians(12):
                speeds[i] = min(speeds[i], 0.16)

    accel = 0.45  # m/s^2
    speeds[0] = 0.0
    speeds[-1] = 0.0
    for i in range(1, len(speeds)):
        ds = distances[i] - distances[i - 1]
        speeds[i] = min(speeds[i], math.sqrt(speeds[i - 1] ** 2 + 2 * accel * ds))
    for i in range(len(speeds) - 2, -1, -1):
        ds = distances[i + 1] - distances[i]
        speeds[i] = min(speeds[i], math.sqrt(speeds[i + 1] ** 2 + 2 * accel * ds))

    times = [0.0]
    for i in range(1, len(points)):
        ds = distances[i] - distances[i - 1]
        avg = (speeds[i - 1] + speeds[i]) / 2
        if avg < 1e-6:
            avg = max(0.05, min(caps[i - 1], caps[i]))
        times.append(times[-1] + ds / avg)

    if times[-1] > 200:
        raise SystemExit(f"Route duration {times[-1]:.1f} s exceeds the 200 s limit")

    rows = []
    segment = 0
    count = math.floor(times[-1] / DT)
    for n in range(count + 1):
        t = n * DT
        while segment + 1 < len(times) - 1 and times[segment + 1] < t:
            segment += 1
        span = times[segment + 1] - times[segment]
        u = 0.0 if span <= 0 else (t - times[segment]) / span
        x = points[segment][0] + u * (points[segment + 1][0] - points[segment][0])
        y = points[segment][1] + u * (points[segment + 1][1] - points[segment][1])
        rows.append((t, x, y, 0.0))
    rows.append((times[-1], points[-1][0], points[-1][1], 0.0))
    return rows


def main():
    points, caps = build_route()
    rows = sample_route(points, caps)
    if len(rows) > MAX_POINTS:
        raise SystemExit(
            f"Generated {len(rows)} points, above the {MAX_POINTS} point limit; "
            "increase DT and match it with send_trajectory.py --dt."
        )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(("t", "x", "y", "z"))
        writer.writerow(("0", "0", "0", "0"))
        writer.writerows((f"{t:.2f}", f"{x:.6f}", f"{y:.6f}", "0")
                         for t, x, y, _ in rows[1:])

    max_speed = max(math.dist(a[1:3], b[1:3]) / DT
                    for a, b in zip(rows, rows[1:]))
    print(f"Wrote {len(rows)} points: {OUTPUT}")
    print(f"Duration {rows[-1][0]:.1f} s; sampled max speed {max_speed:.3f} m/s")
    print("Starts and ends at (0,0,0); all z values are zero.")


if __name__ == "__main__":
    main()
