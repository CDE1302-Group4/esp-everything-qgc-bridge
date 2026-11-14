#!/usr/bin/env python3
"""Generate a level, inward rectangular-spiral trajectory CSV.

The start is the centre of the near edge: x=0, y=0. The scan area extends
forward along +x for ``length`` metres and spans ``width`` metres along y.
Output uses the trajectory uploader's t,x,y,z format (NED metres).
"""

import argparse
import csv
import math
from pathlib import Path


def rectangular_spiral(length: float, width: float, spacing: float):
    """Return waypoints for an inward rectangular spiral within the area."""
    half_width = width / 2
    points = [(0.0, 0.0)]
    x_min, x_max = 0.0, length
    y_min, y_max = -half_width, half_width

    # Reach the far edge, then wind inward by one track spacing each lap.
    points.extend([(x_max, 0.0), (x_max, y_max)])
    while True:
        next_x_min = x_min + spacing
        if next_x_min >= x_max:
            break
        points.append((next_x_min, y_max))
        next_y_min = y_min
        if next_y_min < y_min + spacing:
            next_y_min = y_min + spacing
        if next_y_min >= y_max:
            break
        points.append((next_x_min, next_y_min))

        next_x_max = x_max - spacing
        if next_x_max <= next_x_min:
            break
        points.append((next_x_max, next_y_min))

        next_y_max = y_max - spacing
        if next_y_max <= next_y_min:
            break
        points.append((next_x_max, next_y_max))

        x_min, x_max = next_x_min, next_x_max
        y_min, y_max = next_y_min, next_y_max

    # Avoid zero-length legs for very narrow areas or small spacing values.
    return [p for i, p in enumerate(points)
            if i == 0 or math.dist(p, points[i - 1]) > 1e-9]


def sample_path(points, dt: float, speed: float):
    """Sample each leg with a smooth start/stop profile and constant dt."""
    rows = [(0.0, 0.0, 0.0, 0.0)]
    t = 0.0
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        distance = math.hypot(x1 - x0, y1 - y0)
        duration = max(dt, 1.5 * distance / speed)
        steps = max(1, math.ceil(duration / dt))
        leg_duration = steps * dt
        for i in range(1, steps + 1):
            u = i / steps
            eased = u * u * (3 - 2 * u)
            rows.append((t + i * dt,
                         x0 + (x1 - x0) * eased,
                         y0 + (y1 - y0) * eased,
                         0.0))
        t += leg_duration
    return rows


def main():
    default_dir = Path(__file__).resolve().parent.parent / "trajectory"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=float, required=True,
                        help="area width in metres (across y)")
    parser.add_argument("--length", type=float, required=True,
                        help="area length in metres (forward along +x)")
    parser.add_argument("--spacing", type=float, default=0.25,
                        help="spacing between spiral tracks in metres (default: 0.25)")
    parser.add_argument("--speed", type=float, default=0.15,
                        help="maximum approximate speed in m/s (default: 0.15)")
    parser.add_argument("--dt", type=float, default=0.2,
                        help="CSV sample interval in seconds (default: 0.05)")
    parser.add_argument("--output", type=Path, default=None,
                        help="output CSV (default: ../trajectory/rect_spiral_<L>x<W>.csv)")
    args = parser.parse_args()

    for name, value in (("width", args.width), ("length", args.length),
                        ("spacing", args.spacing), ("speed", args.speed), ("dt", args.dt)):
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name} must be a finite positive number")

    points = rectangular_spiral(args.length, args.width, args.spacing)
    rows = sample_path(points, args.dt, args.speed)
    output = args.output or default_dir / (
        f"rect_spiral_{args.length:g}x{args.width:g}.csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(("t", "x", "y", "z"))
        writer.writerow(("0", "0", "0", "0"))
        writer.writerows((f"{t:.2f}", f"{x:.6f}", f"{y:.6f}", "0")
                         for t, x, y, _z in rows[1:])
    print(f"Wrote {len(rows)} points to {output}")
    print(f"Area: {args.length:g} m forward x {args.width:g} m wide; "
          f"duration {rows[-1][0]:.2f} s; all z=0")


if __name__ == "__main__":
    main()
