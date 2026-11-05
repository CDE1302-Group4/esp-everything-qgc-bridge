# Trajectories

CSV files use `t,x,y,z` (seconds and NED metres). The uploader makes each path
relative to the hover position and holds the drone's starting heading.

## Minimum-snap ellipse

- 2.00 m North-South major axis × 1.00 m East-West minor axis
- 34 s clockwise lap; approximately 0.300 m/s peak speed
- Starts and finishes at rest at the +North major-axis tip

Place the drone at that tip with its nose outward in **+North / +x**. The centre
is 1.00 m behind it in **-North / -x**, and the path extends 0.50 m to either
side. Confirm the axes in QGC and allow extra safety clearance.

Run `ellipse_min_snap_demo.m` in MATLAB to regenerate the CSV and placement
guide. Set `run_simulation = true` for the optional Simulink test.

```bash
python3 laptop/send_trajectory.py --drone-id 22 --takeoff --takeoff-wait 12 --confirm \
    --trajectory trajectory/ellipse_min_snap_traj.csv
```

## Circle

Run `circle_traj_demo.m` in MATLAB to regenerate the default clockwise circle:
2 m diameter, approximately 21 s, starting toward `+y`. From takeoff it spans
`x = 0..-2 m` and `y = -1..+1 m`.

```bash
python3 laptop/send_trajectory.py --drone-id 22 --takeoff --takeoff-wait 12 --confirm \
    --trajectory trajectory/circle_traj.csv
```

## Slow 2 m by 1 m rectangular spiral

`rectangle_spiral_2x1_slow.csv` starts at the centre of the near 1 m edge,
with the drone facing into the rectangle along **NED +x (North)**. The uploader
holds that heading throughout; it does not rotate the path to match the nose.
The rectangle extends 2 m forward (`x = 0..2`) and 0.5 m to either side
(`y = -0.5..+0.5`). See [the placement and path preview](rectangle_spiral_2x1_preview.png).

The drone climbs 0.5 m above its starting hover, waits 2 seconds, then follows
an inward rectangular spiral with 25 cm spacing. With the firmware's current
0.5 m takeoff hover, the scan altitude is approximately **1.0 m above takeoff**.
The CSV begins at `z = 0` and climbs to `z = -0.5`, since the uploader subtracts
the first position and NED uses negative z for upward motion.

Horizontal speed is at most **0.15 m/s**, with smooth velocity ramps and a stop
at each corner; climb speed is at most 0.10 m/s. The complete path takes
**109.45 seconds**, including the climb and holds, and uses **2,190 points** at
the default 0.05 s upload interval (below the 2,400-point limit).
It finishes at `(x, y) = (1.5, 0)` at scan altitude with a 3-second hold.
The sender then lands by default; `--finish hold` keeps it hovering there.
Coverage between the 25 cm tracks depends on the camera's usable footprint.

```bash
python3 laptop/send_trajectory.py --drone-id 22 --takeoff --takeoff-wait 12 --confirm \
    --trajectory trajectory/rectangle_spiral_2x1_slow.csv
```

Keep generated paths at or below 0.30 m/s. Circle and ellipse regeneration needs MATLAB; the
optional simulations also require Simulink and the model's toolboxes.
