# Drone Terminal

Numbered terminal UI for the existing ESP32/PX4 drone stack. It is
self-contained: the flight runner (`flight.py`), UDP link (`transport.py`) and
trajectory generator (`generate_flying_area_search.py`) live in this folder.
It opens the original `laptop/tag_stream.py` process for camera detection and
permanent mapping.

Run from the project root after activating the laptop Python environment:

```powershell
python -m drone_terminal
```

The trajectory picker reads CSV files from `drone_terminal/../trajectory`.
It infers the upload interval from the first two CSV timestamps.

Both trajectory formats are accepted:

- `t,x,y,z` uses the legacy fixed-heading playback.
- `t,x,y,z,yaw_deg` uses explicit absolute NED yaw (`0` = North/+x,
  clockwise positive). The terminal converts degrees to radians, validates the
  yaw rate/acceleration, uploads the yaw packet format, and waits while the
  drone aligns to the first heading before playback begins.

The map option starts `laptop/tag_stream.py` with telemetry mirror port 5010.
The terminal controller owns telemetry port 5005 and mirrors packets to the
map. Do not run another mission controller or direct telemetry listener at the
same time. The telemetry-log option opens a second console on Windows and is
also opened automatically when TAKEOFF or FLY begins.


## Trajectory simulation (no drone needed)

Menu option 12 opens a real-time 2D simulation of the selected CSV, or run it
directly:

```powershell
python -m drone_terminal.trajectory_sim trajectory/flying_area_search_5p4x4p5_yaw_center_slow.csv
python -m drone_terminal.trajectory_sim --speed 4 --start-heading 90
python -m drone_terminal.trajectory_sim --wind 2 --seed 7    # rougher air
python -m drone_terminal.trajectory_sim --ideal              # no disturbances
```

It uses the same loader and sample interval as the upload, so a CSV the drone
would refuse fails here too. A simulated drone then flies the route the way
the firmware plays it:

- It hovers for 3 s after take-off to settle (grey), then START is sent.
  The route is offset from where PX4 *thinks* the drone is at that moment.
- `t,x,y,z`: the setpoint heading is the heading at START.
- `t,x,y,z,yaw_deg`: the drone turns on the spot to the first yaw (red) and
  the playback clock starts once the yaw error is under 0.1 rad.
- Setpoints refresh at the firmware's 10 Hz nav tick, with a short link delay.
- At the end it holds the last point until it has stayed within 0.15 m for
  1 s (green), the flight runner's landing condition.

The drone is flown by a PX4-style controller (position P, velocity PI,
tilt-limited thrust with attitude lag) and is pushed off the route by
turbulence (stronger near walls), a steady draft, motor vibration, drag,
optical-flow drift in the position estimate, and yaw disturbances. Walls are
solid: contacts are marked with a red X (ToF collision avoidance is disabled
in the firmware). The constants are typical of a small indoor quad, not
measured on our drone, so use it to see what kind of errors to expect.

On screen: orange is the true flown path, purple is PX4's estimate, the cyan
circle is the current setpoint, and the yellow line shows which way and how
hard the drone is tilting. The strip below plots the distance from the
planned route over time; the same summary is printed to the console.

The view is top-down with North (x) to the right and East (y) down, so a
clockwise yaw looks clockwise. Keys: Space pause, R restart, N new random
flight, D disturbances on/off, +/- speed, L loop, Q close.
