# Drone Terminal

Numbered terminal UI for the existing ESP32/PX4 drone stack. It reuses the
validated `drone_app` flight runner and opens the original
`laptop/tag_stream.py` process for camera detection and permanent mapping.

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

