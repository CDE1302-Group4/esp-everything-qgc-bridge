# Drone Desk

A separate desktop application for the existing ESP32 drone firmware. All app
code, tests, documentation and recordings live in `drone_app/`. The existing
`laptop/` scripts and their command-line workflows are unchanged.

The app imports the existing packet definitions, camera frame assembler and
AprilTag detector as read-only helpers. Its GUI, connection lifecycle, maps and
cancellable flight runner are separate implementations in this folder.
The detector's compiled library is cached separately in `drone_app/.cache/`;
the existing viewer's detector cache is not changed. An existing compiled
library is copied into the private cache when available. If a rebuild is needed,
it uses the same C compiler as the existing AprilTag viewer.

## Run

From the repository root, activate the existing Python environment:

```powershell
conda activate cde1302-laptop
python drone_app/app.py
```

The default selection is drone **4**, IP **192.168.1.204**. Both can be changed
before connecting. To select another drone from the command line:

```powershell
python drone_app/app.py --drone-id 22
```

No new GUI framework needs installing: this uses Tkinter and Matplotlib. Pillow
is already installed with Matplotlib. For a fresh environment, use:

```powershell
python -m pip install -r drone_app/requirements.txt
```

Preview the whole interface with simulated camera, tags and telemetry:

```powershell
python drone_app/app.py --demo
```

Demo mode opens no network sockets and disables all flight commands. Its banner
and camera explicitly identify the data as simulated.

## Use

1. Close standalone `tag_stream.py`, `send_trajectory.py`, mission scripts and
   other programs using telemetry UDP 5005 or camera UDP 5009. Drone Desk owns
   one shared connection; it replaces those terminals while it is open.
2. Check the drone ID/IP and click **Connect**. Connecting requests camera video
   and listens for telemetry; it does not arm or launch the drone. Fleet
   configuration is read from `laptop/setup.yaml`, as in the existing sender.
   The laptop must still have the IP configured as the firmware's telemetry host.
3. Choose an included CSV or **Browse CSV**, select the sample interval, and
   click **Preview / validate**. The sidebar shows duration, peak speed, vertical
   offsets and the actual number of uploaded points. The dashed map line shows
   the planned NED route. Sample interval 0.20 s uses fewer points than 0.05 s.
4. Select whether to take off before playback, the takeoff wait and the final
   action (**hold** or **land**). **Fly trajectory** asks for confirmation naming
   the exact drone, route and finish action before sending flight commands.
   **Take off only** requests takeoff without a route, then leaves the drone
   hovering. After it completes, the takeoff-before-trajectory option is cleared.
5. **HOLD** cancels the current upload/playback operation and requests position
   hold. **LAND** cancels it and requests landing. These controls share a lock
   with the runner, preventing a queued START after a stop request. An interrupted
   route is cancelled; there is no resume operation.

The camera and telemetry run in background workers. Flight monitoring also
runs in a worker, so plotting and camera rendering do not perform flight waits
on the UI thread. A stale connection disables new takeoff/playback operations;
Hold/Land remain available to attempt a command to the selected known drone.

CSV axes are fixed local NED, not rotated to follow the nose. Face **NED +x
(North)** when the route describes forward as +x. The map axes are forward/right
relative to the map origin; the CSV preview is rotated into those axes. Use the
plot toolbar for pan and zoom.

The maps use most of the main panel. Drag the horizontal dividers to resize
them, or click **Expand maps** to fill the main panel; **Restore layout** brings
back the camera, status and log. Flight controls remain in the sidebar.
Click a tag marker or its label on the permanent map to show **X (right)** and
**Y (forward)** in metres relative to the map origin. The coordinates update
as sightings refine the tag position. Click empty map space to dismiss them;
turn off toolbar pan/zoom before selecting a tag. The compact trajectory key
sits above the plot, including the CSV preview.

## Existing firmware

No reflash is required. Original 55-byte telemetry supplies XY position,
heading and navigation state. Height and armed/mode fields show **Unavailable**.
Legacy takeoff uses the selected timed wait (default 12 seconds), checking fresh
telemetry throughout. It cannot prove takeoff or verify vertical motion: use
QGC or visual observation for those. Playback must be acknowledged as NAV_TRAJ,
then NAV_ARRIVED, and the horizontal endpoint must settle within 0.15 m.

If TF01 telemetry is available, the flight runner additionally checks fresh PX4
position/heartbeat, armed OFFBOARD mode, settled takeoff and the 3D endpoint.
It refuses a telemetry format change during an operation. Unexpected failures
after taking control request landing; explicit Hold/Land supersedes this cleanup.
Commands are requests, not proof of physical touchdown.

Tag localization uses the existing camera calibration/mount and horizontal
position plus yaw; it assumes level flight. Tags remain after leaving view.
For old telemetry, estimated preflight drift and configuration offsets can affect
the map. **Reset map origin** clears the map at the latest position. TF01 map
behavior follows the existing viewer's disarmed/armed handling.

## Save and close

**Save maps + camera** (or Ctrl+S) writes a timestamped folder under
`drone_app/recordings/` containing `maps.png`, `camera.png`, `map.json` and
`log.txt`. Map JSON includes the origin, persistent tag coordinates and estimated
drone path. Recorded demo data is marked as demo. Recordings are ignored by Git.

Disconnect and window close also save the current session. After the app has
controlled a drone, closing asks whether to request **Land**, request **Hold**,
or stay connected. Closing without ever starting flight sends no flight command.

## Validation

```powershell
python -m pytest drone_app/tests -q
python drone_app/app.py --demo --smoke-test
```

The smoke check keeps the window hidden, feeds synthetic data for a few seconds,
and closes it. It sends no flight commands. Host tests and demo checks do not
replace testing camera reception and flight on the real drone.
