# Arm, takeoff, hover and land test

`laptop/simple_arm_takeoff_land.py`: CMD_START → hover → CMD_HOLD → CMD_LAND.
Run from the repo root. Examples use drone **22**; use your flashed ID.

## Setup

- Flash with `./flash_drone.sh <id> <port>`. The ESP32 gets `192.168.1.(200+id)`
  and sends telemetry only to `192.168.1.(100+id)`, so the laptop must hold that
  address on the drone Wi-Fi (README, "Laptop IP address").
- Python: README section 2.
- Serial monitor: `idf.py -p <port> monitor`. Open it before arming (it resets
  the ESP32). Wait for `Waiting for CMD_START from laptop...`.
- QGroundControl: observe only.
- Optional link check (close QGC first): `python3 laptop/check_px4_esp32_link.py --drone-id 22` → `RESULT: PASS ...`.
- `camera_stream.py` / `tag_stream.py` can run alongside (use `--fps 5` in
  flight). Tags never affect the flight. Start `tag_debug.py` only after the
  flight script, because it takes UDP 5005.

## Commands

| Command | ESP32 behaviour |
|---------|-----------------|
| `CMD_START` | Accepted once per flight, from `Pre-streaming hold setpoint...` on; otherwise ignored (`CMD_START ignored — not waiting for it`). OFFBOARD → arm (gives up after 10 s) → climb to 0.5 m. |
| `CMD_HOLD` | Stops an active goal/trajectory; no effect otherwise. |
| `CMD_LAND` | Any time after START: aborts OFFBOARD/arming, or lands (also mid-climb). Ignored before START. |
| `CMD_GOTO`, trajectory start | Only after takeoff completes; otherwise ignored (`... ignored — not flying`). |

## 1. Communication check

```bash
python3 laptop/simple_arm_takeoff_land.py --drone-id 22 --monitor-only
```

Expect `Connected: drone=22, ESP32_IP=192.168.1.222, ...`, then 15 s of
`MONITOR | ... | nav=IDLE | age=...` with `age` well below 1 s. No flight commands are sent.

If it fails with `No telemetry from drone 22 on UDP 5005 (heard drones: [...])`:
another ID listed → wrong `--drone-id`; `none` → check the laptop IP and firewall.

## 2. Props-off test, then flight

```bash
python3 laptop/simple_arm_takeoff_land.py --drone-id 22
```

Type `ARM-22`. The script sends CMD_START, waits `--takeoff-wait` (12 s), sends
CMD_HOLD, waits `--hover-time` (5 s), sends CMD_LAND and exits.

Key ESP32 lines (others interleave):

```text
mission: CMD_START received
mission: ToF disabled (TOF_ENABLED=0) — skipping pre-arm sensor check
mission: OFFBOARD mode confirmed
mission: Armed confirmed
mission: Taking off to 0.5 m AGL (NED z=-0.50)...
mission: Altitude reached: NED z=-0.3x (target=-0.50)
mission: Exploration mode — waiting for laptop goals...
wifi: CMD_HOLD
wifi: CMD_LAND
mission: CMD_LAND received from laptop
mission: Land command sent
mission: Disarmed — mission complete
mission: Mission loop complete — waiting for next CMD_START
```

- Props off: `Takeoff timeout: NED z=0.0x ...` replaces `Altitude reached` (expected).
- `Takeoff aborted: OFFBOARD refused (check QGC)` / `arming refused (check QGC)`:
  PX4 rejected it. Fix the cause in QGC and run again.
- `Still armed after 20 s — land with RC`: take over and land with the RC.

In flight, watch QGC: Offboard → armed → ~0.5 m → steady hover → Land → disarmed.

## 3. Emergency

- **RC pilot first:** switch out of Offboard or use the kill switch. The ESP32
  never overrides this. After an RC landing it logs `Disarmed without CMD_LAND`
  and is ready for the next START.
- **Ctrl+C** after START sends CMD_LAND (3×): it aborts arming, or lands the
  drone, even mid-climb. Stale telemetry (3 s) does the same.
- If the script or laptop dies without sending LAND, the drone keeps hovering: use the RC.
- **Wi-Fi killswitch:** if the ESP32 loses the access point it holds position,
  and after 3 s sends a non-forced disarm. PX4 normally refuses this in the air
  (`Disarming denied: not landed`), so take over with the RC.

## 4. Repeat

No reboot needed. Wait for `Waiting for CMD_START from laptop...` and run again.

## Limitations

- Telemetry has no altitude, armed state or flight mode: use QGC and the serial monitor.
- Commands are not acknowledged: check `wifi: CMD_...` in the serial monitor.
- A START sent too early is ignored. The script can't tell, so check the serial monitor.
- `tag_id` / `tag_dist_m` in telemetry are info only, latched until the ESP32 reboots.

## AprilTag exploration mission

`simple_exploration_mission.py` extends the flight sequence in
`simple_waypoint_mission.py` (the repository's waypoint base). It takes off,
flies a lawnmower search, records live AprilTag observations, retraces reached
waypoints to the recorded starting position, and requests landing there.
The camera/detector runs in a separate thread throughout the mission, including
takeoff and return. No prior target-tag locations are needed.

Run these commands from the **laptop directory**, using the project environment:

```bash
conda activate cde1302-laptop

# Review a small example search without connecting or sending commands.
python simple_exploration_mission.py --drone-id 22 \
    --bounds 0 2 -1 1 --lane-spacing 0.5 --dry-run

# Place the UAV at its assigned start, then initiate from the laptop.
python simple_exploration_mission.py --drone-id 22 \
    --bounds 0 2 -1 1 --lane-spacing 0.5 \
    --arrival-timeout 60 --max-search-time 300 \
    --log exploration_22.csv --confirm
```

Replace the example bounds with the assigned search area. `--bounds` is
`XMIN XMAX YMIN YMAX` in metres **relative to the recorded start**, along
local NED X/Y (north/east), not forward/right relative to the UAV. The script
clears the firmware's arena offset, so telemetry and commands use PX4 local
coordinates; it does not load `setup.yaml`. It records the actual pre-takeoff
XY and adds that position to the planned offsets, including after a previous
flight where the starting XY is no longer zero. Physically place the vehicle
at the assigned starting position before invoking it.

Search rows run along X, with Y spacing no greater than `--lane-spacing`.
Choose spacing and bounds for the camera's effective tag footprint at flight
height; the default is not a guarantee of visual coverage. The camera points
forward and 45 degrees down. Allow clearance for the vehicle and turns:
the rectangle constrains planned goals, not the complete obstacle-avoidance
flight path. The transit from the start to the rectangle must also be clear.
There is a one-second detection dwell at each goal (`--dwell`). The mission
finishes after visiting all goals, even if it detects no tags.

Close other flight controllers and camera viewers before running: this process
owns telemetry UDP 5005 and camera UDP 5009, and learns the ESP IP from the
selected drone's telemetry. It requires a decoded camera frame before START.
The host detector uses the existing in-tree C sources and needs a C compiler
on first run, just like `tag_stream.py`. The existing family detects IDs 0–21;
tag size, camera intrinsics, and camera mount must match `apriltag_host.py`
(currently a 0.12 m tag and 320×240 image calibration).

Exploration records default to a timestamped `exploration_*.csv` in the working
directory. `--log name.csv` selects CSV, and `--log name.jsonl` selects JSONL.
Records are appended to a new file and flushed after each record; an existing
file is never overwritten. CSV contains only the tag number and local
coordinates. For detailed recording, JSONL includes the following for each
decoded detection in a processed frame:

- Tag ID/family, quality metrics and acceptance/rejection reason, frame ID,
  camera boot nonce, and mission phase.
- Image centre/corners in pixels and camera-to-tag range when a valid pose exists.
- UTC record time, UTC/monotonic image receipt time, and camera processing age.
- Camera-relative and body-relative translations in metres and rotations as
  nine-element row-major matrices (tag axes expressed in the named frame).
- Estimated tag position and orientation in local NED, UAV XY/heading, and
  the telemetry/image receipt-time difference used for localization.

CSV has just four columns, with one row per confirmed tag observation:

```csv
tag_id,local_x_m,local_y_m,local_z_m
5,1.2,0.7,
```

Coordinates are estimated positions in the UAV's local NED frame, in metres.
Z is normally blank unless `--assumed-altitude` is supplied. Observations without
a valid local XY estimate are excluded from CSV. Repeated confirmed sightings
remain separate rows. Mission events, timestamps,
relative poses, image coordinates, and quality fields are omitted from CSV.
JSONL retains those details and mission events in the original nested format.

Both mission scripts use `CMD_GOTO`. With the updated ESP32 firmware, waypoint
commands preserve heading instead of turning toward the next point, including
on arrival and return. PX4 uses pitch/roll and thrust to move forward, backward,
or sideways as needed while stabilizing yaw. The lawnmower route can therefore
reverse travel direction without turning the nose. The camera stays pointed in
the same direction, which must be considered when choosing search coverage.

This behavior is implemented in `main/nav_task.c`, not a Python CLI option.
Rebuild and flash the updated ESP32 firmware for the selected drone ID before
expecting heading hold; running the scripts against older firmware still turns
toward waypoints. Takeoff, landing, and RC takeover behavior are unchanged.
Timed trajectories already hold the heading captured when playback starts.
Heading hold is a command policy, not a guarantee of zero physical yaw motion.

The transform is `tag_local_xy = uav_local_xy + Rz(heading) * tag_body_xyz`,
using the mount transform from `apriltag_host.py`. **Roll and pitch are assumed
zero** because the UDP telemetry contains only XY and yaw. The latest telemetry
received before the image is used, not the UAV position after detection
finishes; matches older than `--pose-max-age` (0.5 s) are rejected. These are
receipt timestamps, not synchronized exposure timestamps, so transport delay
and attitude changes introduce error. Missing/invalid poses produce a rejected
diagnostic record in JSONL, but no CSV row.

Both mission scripts now apply stricter acceptance than the raw detector in
`tag_stream.py`. A saved detection must have zero corrected bits (`hamming=0`),
decision margin above 80, a valid pose in front of the camera, and a valid local
XY estimate. The same ID must pass these checks in three consecutive processed
frames, with local XY within 0.5 m of its first sighting. Missing/rejected
sightings, a camera restart, or a frame gap over 1.5 s reset confirmation.
Duplicate frames cannot confirm a tag; multiple observations of the same ID
in one frame are treated as ambiguous. These defaults reduce false IDs but
can reject real tags seen briefly or with noisy position estimates. They are
not a guarantee against a consistently misdecoded pattern.

Use `--tag-min-margin 95` or `--tag-confirm-frames 4` for stricter acceptance.
Margin is a decoder score, not a confidence percentage. If the tag IDs used
in your search area are known, supply an allowlist, for example `--tag-ids 0 5 12`;
otherwise all supported IDs (0–21) are considered. Both scripts accept these
options. Example (replace the bounds and IDs with your assigned setup):

```bash
python simple_exploration_mission.py --drone-id 15 --bounds 0 2 -1 1 \
    --tag-ids 0 5 12 --log confirmed_tags.csv --confirm
```

JSONL preserves every decoded observation as `detection`, `detection_candidate`,
or `detection_rejected`, with `mission_accepted`, `acceptance_reason`, and
`confirmation_count`. The original `quality_accepted` field describes the
weaker firmware quality gate, not final mission acceptance. CSV includes only
`detection` events. Repeated confirmed sightings are recorded individually.
These changes apply to laptop mission recording immediately; they do not change
the ESP32 detector or the raw observations displayed by `tag_stream.py`.

Local tag Z is null by default because altitude is not transmitted. Optional
`--assumed-altitude 0.5` estimates Z as `body_down - 0.5` during exploration and
return only, explicitly marked as an assumption. It **does not command flight
altitude**. START/GOTO use the firmware's cruise altitude (currently 0.5 m).
Measured full 3D localization would require timestamped XYZ and full attitude
telemetry from the firmware.

`--max-search-time` limits exploration only; budget additional time/battery for
retracing the route and landing. A search waypoint timeout or exhausted search
budget stops further searching and returns via reached goals, with exit status
1 for incomplete exploration. A return timeout, camera/detector/logging failure,
stale telemetry, or Ctrl+C requests LAND in place. LAND is repeated three times.
Successful search and return with LAND sent exits 0; Ctrl+C exits 130.

As with the base waypoint script, START/LAND are not acknowledged, and telemetry
has no altitude, armed status, or landed status. The takeoff wait is configurable
(`--takeoff-wait`, default 12 s); confirm takeoff and final touchdown/disarm in
QGC and the ESP serial monitor. Exit 0 means the route completed and LAND was
sent, not that touchdown was verified. Detection does not cause landing on a tag.

Hardware-free validation:

```bash
python -m pytest -q tests/test_exploration_mission.py
```

## CSV waypoint mission with AprilTag logging

`simple_waypoint_mission.py` now uses the same camera recorder and mission
lifecycle as `simple_exploration_mission.py`. Supply the route yourself instead
of generating it with `lawnmower()`. The default sequence is START → CSV points
in file order → reached points in reverse order → recorded start → LAND.

From the **laptop directory**:

```bash
conda activate cde1302-laptop

# Validate the included example and print its points without connecting.
python simple_waypoint_mission.py --drone-id 22 \
    --waypoints-csv waypoints_example.csv --dry-run

# Fly after placing the UAV at the assigned start.
python simple_waypoint_mission.py --drone-id 22 \
    --waypoints-csv waypoints_example.csv --log waypoints_22.jsonl --confirm
```

CSV format (metres):

```csv
x,y
0.5,0.0
0.5,0.5
0.0,0.5
```

The `x,y` header is optional. Blank lines and whole lines beginning with `#`
are allowed. All rows are validated before connecting: empty routes,
malformed rows, missing coordinates, and non-finite values are rejected.
Points are neither reordered nor resampled. This is a horizontal waypoint
mission using `CMD_GOTO`, not a timed trajectory upload. The `t,x,y,z` CSVs
used by `send_trajectory.py` are rejected rather than ignoring time/altitude.
Flight altitude is set by the firmware (currently 0.5 m).

By default, all points are offsets from the actual recorded starting XY along
local NED north/east. A point `0,0` means that starting position, and all points
use the same origin; they are not incremental offsets from the previous point.
With `--map-frame --config setup.yaml`, points instead specify absolute arena
coordinates. That mode requires the selected drone's start offset in the
configuration. Return still targets the actual position recorded before START.
Tag localization subtracts the configured arena offset so detection records
remain in the UAV's **local** frame in either mode.

The existing `--waypoint x,y` and `--waypoints-file` options remain supported.
A `.csv` passed through `--waypoints-file` uses the CSV reader; other files
retain the legacy `x,y` / `x y` line format. If inline points and a file are
both supplied, inline points are flown first, then file points.

The camera must be working before START; close separate camera viewers first.
Detection, timestamps, local-pose assumptions, and logging follow the
exploration mission described above. Use `--assumed-altitude` only to supply
an assumed cruise altitude for tag Z estimation, not to change flight height.
Waypoint missions retain a timestamped JSONL filename by default; use
`--log waypoints_22.csv` to select the same CSV output as exploration missions.

Defaults are a 12 s takeoff wait, a 60 s timeout per waypoint, a 0.25 m arrival
radius, and a 1 s dwell. `--max-search-time` defaults to 300 s for the outbound
route; return and landing require additional time. A missed waypoint or expired
budget ends the outbound route and returns through reached points. Connection,
camera, or return failures and Ctrl+C after START request landing in place.
The existing telemetry cannot confirm takeoff or touchdown.

The former script landed at the last supplied point and continued past missed
points. Its default now returns to the recorded start and stops the outbound
route at the first missed point. Explicit `--finish-action hold` also returns
to the start, then requests HOLD and exits, stopping camera/telemetry handling;
the user is responsible for subsequent landing.

```bash
python -m pytest -q tests/test_waypoint_csv_mission.py tests/test_exploration_mission.py
```
