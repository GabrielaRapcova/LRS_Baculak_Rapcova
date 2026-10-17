# A1.3 outdoor mission specification

Status: the complete outdoor sequence is implemented: takeoff, square, centre
alignment, one figure-eight lap, return and landing. Full-flight SITL acceptance
and recordings remain. Requirements:
[course A1.3](../assignments/assignment1/03_outdoor_gps_mission.md).
Initial target: ROS 2 Jazzy, MAVROS and ArduPilot SITL in
`../worlds/fei_lrs_outdoor.world`.

## 1. Mission and responsibility

Take off to **5 m above launch**, fly a **5 m square** while facing a ground POI,
move above the POI, fly one continuous **figure-eight with amplitude 3 m** while
facing its tangent, return to launch, land and confirm disarm.

Implement `outdoor_mission_node` in `mission_executor`, with a separate launch file
and YAML. Preserve the indoor executable. ArduPilot performs position/attitude
control; this node generates targets and sequences the mission. The prescribed
open-field mission does not require the mapper, costmap or A* planner.

Use a C++ enum/switch state machine and one numerical geometry module for testing.
FlexBE is not part of this initial design. Avoid a general mission framework.

## 2. Execution model

```text
WAIT_READY -> CAPTURE_REFERENCE -> CHECK_MISSION -> SET_GUIDED -> ARM
 -> TAKEOFF -> STABILIZE -> SQUARE -> MOVE_TO_CENTER -> ALIGN_TANGENT
 -> FIGURE_EIGHT -> SETTLE_CENTER -> RETURN_HOME -> LAND -> DISARM -> DONE
```

- **Readiness:** fresh connected state, valid GPS status and coordinates, at least
  8 satellites (255 means unknown), and fresh pose, velocity, heading and altitude.
  Require these conditions for 3 s before arming.
- **Takeoff/settling:** check service responses and observed GUIDED/armed/airborne
  states. Require position/altitude tolerance, low velocity and a settling dwell.
  Do not send a ground-hold target that opposes the takeoff command.
- **Square:** reach W1, then W2, W3, W4 and W1 again to close it. Relative to the POI,
  the corners are (-2.5,+2.5), (+2.5,+2.5), (+2.5,-2.5), (-2.5,-2.5) in ENU metres.
  Recompute POI bearing from measured position every setpoint tick. Stop at every
  corner; advance only after position, altitude, speed and heading meet their
  tolerances continuously for the settling dwell.
- **Centre:** reach and settle above the POI, then align with the initial tangent
  before starting the figure-eight clock. Hold position and align to 45 degrees
  in mission ENU (the initial tangent for positive omega); require heading settling.
- **After the lap:** hold the centre at flight altitude and the final tangent
  heading until settled, then begin the return.
- **Return/landing:** return to captured launch XY at flight altitude, request LAND,
  stop flight setpoints, and confirm on-ground state before any explicit disarm.

Use a `SingleThreadedExecutor`: telemetry, the 10 Hz state machine, service
responses and the 30 Hz setpoint timer run serially in the default callback group.
Calls are asynchronous; callbacks must stay short and never wait on futures.
No mutex is needed while all state access stays within these callbacks.
Each active phase has a timeout; acknowledgements alone do not prove completion.

## 3. Coordinates and altitude

Use `GeographicLib::LocalCartesian` for WGS84 geodetic ↔ ENU conversion. It is already
available locally and avoids writing our own geographic approximation.

Prefer FCU home as the geodetic origin. If unavailable before arming, use a qualified
fix paired with a fresh local pose. Capture the actual launch separately; home and
local (0,0,0) are not assumed to be the launch point. Record the origin source and
freeze mission references for the run. Accept an expected home update during arming
while still on ground, then establish the relative-altitude baseline before takeoff.
Unexpected reference changes in flight abort the mission.

Configure the POI as east/north offsets from that origin; no hardcoded lat/lon.
Generate geodetic POI/corner coordinates and convert them back to ENU, exercising
both required conversion directions. Verify a fixed transform to MAVROS local:

```text
p_local_xy = local_anchor_xy + R(delta) * p_mission_xy
yaw_local  = yaw_mission + delta
z_command  = launch_local_z + 5 m
```

Verify delta with ground heading/translation checks; use zero only when alignment
is confirmed. This is a fixed reference, not an online calibration subsystem.

Keep three altitude quantities distinct:

| Quantity | Use |
| --- | --- |
| GPS/home ellipsoid height | Geodetic conversion and reporting; not labelled AMSL |
| MAVROS relative altitude | Cross-check against launch baseline + 5 m |
| Local Z | Control at launch_local_z + 5 m |

Use local pose/velocity for tracking, not absolute GPS altitude.

## 4. Position and heading targets

Publish `mavros_msgs/PositionTarget` at 30 Hz to `mavros/setpoint_raw/local`.
Activate position and yaw; ignore velocity, acceleration and yaw_rate initially.
Supply ROS ENU values with the `FRAME_LOCAL_NED` enum: MAVROS handles conversion.
Topic names are relative/remappable; only one mission controller commands a vehicle.

Square/return transfers use interpolated, bounded-speed position targets;
interpolation controls target motion, independently of yaw calculation. Holds
repeat the same position. During the square, compute:

```text
yaw_POI = atan2(poi_n - actual_n, poi_e - actual_e)
```

Inside 0.2 m horizontal distance from the POI, retain the last valid bearing.
At the centre, explicitly switch to tangent heading. Return flight faces travel;
takeoff and landing preserve the preceding heading. No gimbal control is required.

For phi = omega * (now - t_start), evaluate every timer tick:

```text
e = poi_e + A*sin(phi)
n = poi_n + A*sin(phi)*cos(phi)
z = launch_height + 5 m

de/dt = A*omega*cos(phi)
dn/dt = A*omega*cos(2*phi)
yaw_tangent = atan2(dn/dt, de/dt)
```

Run through phi=2*pi, hold the centre and final tangent, settle, then return.
Do not sample the curve into mission
waypoints. Use elapsed ROS time; clock jumps or long telemetry gaps abort rather
than skip ahead. Limit tracking error during transfers and the curve.

Convert North/clockwise heading to East/counter-clockwise yaw:
`yaw_ENU = wrap_pi(pi/2 - heading_rad)`. Verify the FCU heading reference in SITL;
MAVROS supplies fused heading, not a separate raw compass reading. Apply a magnetic
correction only if the configured source actually needs one. Unwrap consecutive
yaw commands with `previous + wrap_pi(new - previous)` and check shortest-turn
behavior across +/-pi in simulation.

## 5. Parameters and envelope

Assignment geometry remains fixed. Agreed simulation starting values are 1 m/s
square/return speed, POI offset (2.5 m east, 2.5 m north), omega=0.2 rad/s,
0.25 m horizontal and 0.2 m altitude
tolerances, speed below 0.2 m/s, heading error within 5 degrees, and a 1 s continuous
settling dwell. Other values below remain proposals. Tune against measured simulation
performance; these are not validated flight settings. Parameters stay fixed during a run.

| Parameter | Value |
| --- | --- |
| altitude / square side / figure-eight amplitude | 5 m / 5 m / 3 m |
| POI east/north offset | 2.5 m / 2.5 m |
| omega / laps | 0.2 rad/s / 1 |
| square / return speed | 1 m/s |
| setpoint rate | 30 Hz (allowed 20–50 Hz) |
| minimum satellites / readiness dwell | 8 / 3 s |
| live telemetry timeout | 2 s; verify against actual topic rates |
| horizontal position / altitude tolerance | 0.25 m / 0.2 m |
| velocity tolerance / settling dwell | 0.2 m/s / 1 s |
| yaw tolerance | 5 degrees; measure error throughout flight |
| maximum speed | 1.5 m/s |
| horizontal / altitude limit from launch | 10 m / 6 m |

Peak figure-eight speed is A*omega*sqrt(2): **0.85 m/s**, with a **31.4 s** lap.
Validate omega against the speed limit and measure achievable yaw/tracking response.

The curve spans **6 × 3 m**, extending 0.5 m beyond the square east/west. Check both
shapes and all transfers against the envelope, including vehicle/tracking margin.
A conservative curve distance bound from launch is:

```text
sqrt((abs(poi_e - launch_e) + A)^2 + (abs(poi_n - launch_n) + A/2)^2)
```

With home=launch and POI=(2.5,2.5), this is 6.80 m before margin; the square's
farthest corner is 7.07 m from launch. Account for home-to-launch
offset. Check measured bounds during flight too. The software envelope does not
replace ArduPilot's geofence or verification that the field is unobstructed.

## 6. Interfaces and failures

| MAVROS input | Type |
| --- | --- |
| `state`, `extended_state` | `State`, `ExtendedState` |
| `global_position/global` | `sensor_msgs/NavSatFix` |
| `global_position/raw/satellites` | `std_msgs/UInt32` |
| `home_position/home` | `HomePosition` |
| `global_position/compass_hdg`, `global_position/rel_alt` | `std_msgs/Float64` |
| `local_position/pose`, `local_position/velocity_local` | `PoseStamped`, `TwistStamped` |

Names are relative to MAVROS; MAVROS-specific message types are from `mavros_msgs`,
pose/twist types from `geometry_msgs`. Use sensor-data-compatible subscriptions;
verify home-topic durability and freshness behavior during integration.
Use asynchronous `set_mode`, `cmd/arming` and `cmd/takeoff` services.

| Event | Response |
| --- | --- |
| Explicit abort, lost navigation quality, tracking/fence violation or phase failure | Stop mission progression; if connected and still controlling GUIDED, stop flight setpoints and request LAND; confirm ground/disarm and report ABORTED |
| Pilot mode change | Stop setpoints; RELEASE_CONTROL; never force GUIDED back |
| Lost connection/stale FCU state | Stop setpoints; report failure; defer to onboard failsafes |

Expose `outdoor_mission/abort` (`std_srvs/Trigger`); acceptance is not landing
completion. No automatic restart or in-air disarm. Expected LAND transitions must
not be mistaken for pilot override. Recheck control ownership before every publish.
Stopping position messages alone is not a hold: FCU position targets can be latched.

## 7. Validation and implementation order

1. Geometry tests: conversions, frame transforms, angles, square/curve equations,
   derivatives, speed and envelope bounds.
2. Telemetry-only node: readiness, origin/launch capture, altitude reporting and
   ground heading/axis verification. No arming.
3. GUIDED/arm/takeoff/settle/return/LAND/disarm and abort in SITL.
4. Square with continuous POI yaw, then centre alignment and timer figure-eight.
5. Failure tests: poor/missing GPS, unknown satellites, stale telemetry, mode override,
   rejected calls, timeouts, clock/reference changes and excessive tracking error.
6. Two complete unattended simulation runs, abort evidence, and commanded/actual
   track and heading-error plots for the defence.

Publish state/reason and bounded commanded/actual `nav_msgs/Path` histories under
`outdoor_mission`. At 1 Hz report geodetic position, mission ENU, launch-relative
height, both heading conventions, GPS quality and state. Record raw telemetry,
setpoints and diagnostics; path histories are visualization only.

Start Gazebo, SITL and MAVROS separately. Match Gazebo georeferencing with SITL home;
those simulator settings do not belong as absolute coordinates in mission code.
Real flights remain subject to the course's teacher/safety-pilot conditions.

Technical references: [MAVROS global position](https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/src/plugins/global_position.cpp),
[home position](https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/src/plugins/home_position.cpp),
[raw setpoints](https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/src/plugins/setpoint_raw.cpp),
[NavSatFix altitude convention](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/NavSatFix.msg).


## Current implementation: telemetry only

For simulation, start Gazebo and MAVROS separately, then run:

```bash
ros2 launch mission_executor outdoor_sim.launch.py
```

This launch starts the one-way Gazebo `/clock` bridge and the mission node with
`use_sim_time=true`. Stop any manually started `/clock` bridge first to avoid
duplicate publishers. Configure the separately launched MAVROS to use simulation
time as well. Flight remains disabled unless `enable_flight:=true` is supplied.

`outdoor_real.launch.py` is a placeholder that starts the mission node with
`use_sim_time=false` and no clock bridge. It contains TODOs for MAVROS hardware
transport, telemetry setup, frame alignment, limits and failsafe validation;
real-flight setup has not been completed. Both entry points reuse the common
`outdoor_launch.py` and accept `params_file`, `enable_flight`, `execute_square`
and `execute_figure_eight`.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mission_executor
source install/setup.bash
ros2 launch mission_executor outdoor_launch.py
```

Run with MAVROS connected, or exercise fake telemetry without a simulator:

```bash
python3 src/mission_executor/test/outdoor_smoke_test.py
python3 src/mission_executor/test/outdoor_smoke_test.py --no-home
```

On FCU connection, the node requests MAVLink `EXTENDED_SYS_STATE` (245) at 2 Hz
through `mavros/set_message_interval` once per node run. Both mission modes wait
for a successful response, then require actual fresh landed-state messages. A
rejection or no response within `command_timeout` wall seconds enters terminal
FAILED; the node keeps reporting the reason. Restart to try again, including
after an FCU restart. There are no retries or reconnection requests. Waiting for
the FCU/service to become available does not start the response timeout.

By default (`enable_flight:=false`) the node creates no flight command clients or
setpoint publishers. Alongside the telemetry stream request, it
subscribes to telemetry and publishes `outdoor_mission/state` and
`outdoor_mission/diagnostics` (`DiagnosticArray`, 1 Hz). Inspect with:

```bash
ros2 topic echo /outdoor_mission/diagnostics
```

`REFERENCE_READY` means telemetry and the diagnostic reference are available,
not that heading alignment or flight is approved. Compare
`compass_yaw_enu_degrees`, `local_yaw_degrees` and `yaw_alignment_error_degrees`
during ground rotation using `ros2 topic echo /outdoor_mission/diagnostics` or
rosbag recordings. Compass/local-yaw disagreement is diagnostic only; it does not
block or abort flight. Commanded-versus-measured local yaw still determines corner
settling. Also compare GPS-derived ENU with local-derived ENU during
known ground translation. `local_yaw_offset_deg` is configured, never estimated
silently. Home is subscribed with reliable transient-local QoS; absent/incompatible
home telemetry falls back to the qualified-fix origin. Check actual publisher QoS.

Capture requires a disarmed vehicle on ground and GPS/pose receive times within
`reference_sample_window` (0.5 s). After capture, local/GPS updates do not move the
origin or launch baseline; telemetry loss resets the readiness dwell, not the
reference. Restart the node while disarmed to capture a new run. The flight mode
checks home/frame changes; GPS/local disagreement is diagnostic only.
All callbacks run serially on a single-threaded executor, without a mutex or
separate setpoint callback group. Validate setpoint cadence under simulator load.

The diagnostic node checks the proposed envelope with `envelope_margin` (initial
0.5 m); an invalid horizontal envelope produces `INVALID_ENVELOPE`. Shape, heading,
WGS84 conversions and frame transforms are numerical functions in
`outdoor_geometry`. Geometry tests can be run separately from the existing indoor
package's style checks:

```bash
colcon test --packages-select mission_executor --ctest-args -R test_outdoor_geometry
colcon test-result --test-result-base build/mission_executor/test_results --verbose
```

## Outdoor flight sequence

With flight enabled, the current sequence is:

```text
WAIT_READY -> SET_GUIDED -> ARM -> TAKEOFF -> STABILIZE
 -> SQUARE (W1, W2, W3, W4, W1) -> MOVE_TO_CENTER -> ALIGN_TANGENT
 -> FIGURE_EIGHT -> SETTLE_CENTER -> RETURN_HOME -> LAND -> DISARM -> DONE
```

The square uses bounded-speed position interpolation and continuously faces the POI
from measured position. Every corner requires position, altitude, speed and yaw
settling for the configured dwell. Each leg has its own `phase_timeout`.
After the square, the drone transfers above the POI, settles, then holds the
centre while aligning to the initial tangent (45 degrees in mission ENU). Only
after position, speed and heading settle does the figure-eight clock start.
Targets are evaluated directly from elapsed ROS time for one lap; position and
unwrapped tangent yaw are published continuously. At lap completion, the drone
holds the centre and final tangent until settled, then returns to launch.

Set `execute_figure_eight:=false` for the square-only flight, or
`execute_square:=false` for takeoff/return only (also skips the figure-eight). In an already started outdoor
SITL environment, after verifying the coordinate/heading diagnostics above:

```bash
ros2 launch mission_executor outdoor_sim.launch.py enable_flight:=true
```

This command automatically enters GUIDED and arms after the readiness gate. It is
not a diagnostic-only run. The simulation launch starts the clock bridge and enables
simulation time; clocks and telemetry must continue throughout the run.

Telemetry receipt ages, GPS/pose pairing, readiness/settling dwell and flight-phase
durations use ROS time. In simulation, supply Gazebo's bridged `/clock` and set
`use_sim_time:=true` on MAVROS as well as this node. Without simulation time, the
same checks use normal system time. Service-response deadlines and command-confirmation phases retain steady wall time.
`clock_stall_timeout` (10 wall seconds) invalidates readiness if ROS time stops
advancing; active navigation then aborts. Backward clock resets discard telemetry
timestamps and readiness dwell; an active mission releases control and requires
a node restart. Wall timers continue checking these conditions during a pause.
The controller still uses the configured telemetry timeout for heartbeat messages.

Exercise slow time, pauses and reset recovery without a simulator:

```bash
python3 src/mission_executor/test/outdoor_clock_test.py
```

Abort from another sourced terminal:

```bash
ros2 service call /outdoor_mission/abort std_srvs/srv/Trigger "{}"
```

The node checks each response and observed mode/armed/airborne/landed telemetry.
Takeoff altitude is the current launch-relative-to-home reading plus 5 m; the local
position target is captured launch Z plus 5 m. No position setpoints oppose the
initial autopilot takeoff. Once airborne and takeoff accepted, a 30 Hz
timer publishes position and yaw, including during settling. Return transfers use
1 m/s interpolation; arrival requires the agreed position, altitude, speed and yaw
conditions for 1 s. Only fresh on-ground confirmation permits disarming.

The current heading is preserved for takeoff/hover. Mode override and connection
loss stop the stream and enter RELEASE_CONTROL. Explicit abort, navigation-quality
loss, home/frame changes, tracking/fence violation and phase failures request
LAND while GUIDED control is still owned. A successful abort ends in ABORTED after
ground/disarm; rejected landing or landing timeout ends in FAILED and needs operator
recovery. No terminal state restarts automatically. Stale service responses cannot
advance a later phase.

Current limits in YAML are simulation starting values: command 10 s, takeoff 45 s,
settling/return 60 s, landing 60 s; maximum position-target error 2 m, GPS/local
diagnostic horizontal disagreement 3 m, and diagnostic relative/local altitude disagreement 1 m. A home
position change greater than 0.5 m after takeoff reference capture aborts the mission.
The FCU home must be available before the takeoff service is called, even when the
mission origin used fix fallback. This mode does not prove the alignment of a new
vehicle: confirm the diagnostics first.

Fake-vehicle integration tests (isolated ROS domain; no simulator/hardware):

```bash
python3 src/mission_executor/test/outdoor_flight_test.py
```

The user has accepted the square behavior in simulation. Full-sequence
Gazebo/SITL acceptance, repeated runs and recordings remain to be performed.

Current measurement assumptions, open questions and fault policy are tracked in
[OUTDOOR_REVIEW.md](OUTDOOR_REVIEW.md). Compass freshness/validity gates preflight,
not flight; GPS/local and altitude consistency are diagnostic-only comparisons.

Startup stream setup tests (rejection, late response, missing service/data, both modes):

```bash
python3 src/mission_executor/test/outdoor_stream_test.py
```
