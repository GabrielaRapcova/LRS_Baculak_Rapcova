> **Archived development document.** Progress notes, examples and TODOs below
> describe earlier revisions and are not the current acceptance checklist.
> See [DOCUMENTATION_AND_DEFENCE.md](../../DOCUMENTATION_AND_DEFENCE.md).

# ROS 2 UAV navigation

Target architecture: [system overview](README_mapper_planner_overview.md).
Stages and acceptance criteria: [implementation plan](IMPLEMENTATION_PLAN.md).

## Foundation (ROS 2 Jazzy)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select uav_navigation_msgs voxel_grid_core
source install/setup.bash
colcon test --packages-select uav_navigation_msgs voxel_grid_core
colcon test-result --verbose
ros2 interface show uav_navigation_msgs/msg/VoxelMap
```

`uav_navigation_msgs` defines sparse 8³ voxel blocks, occupancy/costmap snapshots
and block replacement updates. `voxel_grid_core` provides ROS-independent C++17
coordinate conversion and indexing, exported as `voxel_grid_core::voxel_grid_core`.

## Static map provider

```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
ros2 launch mapper mapper_launch.py map_file:=/absolute/path/to/map.pcd
```

For the course checkout, the map is `../maps/FEI_LRS_PCD/map.pcd` relative to
this workspace. Pass its absolute path. Override `params_file` to use your own
YAML; the default is [params.yaml](../../src/mapper/config/params.yaml).

- `/map`: retained reliable `VoxelMap` snapshot, periodically republished.
- `/map_cloud`: retained `PointCloud2` of occupied voxel centers; display it in
  RViz with Fixed Frame `map` and transient-local durability.
- `/query_occupancy`: `CheckCollision` service; coordinates are in the map frame.
  Costs are 0 free, 254 occupied and 255 unknown. `collision` is true for occupied
  and unknown cells. This is an occupancy query without vehicle inflation.

```bash
ros2 service call /query_occupancy uav_navigation_msgs/srv/CheckCollision \
  "{point: {x: 1.0, y: 2.0, z: 1.0}}"
python3 src/mapper/test/smoke_test.py
```

The provider directly bins finite PCD samples into voxels (one occupied voxel per
bin), retaining negative coordinates. `origin` defines voxel boundaries and does
not translate the point cloud. `resolution` is rounded to the message's float32
representation before mapping so consumers use identical geometry.

By default only sampled voxels are known. To assert known-free space, configure
both `known_free_min` and `known_free_max` as three-element arrays. Voxel centers
inside that half-open box become free before occupied samples are inserted.
Only declare a region free when your static-map assumptions justify it. Boundary
voxels are classified by their centers; samples outside the box still become
occupied. Allocated blocks are limited by `max_blocks` (default 100000).

Parameters are read at startup; restart to apply changes. This static provider
publishes no incremental updates. Its sequence stays zero and map_id changes on
restart. Planning is the next stage. Shelf-gap evaluation remains
in the user's testing reminder in the implementation plan.


## Voxel costmap

Start the mapper as above, then in another sourced terminal:

```bash
ros2 launch voxel_costmap costmap_launch.py
```

Override `params_file` with a YAML file to tune the vehicle dimensions. The current
YAML has vehicle radius 0.3 m, position tolerance 0.1 m and safety margin 0.5 m,
giving 0.9 m blocked clearance. A 0.8 m soft inflation zone extends beyond that
clearance, giving a total reach of 1.7 m. The node fallback safety margin is 0.1 m
when no YAML is supplied (0.5 m clearance and 1.3 m total reach).
Distances are measured between voxel centers; voxel discretization and controller
accuracy must be accounted for when choosing a margin. These values are not a
validated flight configuration.

The costmap inherits frame, origin and resolution from `/map`; it has no geometry
parameters. Costs follow the overview: 0 free, 1–252 soft inflation, 253 blocked
clearance, 254 physical obstacle, 255 unknown. Soft costs decrease exponentially
with distance using `cost_scaling_factor` (inverse metres). `inflation_radius` is
the width of the soft zone beyond blocked clearance, not the total radius.
The total radius is vehicle_radius + position_tolerance + safety_margin +
inflation_radius. A zero inflation_radius disables soft costs while retaining
the blocked clearance. All radius/tolerance values must be finite and nonnegative.
Only `unknown_policy: blocked` is currently supported. Unknown cells remain
unknown and are not overwritten by inflation.

- `/costmap`: reliable transient-local full snapshot, published once after
  initialization and again only when new map input is processed or invalidated.
  There is no periodic republication and no `/costmap_updates` publisher.
- `/costmap_cloud`: retained PointCloud2 containing nonzero known costs in an
  `intensity` field. Add it in RViz with transient-local durability, Boxes of size
  equal to map resolution, and Color Transformer `Intensity` (range 1–254).
- `/check_collision`: inherited map-frame coordinates, blocked for costs ≥253,
  including unknown. Returns 255 and collision=true before a map arrives or while
  waiting to recover synchronization. `/query_occupancy` still queries the raw map.

**To see an inflation halo, configure known-free bounds on the mapper.** With the
occupied-only default, neighbors remain unknown, so the cost cloud shows only
physical obstacles. Choose bounds based on your static-map assumptions rather
than treating the entire world as free.

A cached 3D stencil is applied around occupied cells; overlapping inflation takes
maximum cost. Costs are rebuilt from occupancy after each update, so removal also
clears inflation in neighboring blocks. This first version favors correctness
before incremental-performance optimization. The costmap package defaults to
`RelWithDebInfo` when no build type is selected. `max_stencil_cells` limits the
candidate stencil cube to prevent excessive allocations.

Duplicate/stale updates are ignored. Sequence gaps, frame/epoch mismatches and
malformed input invalidate costs and publish an empty snapshot (all space unknown).
Only a full map snapshot restores synchronization. Full costmap snapshots are
published after accepted map changes; repeated identical map snapshots are ignored.
Late subscribers receive the retained costmap while the costmap node stays alive. Map geometry cannot change
within one `map_id`. Startup parameters require a node restart to change.

```bash
python3 src/voxel_costmap/test/smoke_test.py
ros2 service call /check_collision uav_navigation_msgs/srv/CheckCollision \
  "{point: {x: 1.0, y: 2.0, z: 1.0}}"
```


## Request-driven 3D planner

The planner follows [README_planner.md](README_planner.md). Start mapper and costmap,
then in another sourced terminal:

```bash
ros2 launch astar_planner planner_launch.py
ros2 action send_goal /compute_path_3d uav_navigation_msgs/action/ComputePath3D \
  "{start: {header: {frame_id: map}, pose: {position: {x: 13.6, y: 1.5, z: 1.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: 8.65, y: 2.02, z: 1.0}}}}" \
  --feedback
```

Those example coordinates come from the assignment mission. Their usability
must be checked against your current known-free domain and clearance settings.
The action returns the refined `nav_msgs/Path`, status/message, duration in seconds,
expanded node count and point counts before/after refinement. Both input frame IDs
must equal the costmap frame; this planner performs no TF transforms. Positions
are preserved exactly in a successful path; request orientations are ignored and
output orientations are identity because the result is an XYZ geometric path.

For visualization add a **Path** display in RViz, Topic `/plan`, Fixed Frame `map`,
with transient-local durability. `/plan` is optional (`publish_plan: false` disables
it); the action result is authoritative. Failed requests do not publish a new path,
so an older visualization may remain visible. Costmap reception never triggers
planning, invalidation or republication of a previous path. A mission node must
request a new path and decide whether an earlier result is still suitable.

Search uses 26-connected A* with an admissible Euclidean heuristic and symmetric
edge costs: distance × (1 + cost_weight × mean endpoint cost / 252). Unknown and
costs ≥253 are blocked. Diagonal edges check all touched neighbor cells. Search
state is a vector of discovered nodes, index lookup and priority queue, discarded
after each request. The costmap is an immutable snapshot captured for that search;
updates during a request apply to subsequent requests.

Greedy line-of-sight refinement removes intermediate points and rechecks every
returned segment with conservative voxel traversal, including face/edge/corner
contacts. It preserves collision freedom, but can increase soft-cost exposure
relative to the discrete A* route; it is not trajectory optimization. Endpoints
on voxel boundaries can be rejected if they touch an unknown or blocked neighbor.

The node accepts one active request at a time and rejects concurrent requests.
Search runs in a managed worker so costmap reception and cancellation remain
responsive. Feedback reports expansions every 256 nodes. Configure time, expansion,
discovered-node and segment-traversal limits in
[astar.yaml](../../src/astar_planner/config/astar.yaml), or override `params_file` at launch.
Cancellation, invalid start/goal, missing map, no path and limit exhaustion return
explicit statuses with empty paths. Parameters are read at startup.

```bash
colcon test --packages-select astar_planner voxel_grid_core
colcon test-result --verbose
python3 src/astar_planner/test/smoke_test.py
```

## Indoor mission executor (A1.2)

The `mission_executor` package implements **A1.2 — indoor mission execution** in
ROS 2 Jazzy (C++). It reads waypoints from a CSV file and executes them using
ArduPilot through MAVROS. The route **between CSV waypoints** is requested from
the A1.1 3D A* planner (`/compute_path_3d`); intermediate path points are not
additional mission tasks.

Supported mission tasks: `takeoff`, `land`, `landtakeoff`, `yaw90`, `yaw180`
(and other `yaw<angle>` values). `HARD` waypoints require close, stable arrival
(0.15 m, speed <= 0.15 m/s for at least 1 s), whereas `SOFT` waypoints use a
0.50 m acceptance radius. Waypoint yaw tasks are **absolute angles in MAVROS
ENU**, and normal flight heading follows the direction of travel.

### Build

```bash
cd ~/LRS_Baculak_Rapcova
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to mission_executor
source install/setup.bash
ros2 pkg prefix mission_executor
```

The package path printed by the last command should point to this workspace.
Use `source /opt/ros/jazzy/setup.bash` and `source
~/LRS_Baculak_Rapcova/install/setup.bash` in **every new ROS 2 terminal**.

### Start the complete system (separate terminals)

Start the components in this order; leave each process running:

1. **Gazebo Sim** with the indoor/hangar world and UAV model.
2. **ArduPilot SITL** connected to that Gazebo instance.
3. **MAVROS** connected to the SITL autopilot.
4. **A1.1 mapper**, **voxel costmap**, then **3D A* planner**.
5. **Mission Executor**, only after the checks below pass.

Use **three separate terminals** for these simulator processes, following
the Gazebo hangar-world, SITL vehicle-model and MAVROS FCU connection settings
of your existing simulation setup:

- **Terminal 1:** `gz sim` with the configured indoor world/UAV model.
- **Terminal 2:** ArduPilot `sim_vehicle.py` configured for that Gazebo vehicle.
- **Terminal 3:** MAVROS connected to the SITL FCU endpoint.

The exact world file, `sim_vehicle.py` model options and MAVROS `fcu_url` vary
between checkouts and are **not supplied here as unverified commands**. Follow
the simulation setup used for your repository. Starting Gazebo alone is not
equivalent to starting a fresh SITL/MAVROS session; avoid leaving an earlier
armed autopilot running after restarting only Gazebo.

After MAVROS starts, check connection and ground state from another sourced
terminal:

```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/extended_state --once
```

Before a *new ground-start mission*, require `connected: true`, `armed: false`
and `landed_state: 1` (`ON_GROUND`). Verify a fresh pose and velocity as well:

```bash
ros2 topic echo /mavros/local_position/pose --once
ros2 topic echo /mavros/local_position/velocity_local --once
```

### MAVROS telemetry: required message interval in our SITL setup

In previous Gazebo tests, `/mavros/extended_state` was sometimes too slow or
stale, so the executor refused to arm with `A1.2 preflight blocked`. After
**MAVROS and SITL are connected**, request the MAVLink `EXTENDED_SYS_STATE`
message (ID **245**) at **5 Hz**:

```bash
ros2 service call /mavros/set_message_interval \
  mavros_msgs/srv/MessageInterval \
  "{message_id: 245, message_rate: 5.0}"
```

Confirm the response is successful and verify that the topic is updating:

```bash
ros2 topic hz /mavros/extended_state
ros2 topic hz /mavros/local_position/pose
ros2 topic hz /mavros/local_position/velocity_local
```

Run these `hz` checks one at a time (stop each with `Ctrl+C`). Message 245
controls **extended state**, not the pose or velocity streams. If pose/velocity
are stale, investigate their upstream MAVROS/SITL streams; do not assume the
message-245 command fixes every topic. Repeat this service request after
restarting SITL or MAVROS if the stream rate drops. The flight parameter
`task_slow_sitl_telemetry:=true` only relaxes timing thresholds for slow SITL;
it **does not disable preflight safety checks**.

Start A1.1 in **three more terminals**, after sourcing ROS 2 and this workspace
in each of them:

```bash
# Terminal 4: map provider
ros2 launch mapper mapper_launch.py \
  map_file:=$HOME/large_files/maps/FEI_LRS_PCD/map.pcd
```

```bash
# Terminal 5: voxel costmap
ros2 launch voxel_costmap costmap_launch.py
```

```bash
# Terminal 6: A* planner
ros2 launch astar_planner planner_launch.py
```

Check that the A* action server is available:

```bash
ros2 action list -t
# Expected: /compute_path_3d [uav_navigation_msgs/action/ComputePath3D]
```

### Run the seven-waypoint mission (WP1 ground start)

The example file is [`missions/exam_example.csv`](../../missions/exam_example.csv).
The first CSV task must be `takeoff` and the last `land`. Once the UAV is safely
on the ground, disarmed, and the route/map/frame configuration has been checked,
start the executor in **Terminal 7**:

```bash
cd ~/LRS_Baculak_Rapcova
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run mission_executor mission_executor_node --ros-args \
  -p mission_file:=$HOME/LRS_Baculak_Rapcova/missions/exam_example.csv \
  -p enable_task_flight:=true \
  -p task_flight_confirmed:=true \
  -p task_slow_sitl_telemetry:=true \
  -p enable_task_planned_flight:=true \
  -p task_planner_frame_confirmed:=true \
  -p task_planned_max_z_m:=4.0 \
  -p task_path_point_tolerance_m:=0.20
```

The confirmation parameters above apply **only to the previously checked
Gazebo SITL setup**. They are not a way to bypass failed preflight checks.
With dynamic start disabled (the default), the drone must begin close to WP1
(approximately 0.45 m in horizontal distance).

### Optional: start the same mission from another ground position

Recent versions of the executor include an **opt-in dynamic start**: the UAV
first takes off vertically at its **current ground XY**, then A* plans a route
to WP1, which is visited without performing a second takeoff. It then flies
the remaining CSV mission. Use this only in the checked Gazebo configuration
with the mapped area reachable and a **manually verified clear ascent column**.
The vertical ascent itself is **not collision-checked by A***.

Use this **complete command** in place of the preceding `ros2 run` command:

```bash
ros2 run mission_executor mission_executor_node --ros-args \
  -p mission_file:=$HOME/LRS_Baculak_Rapcova/missions/exam_example.csv \
  -p enable_task_flight:=true \
  -p task_flight_confirmed:=true \
  -p task_slow_sitl_telemetry:=true \
  -p enable_task_planned_flight:=true \
  -p task_planner_frame_confirmed:=true \
  -p task_planned_max_z_m:=4.0 \
  -p task_path_point_tolerance_m:=0.20 \
  -p enable_dynamic_start:=true \
  -p dynamic_start_clearance_confirmed:=true
```

The dynamic-start flags exist only in the newer implementation. Check the
source before running this command after changing Git branches:

```bash
grep -n 'DYNAMIC START' src/mission_executor/src/mission_executor_node.cpp
```

**Dynamic start is not an airborne restart:** stopping `mission_executor_node`
does not stop/reset ArduPilot. After `LAND -> DONE`, the next ground-start run
requires `ON_GROUND`, `DISARMED`, fresh telemetry, and a clear ascent area.
Never disarm an airborne UAV to reset the test.

### Non-flying CSV check and troubleshooting

For validation **without issuing flight commands**, use:

```bash
ros2 run mission_executor mission_executor_node --ros-args \
  -p mission_file:=$HOME/LRS_Baculak_Rapcova/missions/exam_example.csv \
  -p enable_task_sequence_check:=true
```

A successful integrated flight ends with:

```text
LAND verified: ON_GROUND and DISARMED.
State transition: LAND -> DONE
```

If the executor reports `A1.2 preflight blocked`, verify `armed: false`,
`landed_state: 1`, current `/mavros/state`, pose/velocity message age/rate,
and the extended-state interval above. A new run is intentionally blocked if
the UAV is still flying or armed. If the node stays in `ARM`, check that you
are executing the binary rebuilt from the expected source and workspace:

```bash
ros2 pkg prefix mission_executor
```

In this Gazebo/PCD configuration, the verified CSV/map to MAVROS ENU mapping is
`(x_enu, y_enu, z_enu) = (-y_map, x_map, z_map)`. This is a setup-specific
conversion, not a general TF transform. Planned-flight mode stops on A*
failure rather than silently switching to an obstacle-crossing straight line.
The earlier seven-waypoint example was completed in Gazebo SITL (including
`yaw180`, `yaw90`, `landtakeoff` and final `LAND -> DONE`). Dynamic start and
repeat flights should be recorded and verified against the **actual Git commit**
being tested before describing them as fully accepted.

## Some interesting points for global planner:

1.262; 6.6042; 4.8463 - over first regal
4.428; 6.7152; 4.8495 - over second regal
7.5038; 6.7619; 4.8526 - over third regal
10.518; 6.3189; 4.8556 - 4th hole in roof
13.655; 6.3526; 4.8587 - 5th hole in the roof
4.3806; 9.1802; 2.2504 - left to first regal
5.7076; 4.8771; 2.0405 - between regals

Historical review note: review the A* heuristic rationale in the current documentation.

## FlexBE installation for A1.2

On Ubuntu 24.04 with ROS 2 Jazzy and its apt repository already configured:

```bash
./scripts/install_flexbe.sh --dry-run
./scripts/install_flexbe.sh
```

The script installs the released `flexbe_core` and `flexbe_onboard` packages,
updates apt package indexes, and verifies the Python state-machine imports.
Use `--with-tools` to include released states, mirror and widget tooling.
The WebUI is not installed by this script; its setup depends on the engine/UI
versions selected for mission development. The script uses the Jazzy binary
release rather than cloning the upstream development branch.


### Latest FlexBE engine and WebUI from source

```bash
./scripts/install_flexbe_source.sh --dry-run
./scripts/install_flexbe_source.sh
```

The source installer targets Ubuntu 24.04 / Jazzy and creates a separate
`~/flexbe_ws` overlay. It fetches the engine's `ros2-devel` and WebUI's `main`
branches, builds the engine, mirror, widget and WebUI, and installs desktop-client
Python dependencies in a system-site-packages virtual environment. The existing
apt FlexBE installation remains as an underlay. Use `--workspace /absolute/path`
to choose a different overlay; `--engine-ref` and `--webui-ref` can pin revisions.
Rerunning fetches the chosen refs again and refuses modified source checkouts.
Fetched commits and Python package versions are recorded in the overlay.

In each fresh terminal:

```bash
source ~/flexbe_ws/setup_flexbe.bash
# Source navigation packages afterward when needed:
source /home/user/LRS-URK/ros2_ws/install/local_setup.bash
ros2 launch flexbe_webui flexbe_full.launch.py
```

For a browser instead of the Qt desktop client, run the onboard engine separately
and launch `flexbe_ocs.launch.py headless:=true`; open `http://127.0.0.1:8000`.
Always source the matching FlexBE overlay on both sides; the source 4.x messages
must not be mixed with the apt 3.x engine. Build future mission packages with this
source overlay active as their underlay.


## Indoor mission launch (A1.2)

Keep Gazebo/SITL running and start MAVROS with `scripts/run_mavros.sh` from the repo
root. The indoor launch configures the existing MAVROS instance; it does not start
or stop it. Mapper, costmap and planner are started separately.
From `ros2_ws`, after sourcing ROS and `install/setup.bash`:

```bash
# Check the CSV sequence and visualize telemetry; no mission flight commands.
ros2 launch mission_executor indoor.launch.py \
  mission_file:="$PWD/missions/exam_example.csv"

# Execute the A*-planned indoor mission in SITL after checking frame alignment.
ros2 launch mission_executor indoor.launch.py \
  mission_file:="$PWD/missions/exam_example.csv" \
  enable_flight:=true \
  task_flight_confirmed:=true \
  task_planner_frame_confirmed:=true
```

Set RViz Fixed Frame to `map`; display `/plan` and `/mavros/local_position/odom`.
MAVROS local telemetry is labelled `mavros_local`, with TF to `base_link`. The launch
publishes `map` → `mavros_local` at yaw −90°, zero translation, matching the current
executor's fixed `(x_local, y_local, z_local) = (-y_map, x_map, z_map)` conversion.
Verify a known position before flight; another origin/world requires revisiting
both the transform and executor conversion. This change labels frames correctly;
it does not rotate the MAVROS telemetry values or change flight targets.

The launch waits up to 10 seconds for `/mavros/local_position` parameter services,
then applies `config/indoor_mavros.yaml` before starting the executor. A rejected
or timed-out configuration stops launch. Frame settings remain on MAVROS until
changed or MAVROS is restarted. Restart/reconfigure it before switching to an
outdoor setup that expects different frame labels. The default `mavros_frame=map`
is retained for direct executor runs outside this launch. Indoor freshness checks
still use wall time; `task_slow_sitl_telemetry:=true` selects the existing slow-SITL
profile for flight. No clock bridge or timing-policy change is included here.
Landed-state telemetry must be present; the indoor node does not request it
automatically. If missing, request it once:

```bash
ros2 service call /mavros/set_message_interval mavros_msgs/srv/MessageInterval \
  '{message_id: 245, message_rate: 2.0}'
```

## Outdoor mission design (A1.3)

Requirements, interfaces, coordinate conventions, state machine and validation
stages are documented in [README_outdoor_mission.md](README_outdoor_mission.md).
Geometry utilities, default telemetry diagnostics, takeoff/return checks and an
explicitly enabled full outdoor mission are implemented: square with POI yaw,
centre alignment, one figure-eight lap with tangent yaw, return and landing.
See the specification for launch, abort and test commands.

Outdoor run recordings: [Google Drive folder](https://drive.google.com/drive/folders/18p4L7dQqtNPm-RtJtOg_cNxpsdhLNMUm?usp=sharing).
GPS and magnetometer noise tests are still pending; these recordings do not establish
robustness to sensor noise.
