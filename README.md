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
YAML; the default is [params.yaml](src/mapper/config/params.yaml).

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
[astar.yaml](src/astar_planner/config/astar.yaml), or override `params_file` at launch.
Cancellation, invalid start/goal, missing map, no path and limit exhaustion return
explicit statuses with empty paths. Parameters are read at startup.

```bash
colcon test --packages-select astar_planner voxel_grid_core
colcon test-result --verbose
python3 src/astar_planner/test/smoke_test.py
```

## Some interesting points for global planner:

1.262; 6.6042; 4.8463 - over first regal
4.428; 6.7152; 4.8495 - over second regal
7.5038; 6.7619; 4.8526 - over third regal
10.518; 6.3189; 4.8556 - 4th hole in roof
13.655; 6.3526; 4.8587 - 5th hole in the roof
4.3806; 9.1802; 2.2504 - left to first regal
5.7076; 4.8771; 2.0405 - between regals

# TODO:
-[ ] check the A* heuristic
-[ ] 

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
