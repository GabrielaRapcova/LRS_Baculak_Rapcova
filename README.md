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

Override `params_file` with a YAML file to tune the vehicle dimensions. Defaults
are initial development values: vehicle radius 0.3 m, position tolerance 0.1 m,
safety margin 0.1 m, giving 0.5 m blocked clearance. Soft costs extend to 0.8 m.
Distances are measured between voxel centers; voxel discretization and controller
accuracy must be accounted for when choosing a margin. These defaults are not a
validated flight configuration.

The costmap inherits frame, origin and resolution from `/map`; it has no geometry
parameters. Costs follow the overview: 0 free, 1–252 soft inflation, 253 blocked
clearance, 254 physical obstacle, 255 unknown. Soft costs decrease exponentially
with distance using `cost_scaling_factor` (inverse metres). `inflation_radius` is
the total radius from an obstacle and must be at least the blocked clearance.
Only `unknown_policy: blocked` is currently supported. Unknown cells remain
unknown and are not overwritten by inflation.

- `/costmap`: reliable transient-local snapshot, periodically republished.
- `/costmap_updates`: reliable complete-block replacements with independent
  costmap sequence numbering, emitted after map updates.
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
Only a full map snapshot restores synchronization. Map geometry cannot change
within one `map_id`. Startup parameters require a node restart to change.

```bash
python3 src/voxel_costmap/test/smoke_test.py
ros2 service call /check_collision uav_navigation_msgs/srv/CheckCollision \
  "{point: {x: 1.0, y: 2.0, z: 1.0}}"
```
