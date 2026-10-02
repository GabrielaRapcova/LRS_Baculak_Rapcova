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
restart. Inflation and planning are the next stages. Shelf-gap evaluation remains
in the user's testing reminder in the implementation plan.
