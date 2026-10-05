# Gradual implementation

The architecture in README_mapper_planner_overview.md is the target. Assignment
requirements were read from the sibling ../assignments checkout. Each stage ends
with a runnable demonstration and validation before the next stage begins.

1. **Contracts and spatial core (implemented).** Buildable uav_navigation_msgs,
   fixed 8³ blocks, coordinate conversion and indexing. Validate negative positions,
   block boundaries, invalid geometry and generated ROS interfaces.
2. **Static map provider (implemented).** Replace the prototype mapper with static_mapper:
   configurable PCD path, resolution, frame and origin; finite-point filtering;
   sparse occupancy, bounds/count logging, occupancy queries and RViz cloud.
   Publish /map with reliable transient-local QoS. No inflation here.
   Explicitly configure a bounded known-free domain for the complete static map;
   missing blocks outside that domain remain unknown. PCD alone does not prove free
   space. Demonstrate occupied/free/unknown queries on the hangar map.
3. **Voxel costmap (implemented).** Inherit geometry, apply configurable vehicle radius + position
   tolerance + margin and a cached 3D inflation stencil. Preserve unknown status
   according to an explicit policy, publish snapshots and replacement updates,
   and move collision checks here. Check distances, block-edge inflation, obstacle
   removal and stale/gapped updates. Start with full rebuilds; optimize afterward.
4. **3D A* and simplification (A1.1, next).** Bounded search, Euclidean admissible heuristic
   with weight 1, configurable connectivity, collision-checked diagonal edges,
   start/goal validation and explicit failure reporting. Publish nav_msgs/Path.
   Simplify with conservative voxel traversal and recheck all segments. Demonstrate
   altitude-changing routes and unreachable goals; record
   search time and point counts before/after for at least two routes.
5. **Composition and repeatable A1.1 demo.** Common/implementation YAML and one launch
   for static_mapper → voxel_costmap → astar_planner; configurable map and requests,
   RViz setup, clean-checkout build/run instructions and evidence collection.
6. **Indoor mission (A1.2).** Mission-file parsing, MAVROS handshake and state machine,
   continuous setpoints independent of planning, pass-through path points, hard/soft
   mission goals, yaw tasks, landing/relaunch and abort. Validate in simulation.
7. **Outdoor mission (A1.3).** Home-relative GPS coordinates, verified ENU/NED heading,
   POI tracking, continuous figure-eight with tangent yaw, return/land and abort.
   Validate transitions and record commanded versus actual flight.
8. **Documentation and defence (A1.4).** Keep design decisions and measured evidence
   throughout development; finalize build instructions, diagram and demo recordings.
9. **Assignment 2 extension.** Select the topic with the user before implementing.
   For obstacle avoidance: sensor TF, ray-based free-space evidence, incremental
   blocks, synchronization recovery and replanning. Theta* and ESDF are optional
   extensions after baseline acceptance.

## Interface decisions

- 8 voxels per block edge; x varies fastest. Signed block indices support negative
  world coordinates. Origin is a Point and grid axes follow the map frame.
- Occupancy providers emit 0/254/255. Only costmaps emit inflated costs 1–253.
- Missing blocks are unknown. All-unknown replacement blocks clear prior data.
- Snapshots fully replace state. Updates have map_id and monotonically increasing
  sequence; consumers reject mismatched geometry epochs and sequence gaps and wait
  for a fresh snapshot. Providers must periodically republish snapshots for recovery.
- Snapshot QoS: reliable, transient-local, depth 1. Update QoS: reliable, volatile;
  depth and snapshot frequency will be configured during provider implementation.
- Existing CheckCollision.srv is retained for compatibility; its semantics and
  owning node will be documented during the costmap stage.

## Reminder for user testing

Check whether configured 3D inflation closes the shelf gaps and keeps planned
paths outside the simulator's solid rack collision volumes. Shelf evaluation and
any decision about explicit volume filling are left to the user's testing.

## Current limitations

The workspace builds the interfaces, spatial core, static mapper and voxel costmap.
Planning and flight control are not implemented yet. Known-free bounds must be
chosen by the user for the static map; defaults preserve unsampled space as unknown.

## Stage 2 validation

- All three packages built with ROS 2 Jazzy.
- Synthetic ROS smoke test passed: duplicate-point voxelization, nonfinite-point
  filtering, negative coordinates, free-domain boundaries, occupancy queries and
  transient-local delivery to a late subscriber.
- Real-map launch loaded 620311 finite points at 0.2 m resolution into 396 blocks
  with 32012 occupied voxels. Bounds: [-0.3, -1.15, 0] to [17.95, 13.2, 5.95041].
- Missing map, unreadable map, zero resolution and oversized known-free domain
  exited with errors; existing coordinate tests passed.
- RViz visual inspection and selection of actual known-free bounds remain for
  user evaluation. No shelf evaluation was performed.

## Stage 3 validation

- The voxel_costmap package builds with ROS 2 Jazzy.
- Synthetic integration checks passed for vertical and diagonal inflation,
  cross-block costs, unknown preservation, collision queries, obstacle removal,
  block clearing, replacement deltas, stale updates, sequence-gap recovery,
  geometry reset, malformed occupancy and retained delivery to late subscribers.
- Real-map composition passed with explicit test-only known-free bounds
  [-1, -2, -1] to [19, 15, 7]: 936 blocks, 32012 occupied and 307988 free voxels.
  At 0.2 m resolution, 0.5 m clearance and 0.8 m inflation, an optimized build
  rebuilt costs in approximately 287 ms on this machine. This is one measurement,
  not a real-time guarantee; full rebuilds can delay callbacks on larger maps.
- Invalid clearance/inflation configuration, negative radius and unsupported
  unknown policy fail at startup. Coordinate tests remain passing.
- Test bounds and clearance values are not validated flight settings. User
  selection of known-free bounds, vehicle margin and shelf evaluation remains.
