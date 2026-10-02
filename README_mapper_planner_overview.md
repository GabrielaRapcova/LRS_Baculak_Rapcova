# ROS 2 3D Navigation System

## Architecture

The system consists of three replaceable layers:

```text
Sensors / Static Map
        │
        ▼
┌─────────────────────┐
│ 1. Map Provider     │
└──────────┬──────────┘
           │ /map
           │ /map_updates
           ▼
┌─────────────────────┐
│ 2. Voxel Costmap    │
└──────────┬──────────┘
           │ /costmap
           │ /costmap_updates
           ▼
┌─────────────────────┐
│ 3. Planner          │
└──────────┬──────────┘
           │ /plan
           ▼
     nav_msgs/Path
```

## 1. Map Provider

Answers:

> **What physically exists in the environment?**

Multiple interchangeable implementations may exist:

```text
static_mapper
lidar_mapper
simulation_mapper
future_slam_mapper
```

Only the provider appropriate for the current scenario is launched.

Responsibilities:

- produce fixed-resolution 3D occupancy
- define map resolution, origin and frame
- sensor processing and occupancy fusion where applicable
- publish complete map snapshots
- publish incremental updates

Interface:

```text
/map          VoxelMap
/map_updates  VoxelMapUpdate
```

`/map` uses reliable + transient-local QoS.

The map is the **spatial authority**. Its resolution and geometry propagate downstream and are not independently configured by the costmap or planner.

---

## 2. Voxel Costmap

Answers:

> **Where can this vehicle safely move?**

Consumes:

```text
/map
/map_updates
```

Maintains a sparse, fixed-resolution voxel representation using the geometry defined by the map.

Responsibilities:

- vehicle dimensions
- safety margin
- obstacle inflation
- ESDF generation if used
- planning costs
- unknown-space policy

Publishes:

```text
/costmap          VoxelCostmap
/costmap_updates  VoxelCostmapUpdate
```

The costmap preserves:

```text
map resolution == costmap resolution
```

---

## 3. Planner

Answers:

> **How do I get from A to B?**

Possible interchangeable implementations:

```text
astar_planner
theta_star_planner
future_planner
```

Consumes:

```text
/costmap
/costmap_updates
start
goal
```

Produces:

```text
/plan
    nav_msgs/msg/Path
```

Planning requests may later be exposed through a ROS 2 action for cancellation, feedback and replanning.

---

# Map Representation

Maps use sparse blocks of homogeneous voxels:

```text
VoxelMap
  Header header
  float32 resolution
  Point origin
  VoxelBlock[] blocks
```

```text
VoxelBlock
  int32 x
  int32 y
  int32 z
  uint8[] occupancy
```

Occupancy:

Based on nav2 convention
```text
 255    unknown
 1-252  inflated
 253    inscribed obstacle (based on robot radius)
 254    lethal (actual obstacle)
```

Updates replace complete blocks:

```text
VoxelMapUpdate
  Header header
  VoxelBlock[] blocks
```

The costmap uses the same spatial structure but stores planning costs.

OctoMap may be used internally by a provider or supported for import/export, but it is **not part of the core navigation interface**.

---

# Configuration Model

Configuration is separated by responsibility.

```text
config/
├── mapper_common.yaml
├── planner_common.yaml
├── costmap.yaml
│
├── lidar_mapper.yaml
├── static_mapper.yaml
│
├── astar.yaml
└── theta_star.yaml
```

### Map-provider configuration

Shared map-provider parameters:

```text
publish_frequency
map frame conventions
update behavior
```

Implementation-specific parameters:

```text
lidar_mapper:
    resolution
    sensor range
    outlier filtering
    occupancy probabilities

static_mapper:
    map_file
    resolution / map metadata
```

The selected map provider establishes the map geometry.

### Costmap configuration

Vehicle-specific rather than mapper-specific:

```text
vehicle dimensions
safety margin
inflation parameters
ESDF parameters
unknown-space policy
```

Resolution and origin are **not configured here**; they come from `/map`.

### Planner configuration

Common planner behavior can be separated from algorithm-specific tuning.

```text
planner_common.yaml
    replanning behavior
    general limits

astar.yaml
    heuristic weight
    connectivity
    search limits

theta_star.yaml
    Theta*-specific parameters
```

The planner similarly obtains spatial resolution from `/costmap`.

---

# Launch Composition

The launch configuration chooses implementations and combines common and implementation-specific parameter files:

```text
Scenario: real LiDAR + A*

lidar_mapper
    mapper_common.yaml
  + lidar_mapper.yaml

voxel_costmap
    costmap.yaml

astar_planner
    planner_common.yaml
  + astar.yaml
```

Another deployment can replace implementations without changing interfaces:

```text
Scenario: simulation + Theta*

simulation_mapper
        ↓
   voxel_costmap
        ↓
 theta_star_planner
```

Topics remain fixed, so no compatibility or remapping layer is required.

---

# Core Design Principle

```text
Map Provider
"What exists?"
      │
      ▼
Voxel Costmap
"Where is it safe to fly?"
      │
      ▼
Planner
"How do I get there?"
```

Each layer owns its configuration.

The **map owns spatial geometry**, the **costmap owns vehicle/environment interaction**, and the **planner owns search behavior**.

Implementations can therefore be replaced independently while retaining the same ROS interfaces.