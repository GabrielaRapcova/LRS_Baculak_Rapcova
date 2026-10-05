3D Global Planner

Purpose

The planner computes a collision-free global path through the 3D voxel costmap.

start + goal + voxel costmap
            ↓
           A*
            ↓
 line-of-sight refinement
            ↓
      nav_msgs/Path

The planner is responsible only for global path computation.

Inputs

The planner maintains the latest costmap from:

/costmap
    VoxelCostmap

If incremental map transport is retained:

/costmap_updates
    VoxelCostmapUpdate

Updates only keep the planner's local costmap current. They do not trigger planning or path validation.

The costmap defines:

resolution

origin and frame

traversability cost

occupied / inflated / unknown voxels

The planner does not modify the costmap.

Planning Interface

Planning is request-driven through a ROS 2 action:

ComputePath3D

Request:

start
goal

Result:

nav_msgs/Path
planning status

A higher-level mission/navigation node decides when a new path is required.

The planner does not:

run periodically

automatically react to costmap changes

validate an already returned path

decide when replanning is necessary

Planning Algorithm

Initial implementation:

3D A*
  ↓
line-of-sight refinement

A* searches traversable voxels using 26-connectivity.

After finding the discrete path, unnecessary intermediate nodes are removed using collision-free line-of-sight checks through the costmap.

A*:

S → ● → ● → ● → G

refined:

S ─────→ ● ─────→ G

The resulting path is therefore suitable as a sparse geometric global path rather than a sequence of every traversed voxel.

Search Representation

The costmap and A* search state remain separate.

Persistent input:

SparseVoxelCostmap<uint8>

Temporary per-search state:

SearchNode
    voxel_index
    g_cost
    f_cost
    parent
    OPEN / CLOSED state

Only voxels reached during the search allocate planner state.

Recommended workspace:

vector<SearchNode> nodes
VoxelIndex → node index
priority queue / open set

The lookup structure provides fast access to previously discovered voxels while keeping search data contiguous.

The complete workspace is discarded/reset after each planning request.

Output

A successful request produces:

nav_msgs/msg/Path

containing the line-of-sight-refined XYZ path.

The action result is the authoritative result. An optional:

/plan

topic may publish the latest path for visualization or other consumers.

No persistent current-path state is required inside the planner.

Responsibility Boundary

Voxel Costmap
"Where can the vehicle travel?"
        ↓
Global Planner
"Find a path from start to goal."
        ↓
Mission / Navigation Layer
"Decide when another path is needed."

The planner owns:

A* search

planner-specific search state

path reconstruction

line-of-sight refinement

It does not own:

mapping

inflation

dynamic collision checking

path invalidation

replanning policy

local trajectory optimization or control