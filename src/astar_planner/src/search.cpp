#include "astar_planner/search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace astar_planner
{

size_t VoxelIndexHash::operator()(const VoxelIndex & index) const
{
  size_t seed = 0;
  for (auto coordinate : index) {
    seed ^= std::hash<int32_t>{}(coordinate) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }
  return seed;
}

VoxelGrid::VoxelGrid(const uav_navigation_msgs::msg::VoxelCostmap & map)
: geometry(map.resolution, {map.origin.x, map.origin.y, map.origin.z}),
  frame_id(map.header.frame_id), map_id(map.map_id), sequence(map.sequence)
{
  if (frame_id.empty()) {
    throw std::invalid_argument("Costmap has an empty frame");
  }
  for (const auto & block : map.blocks) {
    for (auto coordinate : {block.x, block.y, block.z}) {
      if (coordinate < INT32_MIN / 8 || coordinate > INT32_MAX / 8) {
        throw std::invalid_argument("Block index out of range");
      }
    }
    if (!blocks.emplace(VoxelIndex{block.x, block.y, block.z}, block.occupancy).second) {
      throw std::invalid_argument("Duplicate costmap block");
    }
  }
}

uint8_t VoxelGrid::getCost(VoxelIndex index) const
{
  const voxel_grid_core::Index voxel{index[0], index[1], index[2]};
  auto block_index = voxel_grid_core::block_index(voxel);
  auto block = blocks.find({block_index.x, block_index.y, block_index.z});
  if (block == blocks.end()) {
    return 255;
  }
  return block->second[voxel_grid_core::local_index(voxel)];
}

bool VoxelGrid::isTraversable(VoxelIndex index) const
{
  return getCost(index) < 253;
}

VoxelIndex VoxelGrid::worldToVoxel(Point3 point) const
{
  auto index = geometry.world_to_voxel(point);
  return {index.x, index.y, index.z};
}

Point3 VoxelGrid::getVoxelCentre(VoxelIndex index) const
{
  return geometry.voxel_center({index[0], index[1], index[2]});
}

double calculateDistance(Point3 start, Point3 end)
{
  return std::hypot(std::hypot(start.x - end.x, start.y - end.y), start.z - end.z);
}

bool isSegmentCollisionFree(
  const VoxelGrid & grid, Point3 start, Point3 end, size_t max_voxels,
  const std::function<void()> & check_interruption)
{
  auto map_origin = grid.geometry.origin();
  const double resolution = grid.geometry.resolution();
  std::array<double, 3> start_grid{(start.x - map_origin.x) / resolution,
    (start.y - map_origin.y) / resolution, (start.z - map_origin.z) / resolution};
  std::array<double, 3> end_grid{(end.x - map_origin.x) / resolution,
    (end.y - map_origin.y) / resolution, (end.z - map_origin.z) / resolution};
  VoxelIndex current_voxel = grid.worldToVoxel(start);
  // Validate the entire coordinate range before traversal.
  grid.worldToVoxel(end);
  std::array<int, 3> axis_step{};
  std::array<double, 3> next_crossing{};
  std::array<double, 3> crossing_interval{};
  unsigned stationary_planes = 0;
  unsigned initial_planes = 0;
  for (int axis = 0; axis < 3; ++axis) {
    double axis_distance = end_grid[axis] - start_grid[axis];
    bool on_voxel_plane = std::abs(start_grid[axis] - std::round(start_grid[axis])) < 1e-10;
    if (on_voxel_plane) {
      initial_planes |= 1u << axis;
    }
    if (axis_distance == 0) {
      next_crossing[axis] = crossing_interval[axis] = std::numeric_limits<double>::infinity();
      if (on_voxel_plane) {
        stationary_planes |= 1u << axis;
      }
    } else {
      axis_step[axis] = axis_distance > 0 ? 1 : -1;
      const double boundary = static_cast<double>(current_voxel[axis]) +
        (axis_distance > 0 ? 1 : 0);
      next_crossing[axis] = (boundary - start_grid[axis]) / axis_distance;
      crossing_interval[axis] = 1 / std::abs(axis_distance);
    }
  }

  size_t visited_voxels = 0;
  auto areTouchedVoxelsTraversable = [&](VoxelIndex base_voxel, unsigned plane_mask) {
      for (unsigned axis_subset = 0; axis_subset < 8; ++axis_subset) {
        if (axis_subset & ~plane_mask) {
          continue;
        }
        if (++visited_voxels > max_voxels) {
          throw SearchInterrupted{SEARCH_LIMIT};
        }
        if ((visited_voxels & 255) == 0) {
          check_interruption();
        }
        VoxelIndex touched_voxel = base_voxel;
        for (int axis = 0; axis < 3; ++axis) {
          if (axis_subset & (1u << axis)) {
            if (touched_voxel[axis] == INT32_MIN) {
              return false;
            }
            --touched_voxel[axis];
          }
        }
        if (!grid.isTraversable(touched_voxel)) {
          return false;
        }
      }
      return true;
    };

  if (!areTouchedVoxelsTraversable(current_voxel, initial_planes)) {
    return false;
  }

  while (true) {
    double crossing_fraction = std::min({next_crossing[0], next_crossing[1], next_crossing[2]});
    if (!std::isfinite(crossing_fraction) || crossing_fraction > 1 + 1e-10) {
      break;
    }
    unsigned crossing_axes = 0;
    for (int axis = 0; axis < 3; ++axis) {
      if (std::abs(next_crossing[axis] - crossing_fraction) < 1e-10) {
        crossing_axes |= 1u << axis;
      }
    }
    for (unsigned axis_subset = 1; axis_subset < 8; ++axis_subset) {
      if (axis_subset & ~crossing_axes) {
        continue;
      }
      VoxelIndex neighbor_voxel = current_voxel;
      for (int axis = 0; axis < 3; ++axis) {
        if (axis_subset & (1u << axis)) {
          int64_t next_coordinate = int64_t(neighbor_voxel[axis]) + axis_step[axis];
          if (next_coordinate < INT32_MIN || next_coordinate > INT32_MAX) {
            return false;
          }
          neighbor_voxel[axis] = static_cast<int32_t>(next_coordinate);
        }
      }
      if (!areTouchedVoxelsTraversable(neighbor_voxel, stationary_planes)) {
        return false;
      }
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (crossing_axes & (1u << axis)) {
        current_voxel[axis] += axis_step[axis];
        next_crossing[axis] += crossing_interval[axis];
      }
    }
  }
  return true;
}

SearchResult findPath(
  const VoxelGrid & grid, Point3 start, Point3 goal, const SearchLimits & limits,
  const std::function<bool()> & should_cancel,
  const std::function<void(size_t)> & publish_feedback)
{
  const auto search_started = std::chrono::steady_clock::now();
  SearchResult result;
  auto checkInterruption = [&]() {
      if (should_cancel()) {
        throw SearchInterrupted{CANCELLED};
      }
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
        search_started).count() >= limits.max_planning_time)
      {
        throw SearchInterrupted{SEARCH_LIMIT};
      }
    };

  try {
    VoxelIndex start_voxel, goal_voxel;
    try {
      start_voxel = grid.worldToVoxel(start);
    } catch (const std::exception &) {
      throw SearchInterrupted{INVALID_START};
    }
    try {
      goal_voxel = grid.worldToVoxel(goal);
    } catch (const std::exception &) {
      throw SearchInterrupted{INVALID_GOAL};
    }
    if (!grid.isTraversable(start_voxel) || !isSegmentCollisionFree(grid, start,
        grid.getVoxelCentre(start_voxel), limits.max_line_of_sight_voxels, checkInterruption))
    {
      throw SearchInterrupted{INVALID_START};
    }
    if (!grid.isTraversable(goal_voxel) || !isSegmentCollisionFree(grid,
        grid.getVoxelCentre(goal_voxel), goal, limits.max_line_of_sight_voxels, checkInterruption))
    {
      throw SearchInterrupted{INVALID_GOAL};
    }

    struct SearchNode
    {
      VoxelIndex index;
      double path_cost;
      size_t parent_index;
      bool is_closed;
    };

    struct OpenEntry
    {
      double estimated_total_cost;
      double path_cost;
      size_t node_index;
      bool operator<(const OpenEntry & other) const
      {
        return estimated_total_cost > other.estimated_total_cost ||
               (estimated_total_cost == other.estimated_total_cost && path_cost < other.path_cost);
      }
    };

    const size_t no_parent = std::numeric_limits<size_t>::max();
    std::vector<SearchNode> nodes;
    std::unordered_map<VoxelIndex, size_t, VoxelIndexHash> node_lookup;
    std::priority_queue<OpenEntry> open_set;
    nodes.push_back({start_voxel, 0, no_parent, false});
    node_lookup.emplace(start_voxel, 0);
    open_set.push({calculateDistance(grid.getVoxelCentre(start_voxel),
          grid.getVoxelCentre(goal_voxel)), 0, 0});

    size_t goal_node_index = no_parent;
    while (!open_set.empty()) {
      checkInterruption();
      auto entry = open_set.top();
      open_set.pop();
      if (nodes[entry.node_index].is_closed ||
        entry.path_cost != nodes[entry.node_index].path_cost)
      {
        continue;
      }
      auto current_node = nodes[entry.node_index];
      if (current_node.index == goal_voxel) {
        goal_node_index = entry.node_index;
        break;
      }
      if (result.expanded_nodes >= limits.max_expansions) {
        throw SearchInterrupted{SEARCH_LIMIT};
      }
      nodes[entry.node_index].is_closed = true;
      ++result.expanded_nodes;
      if (result.expanded_nodes % 256 == 0) {
        publish_feedback(result.expanded_nodes);
      }
      for (int offset_z = -1; offset_z <= 1; ++offset_z) {
        for (int offset_y = -1; offset_y <= 1; ++offset_y) {
          for (int offset_x = -1; offset_x <= 1; ++offset_x) {
            if (!offset_x && !offset_y && !offset_z) {
              continue;
            }
            VoxelIndex neighbor_voxel;
            std::array<int, 3> offset{offset_x, offset_y, offset_z};
            bool edge_valid = true;
            for (int axis = 0; axis < 3; ++axis) {
              int64_t next_coordinate = int64_t(current_node.index[axis]) + offset[axis];
              if (next_coordinate < INT32_MIN || next_coordinate > INT32_MAX) {
                edge_valid = false;
                break;
              }
              neighbor_voxel[axis] = static_cast<int32_t>(next_coordinate);
            }
            if (!edge_valid || !grid.isTraversable(neighbor_voxel)) {
              continue;
            }
            auto existing_node = node_lookup.find(neighbor_voxel);
            if (existing_node != node_lookup.end() && nodes[existing_node->second].is_closed) {
              continue;
            }
            // Every subset of moved axes is touched by this center-to-center edge.
            unsigned moved_axes = (offset_x ? 1 : 0) | (offset_y ? 2 : 0) | (offset_z ? 4 : 0);
            for (unsigned axis_subset = 1; axis_subset < 8 && edge_valid; ++axis_subset) {
              if (axis_subset & ~moved_axes) {
                continue;
              }
              auto touched_voxel = current_node.index;
              for (int axis = 0; axis < 3; ++axis) {
                if (axis_subset & (1u << axis)) {
                  touched_voxel[axis] += offset[axis];
                }
              }
              edge_valid = grid.isTraversable(touched_voxel);
            }
            if (!edge_valid) {
              continue;
            }
            double edge_length = std::sqrt(double(offset_x * offset_x + offset_y * offset_y +
                offset_z * offset_z)) * grid.geometry.resolution();
            // Symmetric nonnegative cost penalty keeps Euclidean h admissible/consistent.
            double cost_penalty = (double(grid.getCost(current_node.index)) +
              grid.getCost(neighbor_voxel)) / (2 * 252.0);
            double candidate_cost = current_node.path_cost + edge_length *
              (1 + limits.cost_weight * cost_penalty);
            size_t node_index;
            if (existing_node == node_lookup.end()) {
              if (nodes.size() >= limits.max_search_nodes) {
                throw SearchInterrupted{SEARCH_LIMIT};
              }
              node_index = nodes.size();
              nodes.push_back({neighbor_voxel, candidate_cost, entry.node_index, false});
              node_lookup.emplace(neighbor_voxel, node_index);
            } else {
              node_index = existing_node->second;
              if (candidate_cost >= nodes[node_index].path_cost) {
                continue;
              }
              nodes[node_index].path_cost = candidate_cost;
              nodes[node_index].parent_index = entry.node_index;
            }
            open_set.push({candidate_cost + calculateDistance(grid.getVoxelCentre(neighbor_voxel),
                  grid.getVoxelCentre(goal_voxel)), candidate_cost, node_index});
          }
        }
      }
    }

    if (goal_node_index == no_parent) {
      throw SearchInterrupted{NO_PATH};
    }

    std::vector<Point3> voxel_centres;
    for (size_t node_index = goal_node_index; node_index != no_parent;
      node_index = nodes[node_index].parent_index)
    {
      checkInterruption();
      voxel_centres.push_back(grid.getVoxelCentre(nodes[node_index].index));
    }
    std::reverse(voxel_centres.begin(), voxel_centres.end());
    std::vector<Point3> raw_path{start};
    for (auto point : voxel_centres) {
      if (calculateDistance(raw_path.back(), point) > 1e-12) {
        raw_path.push_back(point);
      }
    }
    if (calculateDistance(raw_path.back(), goal) > 1e-12) {
      raw_path.push_back(goal);
    }

    result.raw_point_count = raw_path.size();
    result.path.push_back(raw_path[0]);
    for (size_t anchor_index = 0; anchor_index + 1 < raw_path.size(); ) {
      checkInterruption();
      size_t next_index = raw_path.size() - 1;
      while (next_index > anchor_index + 1 && !isSegmentCollisionFree(grid, raw_path[anchor_index],
          raw_path[next_index], limits.max_line_of_sight_voxels, checkInterruption))
      {
        --next_index;
        checkInterruption();
      }
      if (!isSegmentCollisionFree(grid, raw_path[anchor_index], raw_path[next_index],
          limits.max_line_of_sight_voxels, checkInterruption))
      {
        throw SearchInterrupted{INTERNAL_ERROR};
      }
      result.path.push_back(raw_path[next_index]);
      anchor_index = next_index;
    }

    // Recheck returned segments rather than assuming refinement preserved validity.
    for (size_t segment_index = 1; segment_index < result.path.size(); ++segment_index) {
      checkInterruption();
      if (!isSegmentCollisionFree(grid, result.path[segment_index - 1], result.path[segment_index],
          limits.max_line_of_sight_voxels, checkInterruption))
      {
        throw SearchInterrupted{INTERNAL_ERROR};
      }
    }
    checkInterruption();
    result.status = SUCCESS;
    result.message = "Path found";
  } catch (const SearchInterrupted & error) {
    result.status = error.status;
    result.path.clear();
    switch (error.status) {
      case INVALID_START:
        result.message = "Start is invalid, unknown, blocked, or touches blocked space";
        break;
      case INVALID_GOAL:
        result.message = "Goal is invalid, unknown, blocked, or touches blocked space";
        break;
      case NO_PATH:
        result.message = "No path exists";
        break;
      case SEARCH_LIMIT:
        result.message = "Search time, node, expansion or traversal limit reached";
        break;
      case CANCELLED:
        result.message = "Planning cancelled";
        break;
      default:
        result.message = "Path validation failed";
    }
  }

  result.planning_time = std::chrono::duration<double>(std::chrono::steady_clock::now() -
      search_started).count();
  return result;
}

}  // namespace astar_planner
