#ifndef ASTAR_PLANNER__SEARCH_HPP_
#define ASTAR_PLANNER__SEARCH_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "uav_navigation_msgs/msg/voxel_costmap.hpp"
#include "voxel_grid_core/coordinates.hpp"

namespace astar_planner
{
using Point3 = voxel_grid_core::Point;
using VoxelIndex = std::array<int32_t, 3>;
using VoxelBlock = uav_navigation_msgs::msg::VoxelBlock;

struct VoxelIndexHash
{
  size_t operator()(const VoxelIndex & index) const;
};

/// Immutable search input; missing blocks are unknown and cannot be traversed.
struct VoxelGrid
{
  explicit VoxelGrid(const uav_navigation_msgs::msg::VoxelCostmap & map);
  uint8_t getCost(VoxelIndex index) const;
  bool isTraversable(VoxelIndex index) const;
  VoxelIndex worldToVoxel(Point3 point) const;
  Point3 getVoxelCentre(VoxelIndex index) const;

  voxel_grid_core::Geometry geometry;
  std::string frame_id;
  uint64_t map_id;
  uint64_t sequence;
  std::unordered_map<VoxelIndex, std::array<uint8_t, 512>, VoxelIndexHash> blocks;
};

struct SearchLimits
{
  size_t max_expansions = 200000;
  size_t max_search_nodes = 500000;
  size_t max_line_of_sight_voxels = 100000;
  double max_planning_time = 10;
  double cost_weight = 1;
};

// Numeric values match the ComputePath3D action result.
enum SearchStatus : uint8_t
{
  SUCCESS = 0, NO_MAP = 1, INVALID_START = 2, INVALID_GOAL = 3,
  NO_PATH = 4, SEARCH_LIMIT = 5, CANCELLED = 6, INTERNAL_ERROR = 7
};

struct SearchResult
{
  SearchStatus status = NO_PATH;
  std::string message;
  std::vector<Point3> path;
  size_t expanded_nodes = 0;
  size_t raw_point_count = 0;
  double planning_time = 0;
};

struct SearchInterrupted
{
  SearchStatus status;
};

double calculateDistance(Point3 start, Point3 end);

/**
 * @brief Check every voxel touched by a segment, including edge/plane contacts.
 * @throws SearchInterrupted if the traversal budget or interruption check stops it.
 */
bool isSegmentCollisionFree(
  const VoxelGrid & grid, Point3 start, Point3 end, size_t max_voxels,
  const std::function<void()> & check_interruption = [] () {});

/**
 * @brief Run 26-connected A* and collision-checked line-of-sight simplification.
 *
 * Search state is local to this call. Cancellation and limits return a status
 * with an empty path. Simplification preserves collision freedom, not weighted cost.
 */
SearchResult findPath(
  const VoxelGrid & grid, Point3 start, Point3 goal, const SearchLimits & limits,
  const std::function<bool()> & should_cancel = [] () {return false;},
  const std::function<void(size_t)> & publish_feedback = [] (size_t) {});

}  // namespace astar_planner

#endif  // ASTAR_PLANNER__SEARCH_HPP_
