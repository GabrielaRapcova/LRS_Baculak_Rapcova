#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>
#include "astar_planner/search.hpp"

using namespace astar_planner;

uav_navigation_msgs::msg::VoxelCostmap createFreeCostmap()
{
  uav_navigation_msgs::msg::VoxelCostmap map;
  map.header.frame_id = "map";
  map.resolution = 1;
  VoxelBlock block;
  block.occupancy.fill(0);
  map.blocks.push_back(block);
  return map;
}

void setVoxelCost(
  uav_navigation_msgs::msg::VoxelCostmap & map, int x, int y, int z, uint8_t cost)
{
  map.blocks[0].occupancy[x + 8 * (y + 8 * z)] = cost;
}

TEST(Search, OpenSpaceAndExactEndpoints)
{
  VoxelGrid grid(createFreeCostmap());
  auto result = findPath(grid, {0.6, 0.7, 0.8}, {6.6, 5.7, 4.8}, SearchLimits{});
  ASSERT_EQ(result.status, SUCCESS);
  ASSERT_EQ(result.path.size(), 2u);
  EXPECT_DOUBLE_EQ(result.path.front().x, 0.6);
  EXPECT_DOUBLE_EQ(result.path.back().z, 4.8);
  EXPECT_GT(result.raw_point_count, result.path.size());
}

TEST(Search, AltitudeChangingDetour)
{
  auto map = createFreeCostmap();
  for (int y = 0; y < 8; ++y) {
    for (int z = 0; z < 3; ++z) {
      setVoxelCost(map, 3, y, z, 254);
    }
  }
  VoxelGrid grid(map);
  auto result = findPath(grid, {1.5, 3.5, 1.5}, {5.5, 3.5, 1.5}, SearchLimits{});
  ASSERT_EQ(result.status, SUCCESS);
  EXPECT_TRUE(std::any_of(result.path.begin(), result.path.end(), [](Point3 point) {
      return point.z >= 3.5;
  }));
  for (size_t index = 1; index < result.path.size(); ++index) {
    EXPECT_TRUE(isSegmentCollisionFree(grid, result.path[index - 1], result.path[index], 10000));
  }
}

TEST(Search, CannotCutCornersOrVoxelPlanes)
{
  auto map = createFreeCostmap();
  setVoxelCost(map, 2, 1, 1, 253);
  VoxelGrid grid(map);
  EXPECT_FALSE(isSegmentCollisionFree(grid, {1.5, 1.5, 1.5}, {2.5, 2.5, 1.5}, 10000));
  EXPECT_FALSE(isSegmentCollisionFree(grid, {2.0, 1.2, 1.5}, {2.0, 1.8, 1.5}, 10000));
  EXPECT_FALSE(isSegmentCollisionFree(grid, {1.5, 1.5, 1.5}, {2.0, 1.5, 1.5}, 10000));
  EXPECT_FALSE(isSegmentCollisionFree(grid, {2.0, 1.5, 1.5}, {1.5, 1.5, 1.5}, 10000));
}

TEST(Search, NoPathUnknownAndInvalidCoordinates)
{
  auto map = createFreeCostmap();
  for (int y = 0; y < 8; ++y) {
    for (int z = 0; z < 8; ++z) {
      setVoxelCost(map, 3, y, z, 254);
    }
  }
  VoxelGrid grid(map);
  EXPECT_EQ(findPath(grid, {1.5, 3.5, 1.5}, {5.5, 3.5, 1.5}, SearchLimits{}).status, NO_PATH);
  EXPECT_EQ(findPath(grid, {-1, 0, 0}, {5, 3, 1}, SearchLimits{}).status, INVALID_START);
  EXPECT_EQ(findPath(grid, {1.5, 1.5, 1.5}, {INFINITY, 0, 0}, SearchLimits{}).status, INVALID_GOAL);
}

TEST(Search, LimitsCancellationAndRepeatedRequests)
{
  VoxelGrid grid(createFreeCostmap());
  SearchLimits limits;
  limits.max_expansions = 1;
  EXPECT_EQ(findPath(grid, {0.5, 0.5, 0.5}, {7.5, 7.5, 7.5}, limits).status, SEARCH_LIMIT);
  EXPECT_EQ(findPath(grid, {0.5, 0.5, 0.5}, {7.5, 7.5, 7.5}, SearchLimits{}, []() {
      return true;
  }).status, CANCELLED);
  for (int iteration = 0; iteration < 2; ++iteration) {
    EXPECT_EQ(findPath(grid, {1.5, 1.5, 1.5}, {1.5, 1.5, 1.5}, SearchLimits{}).status, SUCCESS);
  }
}

TEST(Search, NegativeBlockIndicesAndGeometry)
{
  auto map = createFreeCostmap();
  map.blocks[0].x = -1;
  map.origin.x = 10;
  map.resolution = 0.5;
  VoxelGrid grid(map);
  EXPECT_EQ(grid.getCost({-1, 0, 0}), 0);
  EXPECT_EQ(findPath(grid, {6.25, 0.25, 0.25}, {9.75, 2.25, 2.25}, SearchLimits{}).status, SUCCESS);
}

TEST(Search, ThreeAxisCornerAndUnknownDiagonal)
{
  auto map = createFreeCostmap();
  setVoxelCost(map, 2, 1, 1, 254);
  VoxelGrid grid(map);
  EXPECT_FALSE(isSegmentCollisionFree(grid, {1.5, 1.5, 1.5}, {2.5, 2.5, 2.5}, 10000));
  map.blocks[0].occupancy.fill(255);
  setVoxelCost(map, 1, 1, 1, 0);
  setVoxelCost(map, 2, 2, 2, 0);
  EXPECT_EQ(findPath(VoxelGrid(map), {1.5, 1.5, 1.5}, {2.5, 2.5, 2.5}, SearchLimits{}).status,
    NO_PATH);
}

TEST(Search, NodeTimeAndTraversalLimits)
{
  VoxelGrid grid(createFreeCostmap());
  SearchLimits limits;
  limits.max_search_nodes = 1;
  EXPECT_EQ(findPath(grid, {1.5, 1.5, 1.5}, {6.5, 6.5, 6.5}, limits).status, SEARCH_LIMIT);
  limits = SearchLimits{};
  limits.max_planning_time = 1e-12;
  EXPECT_EQ(findPath(grid, {1.5, 1.5, 1.5}, {6.5, 6.5, 6.5}, limits).status, SEARCH_LIMIT);
  EXPECT_THROW(isSegmentCollisionFree(grid, {1.5, 1.5, 1.5}, {6.5, 6.5, 6.5}, 1),
    SearchInterrupted);
}
