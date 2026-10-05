#include <gtest/gtest.h>
#include <limits>
#include "voxel_grid_core/coordinates.hpp"

using namespace voxel_grid_core;

TEST(Coordinates, NegativeBlockBoundaries)
{
  for (int voxel = -25; voxel <= 25; ++voxel) {
    EXPECT_EQ(block_coordinate(voxel) * block_size + local_coordinate(voxel), voxel);
    EXPECT_GE(local_coordinate(voxel), 0);
    EXPECT_LT(local_coordinate(voxel), block_size);
  }
  EXPECT_EQ(block_coordinate(-1), -1);
  EXPECT_EQ(block_coordinate(-8), -1);
  EXPECT_EQ(block_coordinate(-9), -2);
  EXPECT_EQ(local_index({-1, -1, -1}), 511u);
  EXPECT_EQ(local_index({8, 8, 8}), 0u);
  EXPECT_EQ(local_index({1, 2, 3}), 209u);
}

TEST(Coordinates, ExtremeIndices)
{
  for (auto value : {std::numeric_limits<std::int32_t>::min(),
      std::numeric_limits<std::int32_t>::max()})
  {
    EXPECT_EQ(static_cast<std::int64_t>(block_coordinate(value)) * block_size +
      local_coordinate(value), value);
  }
}

TEST(Coordinates, WorldBoundariesAndCenters)
{
  Geometry geometry(0.5, {10, -2, 1});
  auto index = geometry.world_to_voxel({9.99, -2, 1.5});
  EXPECT_EQ(index.x, -1);
  EXPECT_EQ(index.y, 0);
  EXPECT_EQ(index.z, 1);
  for (int x = -17; x < 17; ++x) {
    auto recovered = geometry.world_to_voxel(geometry.voxel_center({x, -x, 3}));
    EXPECT_EQ(recovered.x, x);
    EXPECT_EQ(recovered.y, -x);
    EXPECT_EQ(recovered.z, 3);
  }
}

TEST(Coordinates, RejectsInvalidGeometryAndPoints)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_THROW((Geometry(0, {0, 0, 0})), std::invalid_argument);
  EXPECT_THROW((Geometry(-1, {0, 0, 0})), std::invalid_argument);
  EXPECT_THROW((Geometry(nan, {0, 0, 0})), std::invalid_argument);
  EXPECT_THROW((Geometry(1, {inf, 0, 0})), std::invalid_argument);
  Geometry geometry(0.5, {0, 0, 0});
  EXPECT_THROW(geometry.world_to_voxel({nan, 0, 0}), std::invalid_argument);
  EXPECT_THROW(geometry.world_to_voxel({1e30, 0, 0}), std::out_of_range);
}
