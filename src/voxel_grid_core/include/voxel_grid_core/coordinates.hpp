#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace voxel_grid_core
{
inline constexpr int block_size = 8;
inline constexpr std::size_t block_volume = 512;

struct Index
{
  std::int32_t x, y, z;
};

struct Point
{
  double x, y, z;
};

inline std::int32_t block_coordinate(std::int32_t voxel)
{
  const auto quotient = voxel / block_size;
  return quotient - (voxel % block_size < 0 ? 1 : 0);
}

inline int local_coordinate(std::int32_t voxel)
{
  const int remainder = voxel % block_size;
  return remainder < 0 ? remainder + block_size : remainder;
}

inline Index block_index(Index voxel)
{
  return {block_coordinate(voxel.x), block_coordinate(voxel.y), block_coordinate(voxel.z)};
}

inline std::size_t local_index(Index voxel)
{
  return local_coordinate(voxel.x) + block_size *
    (local_coordinate(voxel.y) + block_size * local_coordinate(voxel.z));
}

class Geometry
{
public:
  Geometry(double resolution, Point origin) : resolution_(resolution), origin_(origin)
  {
    if (!std::isfinite(resolution) || resolution <= 0 ||
      !std::isfinite(origin.x) || !std::isfinite(origin.y) || !std::isfinite(origin.z))
    {
      throw std::invalid_argument("Voxel geometry must be finite with positive resolution");
    }
  }

  Index world_to_voxel(Point point) const
  {
    return {coordinate(point.x, origin_.x), coordinate(point.y, origin_.y),
      coordinate(point.z, origin_.z)};
  }

  Point voxel_center(Index voxel) const
  {
    return {origin_.x + (static_cast<double>(voxel.x) + 0.5) * resolution_,
      origin_.y + (static_cast<double>(voxel.y) + 0.5) * resolution_,
      origin_.z + (static_cast<double>(voxel.z) + 0.5) * resolution_};
  }

  double resolution() const {return resolution_;}
  Point origin() const {return origin_;}

private:
  std::int32_t coordinate(double value, double origin) const
  {
    if (!std::isfinite(value)) {
      throw std::invalid_argument("World coordinate must be finite");
    }
    const double index = std::floor((value - origin) / resolution_);
    if (!std::isfinite(index) || index < std::numeric_limits<std::int32_t>::min() ||
      index > std::numeric_limits<std::int32_t>::max())
    {
      throw std::out_of_range("World coordinate exceeds voxel index range");
    }
    return static_cast<std::int32_t>(index);
  }

  double resolution_;
  Point origin_;
};
}  // namespace voxel_grid_core
