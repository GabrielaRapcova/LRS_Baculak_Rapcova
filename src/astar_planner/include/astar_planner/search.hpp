#pragma once
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <uav_navigation_msgs/msg/voxel_costmap.hpp>
#include <voxel_grid_core/coordinates.hpp>

namespace astar_planner
{
using Point = voxel_grid_core::Point;
using Index = std::array<int32_t, 3>;
using Block = uav_navigation_msgs::msg::VoxelBlock;
struct Hash {
  size_t operator()(const Index & p) const {
    size_t h = 0;
    for (auto value : p) {h ^= std::hash<int32_t>{}(value) + 0x9e3779b9 + (h << 6) + (h >> 2);}
    return h;
  }
};
struct Grid {
  voxel_grid_core::Geometry geometry;
  std::string frame;
  uint64_t map_id, sequence;
  std::unordered_map<Index, std::array<uint8_t, 512>, Hash> blocks;
  explicit Grid(const uav_navigation_msgs::msg::VoxelCostmap & map)
  : geometry(map.resolution, {map.origin.x, map.origin.y, map.origin.z}),
    frame(map.header.frame_id), map_id(map.map_id), sequence(map.sequence)
  {
    if (frame.empty()) {throw std::invalid_argument("Costmap has an empty frame");}
    for (const auto & b : map.blocks) {
      for (auto v : {b.x, b.y, b.z}) {
        if (v < INT32_MIN / 8 || v > INT32_MAX / 8) {throw std::invalid_argument("Block index out of range");}
      }
      if (!blocks.emplace(Index{b.x, b.y, b.z}, b.occupancy).second) {
        throw std::invalid_argument("Duplicate costmap block");
      }
    }
  }
  uint8_t cost(Index p) const {
    const voxel_grid_core::Index voxel{p[0], p[1], p[2]};
    auto b = voxel_grid_core::block_index(voxel);
    auto found = blocks.find({b.x, b.y, b.z});
    return found == blocks.end() ? 255 : found->second[voxel_grid_core::local_index(voxel)];
  }
  bool free(Index p) const {return cost(p) < 253;}
  Index index(Point p) const {auto i = geometry.world_to_voxel(p); return {i.x, i.y, i.z};}
  Point center(Index p) const {return geometry.voxel_center({p[0], p[1], p[2]});}
};
inline double distance(Point a, Point b) {
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}
struct Limits {
  size_t expansions = 200000, nodes = 500000, los_voxels = 100000;
  double seconds = 10, cost_weight = 1;
};
enum Status : uint8_t {SUCCESS=0, NO_MAP=1, INVALID_START=2, INVALID_GOAL=3,
  NO_PATH=4, SEARCH_LIMIT=5, CANCELLED=6, INTERNAL_ERROR=7};
struct Result {
  Status status = NO_PATH;
  std::string message;
  std::vector<Point> path;
  size_t expanded = 0, raw_count = 0;
  double seconds = 0;
};
struct Interrupted {Status status;};

// Conservative supercover traversal: test all cells touched by edges/corners,
// including both sides of a segment that lies exactly on a voxel plane.
inline bool visible(const Grid & grid, Point start, Point end, size_t limit,
  const std::function<void()> & check = [](){})
{
  auto origin = grid.geometry.origin();
  const double resolution = grid.geometry.resolution();
  std::array<double, 3> a{(start.x-origin.x)/resolution, (start.y-origin.y)/resolution,
    (start.z-origin.z)/resolution};
  std::array<double, 3> b{(end.x-origin.x)/resolution, (end.y-origin.y)/resolution,
    (end.z-origin.z)/resolution};
  Index cell = grid.index(start);
  grid.index(end);  // Validate the entire coordinate range before traversal.
  std::array<int, 3> step{};
  std::array<double, 3> next{}, delta{};
  unsigned stationary_planes = 0, initial_planes = 0;
  for (int axis=0; axis<3; ++axis) {
    double d = b[axis]-a[axis];
    bool plane = std::abs(a[axis]-std::round(a[axis])) < 1e-10;
    if (plane) {initial_planes |= 1u << axis;}
    if (d == 0) {
      next[axis] = delta[axis] = std::numeric_limits<double>::infinity();
      if (plane) {stationary_planes |= 1u << axis;}
    } else {
      step[axis] = d > 0 ? 1 : -1;
      const double boundary = static_cast<double>(cell[axis]) + (d > 0 ? 1 : 0);
      next[axis] = (boundary-a[axis])/d;
      delta[axis] = 1/std::abs(d);
    }
  }
  size_t visited=0;
  auto test = [&](Index base, unsigned planes) {
    for (unsigned subset=0; subset<8; ++subset) {
      if (subset & ~planes) {continue;}
      if (++visited > limit) {throw Interrupted{SEARCH_LIMIT};}
      if ((visited & 255) == 0) {check();}
      Index p=base;
      for (int axis=0; axis<3; ++axis) {
        if (subset & (1u << axis)) {
          if (p[axis] == INT32_MIN) {return false;}
          --p[axis];
        }
      }
      if (!grid.free(p)) {return false;}
    }
    return true;
  };
  if (!test(cell, initial_planes)) {return false;}
  while (true) {
    double t = std::min({next[0], next[1], next[2]});
    if (!std::isfinite(t) || t > 1 + 1e-10) {break;}
    unsigned crossing=0;
    for (int axis=0; axis<3; ++axis) {if (std::abs(next[axis]-t) < 1e-10) {crossing |= 1u << axis;}}
    for (unsigned subset=1; subset<8; ++subset) {
      if (subset & ~crossing) {continue;}
      Index neighbor=cell;
      for (int axis=0; axis<3; ++axis) {
        if (subset & (1u << axis)) {
          int64_t value = int64_t(neighbor[axis])+step[axis];
          if (value < INT32_MIN || value > INT32_MAX) {return false;}
          neighbor[axis]=static_cast<int32_t>(value);
        }
      }
      if (!test(neighbor, stationary_planes)) {return false;}
    }
    for (int axis=0; axis<3; ++axis) {
      if (crossing & (1u << axis)) {cell[axis] += step[axis]; next[axis] += delta[axis];}
    }
  }
  return true;
}

inline Result search(const Grid & grid, Point start, Point goal, const Limits & limits,
  const std::function<bool()> & cancelled = [](){return false;},
  const std::function<void(size_t)> & feedback = [](size_t){})
{
  const auto begin=std::chrono::steady_clock::now();
  Result result;
  auto check=[&]() {
    if (cancelled()) {throw Interrupted{CANCELLED};}
    if (std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count() >= limits.seconds) {
      throw Interrupted{SEARCH_LIMIT};
    }
  };
  try {
    Index s, g;
    try {s=grid.index(start);} catch (const std::exception &) {throw Interrupted{INVALID_START};}
    try {g=grid.index(goal);} catch (const std::exception &) {throw Interrupted{INVALID_GOAL};}
    if (!grid.free(s) || !visible(grid,start,grid.center(s),limits.los_voxels,check)) {throw Interrupted{INVALID_START};}
    if (!grid.free(g) || !visible(grid,grid.center(g),goal,limits.los_voxels,check)) {throw Interrupted{INVALID_GOAL};}
    struct Node {Index index; double g; size_t parent; bool closed;};
    struct Entry {
      double f, g; size_t node;
      bool operator<(const Entry & other) const {return f > other.f || (f == other.f && g < other.g);}
    };
    const size_t none=std::numeric_limits<size_t>::max();
    std::vector<Node> nodes;
    std::unordered_map<Index,size_t,Hash> lookup;
    std::priority_queue<Entry> open;
    nodes.push_back({s,0,none,false}); lookup.emplace(s,0);
    open.push({distance(grid.center(s),grid.center(g)),0,0});
    size_t found=none;
    while (!open.empty()) {
      check();
      auto entry=open.top(); open.pop();
      if (nodes[entry.node].closed || entry.g != nodes[entry.node].g) {continue;}
      auto current=nodes[entry.node];
      if (current.index == g) {found=entry.node; break;}
      if (result.expanded >= limits.expansions) {throw Interrupted{SEARCH_LIMIT};}
      nodes[entry.node].closed=true; ++result.expanded;
      if (result.expanded % 256 == 0) {feedback(result.expanded);}
      for (int z=-1; z<=1; ++z) {for (int y=-1; y<=1; ++y) {for (int x=-1; x<=1; ++x) {
        if (!x && !y && !z) {continue;}
        Index target;
        std::array<int,3> d{x,y,z};
        bool valid=true;
        for (int axis=0; axis<3; ++axis) {
          int64_t v=int64_t(current.index[axis])+d[axis];
          if (v < INT32_MIN || v > INT32_MAX) {valid=false; break;}
          target[axis]=static_cast<int32_t>(v);
        }
        if (!valid || !grid.free(target)) {continue;}
        auto prior=lookup.find(target);
        if (prior != lookup.end() && nodes[prior->second].closed) {continue;}
        // Every subset of moved axes is touched by this center-to-center edge.
        unsigned moved=(x ? 1 : 0) | (y ? 2 : 0) | (z ? 4 : 0);
        for (unsigned subset=1; subset<8 && valid; ++subset) {
          if (subset & ~moved) {continue;}
          auto intermediate=current.index;
          for (int axis=0; axis<3; ++axis) {if (subset & (1u<<axis)) {intermediate[axis]+=d[axis];}}
          valid=grid.free(intermediate);
        }
        if (!valid) {continue;}
        double length=std::sqrt(double(x*x+y*y+z*z))*grid.geometry.resolution();
        // Symmetric nonnegative cost penalty keeps Euclidean h admissible/consistent.
        double penalty=(double(grid.cost(current.index))+grid.cost(target))/(2*252.0);
        double candidate=current.g + length*(1+limits.cost_weight*penalty);
        size_t id;
        if (prior == lookup.end()) {
          if (nodes.size() >= limits.nodes) {throw Interrupted{SEARCH_LIMIT};}
          id=nodes.size(); nodes.push_back({target,candidate,entry.node,false}); lookup.emplace(target,id);
        } else {
          id=prior->second;
          if (candidate >= nodes[id].g) {continue;}
          nodes[id].g=candidate; nodes[id].parent=entry.node;
        }
        open.push({candidate+distance(grid.center(target),grid.center(g)),candidate,id});
      }}}
    }
    if (found == none) {throw Interrupted{NO_PATH};}
    std::vector<Point> centers;
    for (size_t id=found; id!=none; id=nodes[id].parent) {check(); centers.push_back(grid.center(nodes[id].index));}
    std::reverse(centers.begin(),centers.end());
    std::vector<Point> raw{start};
    for (auto point : centers) {if (distance(raw.back(),point)>1e-12) {raw.push_back(point);}}
    if (distance(raw.back(),goal)>1e-12) {raw.push_back(goal);}
    result.raw_count=raw.size();
    result.path.push_back(raw[0]);
    for (size_t anchor=0; anchor+1<raw.size();) {
      check();
      size_t next=raw.size()-1;
      while (next>anchor+1 && !visible(grid,raw[anchor],raw[next],limits.los_voxels,check)) {--next; check();}
      if (!visible(grid,raw[anchor],raw[next],limits.los_voxels,check)) {throw Interrupted{INTERNAL_ERROR};}
      result.path.push_back(raw[next]); anchor=next;
    }
    // Recheck returned segments rather than assuming refinement preserved validity.
    for (size_t i=1; i<result.path.size(); ++i) {
      check();
      if (!visible(grid,result.path[i-1],result.path[i],limits.los_voxels,check)) {throw Interrupted{INTERNAL_ERROR};}
    }
    check(); result.status=SUCCESS; result.message="Path found";
  } catch (const Interrupted & error) {
    result.status=error.status; result.path.clear();
    switch (error.status) {
      case INVALID_START: result.message="Start is invalid, unknown, blocked, or touches blocked space"; break;
      case INVALID_GOAL: result.message="Goal is invalid, unknown, blocked, or touches blocked space"; break;
      case NO_PATH: result.message="No path exists"; break;
      case SEARCH_LIMIT: result.message="Search time, node, expansion or traversal limit reached"; break;
      case CANCELLED: result.message="Planning cancelled"; break;
      default: result.message="Path validation failed";
    }
  }
  result.seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  return result;
}
}  // namespace astar_planner
