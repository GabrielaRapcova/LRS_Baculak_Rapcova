#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <uav_navigation_msgs/msg/voxel_map.hpp>
#include <uav_navigation_msgs/msg/voxel_map_update.hpp>
#include <uav_navigation_msgs/msg/voxel_costmap.hpp>
#include <uav_navigation_msgs/srv/check_collision.hpp>
#include <voxel_grid_core/coordinates.hpp>

namespace vg = voxel_grid_core;
using Block = uav_navigation_msgs::msg::VoxelBlock;
using Key = std::tuple<int32_t, int32_t, int32_t>;
using Blocks = std::map<Key, Block>;
using Map = uav_navigation_msgs::msg::VoxelMap;
using Update = uav_navigation_msgs::msg::VoxelMapUpdate;
using Costmap = uav_navigation_msgs::msg::VoxelCostmap;
struct Offset {int x, y, z; uint8_t cost;};

class VoxelCostmap : public rclcpp::Node
{
public:
  VoxelCostmap() : Node("voxel_costmap")
  {
    const double radius = declare_parameter("vehicle_radius", 0.3);
    const double tolerance = declare_parameter("position_tolerance", 0.1);
    const double margin = declare_parameter("safety_margin", 0.1);
    inflation_width_ = declare_parameter("inflation_radius", 0.8);
    scaling_ = declare_parameter("cost_scaling_factor", 3.0);
    stencil_limit_ = declare_parameter<int64_t>("max_stencil_cells", 1000000);
    const auto policy = declare_parameter<std::string>("unknown_policy", "blocked");
    for (auto value : {radius, tolerance, margin, inflation_width_, scaling_}) {
      if (!std::isfinite(value) || value < 0) {throw std::invalid_argument("Clearance parameters must be finite and nonnegative");}
    }
    clearance_ = radius + tolerance + margin;
    outer_radius_ = clearance_ + inflation_width_;
    if (!std::isfinite(clearance_) || !std::isfinite(outer_radius_) ||
      stencil_limit_ <= 0 || policy != "blocked") {
      throw std::invalid_argument("Require finite total radius, positive stencil limit, unknown_policy=blocked");
    }
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    publisher_ = create_publisher<Costmap>("costmap", qos);
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("costmap_cloud", qos);
    map_subscription_ = create_subscription<Map>("map", qos, [this](Map::ConstSharedPtr map) {snapshot(*map);});
    update_subscription_ = create_subscription<Update>("map_updates", rclcpp::QoS(100).reliable(),
      [this](Update::ConstSharedPtr update) {increment(*update);});
    service_ = create_service<uav_navigation_msgs::srv::CheckCollision>("check_collision",
      [this](const std::shared_ptr<uav_navigation_msgs::srv::CheckCollision::Request> request,
      std::shared_ptr<uav_navigation_msgs::srv::CheckCollision::Response> response) {
        response->cost = Block::UNKNOWN;
        if (synchronized_) {
          try {
            auto index = geometry_->world_to_voxel({request->point.x, request->point.y, request->point.z});
            response->cost = get(costs_, index);
          } catch (const std::exception &) {}
        }
        response->collision = response->cost >= Block::INSCRIBED;
      });
  }

private:
  static Key key(const Block & block) {return {block.x, block.y, block.z};}
  static bool unknown(const Block & block)
  {
    return std::all_of(block.occupancy.begin(), block.occupancy.end(),
      [](uint8_t value) {return value == Block::UNKNOWN;});
  }
  static Blocks validate(const std::vector<Block> & blocks)
  {
    Blocks result;
    for (const auto & block : blocks) {
      for (auto coordinate : {block.x, block.y, block.z}) {
        if (coordinate < std::numeric_limits<int32_t>::min() / 8 ||
          coordinate > std::numeric_limits<int32_t>::max() / 8) {
          throw std::invalid_argument("Block coordinate outside voxel index range");
        }
      }
      for (auto value : block.occupancy) {
        if (value != Block::FREE && value != Block::LETHAL && value != Block::UNKNOWN) {
          throw std::invalid_argument("Map provider emitted a planning cost instead of occupancy");
        }
      }
      if (!result.emplace(key(block), block).second) {throw std::invalid_argument("Duplicate block");}
    }
    return result;
  }
  static uint8_t get(const Blocks & blocks, vg::Index index)
  {
    auto b = vg::block_index(index);
    auto found = blocks.find({b.x, b.y, b.z});
    return found == blocks.end() ? Block::UNKNOWN : found->second.occupancy[vg::local_index(index)];
  }
  static vg::Index voxel(const Block & block, size_t i)
  {
    return {block.x * 8 + static_cast<int>(i % 8),
      block.y * 8 + static_cast<int>((i / 8) % 8), block.z * 8 + static_cast<int>(i / 64)};
  }
  std::vector<Offset> stencil(double resolution)
  {
    const double extent = std::ceil(outer_radius_ / resolution);
    const long double width = 2 * static_cast<long double>(extent) + 1;
    if (!std::isfinite(extent) || width * width * width > stencil_limit_) {
      throw std::invalid_argument("Inflation stencil exceeds max_stencil_cells");
    }
    const int cells = static_cast<int>(extent);
    std::vector<Offset> result;
    for (int z = -cells; z <= cells; ++z) {
      for (int y = -cells; y <= cells; ++y) {
        for (int x = -cells; x <= cells; ++x) {
          const double distance = std::sqrt(double(x)*x + double(y)*y + double(z)*z) * resolution;
          if (distance > outer_radius_ + 1e-9) {continue;}
          const uint8_t cost = distance <= clearance_ + 1e-9 ? Block::INSCRIBED :
            static_cast<uint8_t>(std::clamp(252.0 * std::exp(-scaling_ * (distance - clearance_)), 1.0, 252.0));
          result.push_back({x, y, z, cost});
        }
      }
    }
    return result;
  }
  void invalidate(const std::string & reason)
  {
    synchronized_ = false;
    RCLCPP_WARN(get_logger(), "%s; waiting for a full map snapshot", reason.c_str());
    // Replace downstream state with unknown so retained costs cannot remain usable.
    if (geometry_) {
      costs_.clear(); ++cost_sequence_; publish();
    }
  }
  void snapshot(const Map & map)
  {
    try {
      if (map.header.frame_id.empty()) {throw std::invalid_argument("Empty map frame");}
      auto geometry = std::make_unique<vg::Geometry>(map.resolution,
        vg::Point{map.origin.x, map.origin.y, map.origin.z});
      auto blocks = validate(map.blocks);
      const bool same_epoch = geometry_ && map.map_id == map_id_;
      if (same_epoch && (map.resolution != geometry_->resolution() ||
        map.origin.x != geometry_->origin().x || map.origin.y != geometry_->origin().y ||
        map.origin.z != geometry_->origin().z || map.header.frame_id != frame_)) {
        throw std::invalid_argument("Geometry changed without changing map_id");
      }
      if (same_epoch && map.sequence < map_sequence_) {return;}
      if (same_epoch && synchronized_ && map.sequence == map_sequence_) {return;}
      auto offsets = geometry_ && map.resolution == geometry_->resolution() ?
        stencil_ : stencil(map.resolution);
      geometry_ = std::move(geometry); stencil_ = std::move(offsets);
      occupancy_ = std::move(blocks); frame_ = map.header.frame_id;
      map_id_ = map.map_id; map_sequence_ = map.sequence;
      if (!same_epoch) {cost_sequence_ = 0;} else {++cost_sequence_;}
      rebuild(); synchronized_ = true; publish();
    } catch (const std::exception & error) {invalidate(error.what());}
  }
  void increment(const Update & update)
  {
    if (!synchronized_) {return;}
    if (update.map_id != map_id_ || update.header.frame_id != frame_) {
      invalidate("Map update epoch or frame mismatch"); return;
    }
    if (update.sequence <= map_sequence_) {return;}
    if (map_sequence_ == std::numeric_limits<uint64_t>::max() || update.sequence != map_sequence_ + 1) {
      invalidate("Map update sequence gap"); return;
    }
    try {
      const auto replacements = validate(update.blocks);
      for (const auto & entry : replacements) {
        if (unknown(entry.second)) {occupancy_.erase(entry.first);}
        else {occupancy_[entry.first] = entry.second;}
      }
      rebuild(); map_sequence_ = update.sequence; ++cost_sequence_;
      publish();
    } catch (const std::exception & error) {invalidate(error.what());}
  }
  void rebuild()
  {
    auto start = std::chrono::steady_clock::now();
    costs_ = occupancy_;
    for (const auto & entry : occupancy_) {
      for (size_t i = 0; i < vg::block_volume; ++i) {
        if (entry.second.occupancy[i] != Block::LETHAL) {continue;}
        auto source = voxel(entry.second, i);
        for (const auto & offset : stencil_) {
          const int64_t x = int64_t(source.x) + offset.x, y = int64_t(source.y) + offset.y,
            z = int64_t(source.z) + offset.z;
          if (x < INT32_MIN || x > INT32_MAX || y < INT32_MIN || y > INT32_MAX || z < INT32_MIN || z > INT32_MAX) {continue;}
          vg::Index target{static_cast<int32_t>(x), static_cast<int32_t>(y), static_cast<int32_t>(z)};
          if (get(occupancy_, target) != Block::FREE) {continue;}
          auto b = vg::block_index(target);
          auto & cost = costs_.at({b.x, b.y, b.z}).occupancy[vg::local_index(target)];
          cost = std::max(cost, offset.cost);
        }
      }
    }
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    RCLCPP_INFO(get_logger(), "Costmap: %zu blocks, clearance %.3f m, inflation width %.3f m, total radius %.3f m, rebuild %.1f ms",
      costs_.size(), clearance_, inflation_width_, outer_radius_, ms);
    make_cloud();
  }
  void make_cloud()
  {
    size_t count = 0;
    for (const auto & entry : costs_) {
      for (auto cost : entry.second.occupancy) {if (cost > 0 && cost < Block::UNKNOWN) {++count;}}
    }
    sensor_msgs::PointCloud2Modifier modifier(cloud_);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(count);
    sensor_msgs::PointCloud2Iterator<float> x(cloud_, "x"), y(cloud_, "y"), z(cloud_, "z"), intensity(cloud_, "intensity");
    for (const auto & entry : costs_) {
      for (size_t i = 0; i < vg::block_volume; ++i) {
        auto cost = entry.second.occupancy[i];
        if (cost == 0 || cost == Block::UNKNOWN) {continue;}
        auto point = geometry_->voxel_center(voxel(entry.second, i));
        *x = point.x; *y = point.y; *z = point.z; *intensity = cost;
        ++x; ++y; ++z; ++intensity;
      }
    }
  }
  void publish()
  {
    Costmap output;
    output.header.frame_id = frame_; output.header.stamp = now();
    output.resolution = geometry_->resolution();
    auto origin = geometry_->origin();
    output.origin.x = origin.x; output.origin.y = origin.y; output.origin.z = origin.z;
    output.map_id = map_id_; output.sequence = cost_sequence_;
    for (const auto & entry : costs_) {output.blocks.push_back(entry.second);}
    if (!synchronized_) {sensor_msgs::PointCloud2Modifier(cloud_).resize(0);}
    cloud_.header = output.header;
    publisher_->publish(output); cloud_publisher_->publish(cloud_);
  }
  double clearance_, inflation_width_, outer_radius_, scaling_;
  int64_t stencil_limit_;
  bool synchronized_ = false;
  uint64_t map_id_ = 0, map_sequence_ = 0, cost_sequence_ = 0;
  std::string frame_;
  std::unique_ptr<vg::Geometry> geometry_;
  std::vector<Offset> stencil_;
  Blocks occupancy_, costs_;
  sensor_msgs::msg::PointCloud2 cloud_;
  rclcpp::Publisher<Costmap>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Subscription<Map>::SharedPtr map_subscription_;
  rclcpp::Subscription<Update>::SharedPtr update_subscription_;
  rclcpp::Service<uav_navigation_msgs::srv::CheckCollision>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<VoxelCostmap>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("voxel_costmap"), "%s", error.what()); result = 1;
  }
  rclcpp::shutdown(); return result;
}
