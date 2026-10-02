#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <uav_navigation_msgs/msg/voxel_map.hpp>
#include <uav_navigation_msgs/srv/check_collision.hpp>
#include <voxel_grid_core/coordinates.hpp>

namespace vg = voxel_grid_core;
using Block = uav_navigation_msgs::msg::VoxelBlock;
using Key = std::tuple<int32_t, int32_t, int32_t>;

class StaticMapper : public rclcpp::Node
{
public:
  StaticMapper() : Node("static_mapper")
  {
    const auto path = declare_parameter<std::string>("map_file", "");
    const auto frame = declare_parameter<std::string>("frame_id", "map");
    const double resolution = declare_parameter<double>("resolution", 0.2);
    const auto origin = declare_parameter<std::vector<double>>("origin", {0., 0., 0.});
    const auto minimum = declare_parameter<std::vector<double>>("known_free_min", std::vector<double>{});
    const auto maximum = declare_parameter<std::vector<double>>("known_free_max", std::vector<double>{});
    const double frequency = declare_parameter<double>("publish_frequency", 0.2);
    const auto limit = declare_parameter<int64_t>("max_blocks", 100000);
    if (path.empty() || frame.empty() || origin.size() != 3 ||
      !std::isfinite(frequency) || frequency <= 0 || limit <= 0 ||
      !std::isfinite(static_cast<float>(resolution)) || static_cast<float>(resolution) <= 0)
    {
      throw std::invalid_argument("Invalid map_file, frame, origin, resolution, frequency or block limit");
    }
    // Use the exact float32 geometry sent to consumers.
    geometry_ = std::make_unique<vg::Geometry>(static_cast<float>(resolution),
      vg::Point{origin[0], origin[1], origin[2]});
    map_.resolution = geometry_->resolution();
    map_.origin.x = origin[0]; map_.origin.y = origin[1]; map_.origin.z = origin[2];
    map_.header.frame_id = frame;
    map_.map_id = static_cast<uint64_t>(
      std::chrono::system_clock::now().time_since_epoch().count());
    map_.sequence = 0;
    max_blocks_ = static_cast<size_t>(limit);
    if (!minimum.empty() || !maximum.empty()) {
      if (minimum.size() != 3 || maximum.size() != 3) {
        throw std::invalid_argument("Both known-free bounds require three coordinates");
      }
      for (size_t i = 0; i < 3; ++i) {
        if (!std::isfinite(minimum[i]) || !std::isfinite(maximum[i]) || minimum[i] >= maximum[i]) {
          throw std::invalid_argument("Known-free bounds must be finite and increasing");
        }
      }
      const auto low = geometry_->world_to_voxel({minimum[0], minimum[1], minimum[2]});
      const auto high = geometry_->world_to_voxel({maximum[0], maximum[1], maximum[2]});
      const auto first_block = vg::block_index(low);
      const auto last_block = vg::block_index(high);
      const long double requested_blocks =
        (static_cast<long double>(last_block.x) - first_block.x + 1) *
        (static_cast<long double>(last_block.y) - first_block.y + 1) *
        (static_cast<long double>(last_block.z) - first_block.z + 1);
      if (requested_blocks > max_blocks_) {
        throw std::invalid_argument("Known-free domain exceeds max_blocks");
      }
      // Only voxel centers in the half-open configured domain become known free.
      for (int64_t z = low.z; z <= high.z; ++z) {
        for (int64_t y = low.y; y <= high.y; ++y) {
          for (int64_t x = low.x; x <= high.x; ++x) {
            const vg::Index index{static_cast<int32_t>(x), static_cast<int32_t>(y), static_cast<int32_t>(z)};
            const auto center = geometry_->voxel_center(index);
            if (center.x >= minimum[0] && center.x < maximum[0] &&
              center.y >= minimum[1] && center.y < maximum[1] &&
              center.z >= minimum[2] && center.z < maximum[2]) {
              set(index, Block::FREE);
            }
          }
        }
      }
    }
    pcl::PointCloud<pcl::PointXYZ> input;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(path, input) < 0) {
      throw std::runtime_error("Could not load PCD: " + path);
    }
    vg::Point lower{INFINITY, INFINITY, INFINITY}, upper{-INFINITY, -INFINITY, -INFINITY};
    size_t valid = 0;
    for (const auto & point : input) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {continue;}
      set(geometry_->world_to_voxel({point.x, point.y, point.z}), Block::LETHAL);
      lower.x = std::min(lower.x, double(point.x)); upper.x = std::max(upper.x, double(point.x));
      lower.y = std::min(lower.y, double(point.y)); upper.y = std::max(upper.y, double(point.y));
      lower.z = std::min(lower.z, double(point.z)); upper.z = std::max(upper.z, double(point.z));
      ++valid;
    }
    if (valid == 0) {throw std::runtime_error("PCD contains no finite points");}
    pcl::PointCloud<pcl::PointXYZ> occupied;
    size_t free_count = 0;
    for (const auto & entry : blocks_) {
      const auto & block = entry.second;
      for (size_t i = 0; i < vg::block_volume; ++i) {
        if (block.occupancy[i] == Block::FREE) {++free_count;}
        if (block.occupancy[i] != Block::LETHAL) {continue;}
        auto center = geometry_->voxel_center({block.x * 8 + static_cast<int>(i % 8),
          block.y * 8 + static_cast<int>((i / 8) % 8), block.z * 8 + static_cast<int>(i / 64)});
        occupied.push_back(pcl::PointXYZ(center.x, center.y, center.z));
      }
      map_.blocks.push_back(block);
    }
    pcl::toROSMsg(occupied, cloud_);
    cloud_.header.frame_id = frame;
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    publisher_ = create_publisher<uav_navigation_msgs::msg::VoxelMap>("map", qos);
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", qos);
    service_ = create_service<uav_navigation_msgs::srv::CheckCollision>("query_occupancy",
      [this](const std::shared_ptr<uav_navigation_msgs::srv::CheckCollision::Request> request,
      std::shared_ptr<uav_navigation_msgs::srv::CheckCollision::Response> response) {
        response->cost = Block::UNKNOWN;
        try {
          const auto index = geometry_->world_to_voxel({request->point.x, request->point.y, request->point.z});
          const auto block = vg::block_index(index);
          const auto found = blocks_.find({block.x, block.y, block.z});
          if (found != blocks_.end()) {response->cost = found->second.occupancy[vg::local_index(index)];}
        } catch (const std::exception &) {}
        response->collision = response->cost != Block::FREE;
      });
    RCLCPP_INFO(get_logger(), "PCD: %zu finite / %zu points; bounds [%g,%g,%g] to [%g,%g,%g]; %zu blocks, %zu occupied, %zu free voxels",
      valid, input.size(), lower.x, lower.y, lower.z, upper.x, upper.y, upper.z,
      blocks_.size(), occupied.size(), free_count);
    publish();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / frequency), [this]() {publish();});
  }

private:
  void set(vg::Index index, uint8_t value)
  {
    const auto coordinate = vg::block_index(index);
    const Key key{coordinate.x, coordinate.y, coordinate.z};
    auto found = blocks_.find(key);
    if (found == blocks_.end()) {
      if (blocks_.size() >= max_blocks_) {throw std::runtime_error("Map exceeds max_blocks");}
      Block block;
      block.x = coordinate.x; block.y = coordinate.y; block.z = coordinate.z;
      block.occupancy.fill(Block::UNKNOWN);
      found = blocks_.emplace(key, block).first;
    }
    found->second.occupancy[vg::local_index(index)] = value;
  }
  void publish()
  {
    map_.header.stamp = now(); cloud_.header.stamp = map_.header.stamp;
    publisher_->publish(map_); cloud_publisher_->publish(cloud_);
  }
  std::unique_ptr<vg::Geometry> geometry_;
  std::map<Key, Block> blocks_;
  size_t max_blocks_;
  uav_navigation_msgs::msg::VoxelMap map_;
  sensor_msgs::msg::PointCloud2 cloud_;
  rclcpp::Publisher<uav_navigation_msgs::msg::VoxelMap>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Service<uav_navigation_msgs::srv::CheckCollision>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<StaticMapper>());}
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("static_mapper"), "%s", error.what()); result = 1;
  }
  rclcpp::shutdown();
  return result;
}
