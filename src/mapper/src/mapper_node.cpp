#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nav2_msgs/msg/voxel_grid.hpp>
#include "uav_navigation_msgs/srv/check_collision.hpp"

class Mapper : public rclcpp::Node
{
private:

  // --------------------------------------------------------------------------
  // Map representation
  // --------------------------------------------------------------------------

  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;

  pcl::PointCloud<pcl::PointXYZ>::Ptr _downsampled_cloud;

  nav2_msgs::msg::VoxelGrid _voxel_grid;


  // --------------------------------------------------------------------------
  // Parameters
  // --------------------------------------------------------------------------

  std::string _map_path;

  double _voxel_resolution;
  double _inflation_radius;


  // --------------------------------------------------------------------------
  // ROS interfaces
  // --------------------------------------------------------------------------

  rclcpp::Publisher<nav2_msgs::msg::VoxelGrid>::SharedPtr
  _voxel_grid_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    _map_cloud_pub_;

  rclcpp::Service<uav_navigation_msgs::srv::CheckCollision>::SharedPtr
    _collision_service_;


  // --------------------------------------------------------------------------
  // Map creation
  // --------------------------------------------------------------------------

  bool load_map();

  void build_voxel_grid();

  void inflate_voxel_grid();

  void publish_map();

  void publish_downsampled_cloud();


  // --------------------------------------------------------------------------
  // Voxel utilities
  // --------------------------------------------------------------------------

  std::size_t voxel_index(
    int x,
    int y,
    int z) const;

  bool world_to_voxel(
    const geometry_msgs::msg::Point & point,
    int & x,
    int & y,
    int & z) const;

  bool is_inside(
    int x,
    int y,
    int z) const;

  uint8_t voxel_cost(
    int x,
    int y,
    int z) const;


  // --------------------------------------------------------------------------
  // Service
  // --------------------------------------------------------------------------

  void check_collision(
    const std::shared_ptr<
      uav_navigation_msgs::srv::CheckCollision::Request> request,
    std::shared_ptr<
      uav_navigation_msgs::srv::CheckCollision::Response> response);


public:

  Mapper();
};

Mapper::Mapper()
: Node("mapper_node"),
  _cloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()),
  _downsampled_cloud(
    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>())
{
  // --------------------------------------------------------------------------
  // Parameters
  // --------------------------------------------------------------------------

  this->declare_parameter(
    "map_path",
    "/home/user/LRS-URK/maps/map.pcd");

  this->declare_parameter(
    "voxel_resolution",
    0.20);

  this->declare_parameter(
    "inflation_radius",
    0.60);


  _map_path =
    this->get_parameter("map_path").as_string();

  _voxel_resolution =
    this->get_parameter("voxel_resolution").as_double();

  _inflation_radius =
    this->get_parameter("inflation_radius").as_double();


  // --------------------------------------------------------------------------
  // QoS
  //
  // Reliable:
  //     We don't want to silently lose the map.
  //
  // Transient local:
  //     The publisher keeps the last map for late subscribers.
  //
  // Depth 1:
  //     We only care about the latest map.
  // --------------------------------------------------------------------------

  auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  map_qos.reliable();
  map_qos.transient_local();

  _voxel_grid_pub_ =
    this->create_publisher<nav2_msgs::msg::VoxelGrid>(
      "/voxel_grid",
      map_qos);


  _map_cloud_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/map_cloud",
      map_qos);


  // --------------------------------------------------------------------------
  // Collision service
  // --------------------------------------------------------------------------

  _collision_service_ =
    this->create_service<uav_navigation_msgs::srv::CheckCollision>(
      "/check_collision",
      std::bind(
        &Mapper::check_collision,
        this,
        std::placeholders::_1,
        std::placeholders::_2));


  // --------------------------------------------------------------------------
  // Load and process map
  // --------------------------------------------------------------------------

  RCLCPP_INFO(
    this->get_logger(),
    "Loading map from: %s",
    _map_path.c_str());

  if (!load_map())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to load map.");

    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded %zu points.",
    _cloud->size());


  build_voxel_grid();

  inflate_voxel_grid();

  publish_map();

  publish_downsampled_cloud();


  RCLCPP_INFO(
    this->get_logger(),
    "Map initialization complete.");
}

bool Mapper::load_map()
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(_map_path, *_cloud) == -1)
  {
    PCL_ERROR("Couldn't read file %s\n", _map_path.c_str());
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Loaded %zu data points from the map .pcd",
    _cloud->size());

  return true;
}

void Mapper::build_voxel_grid()
{
  pcl::VoxelGrid<pcl::PointXYZ> filter;

  filter.setInputCloud(_cloud);

  filter.setLeafSize(
    static_cast<float>(_voxel_resolution),
    static_cast<float>(_voxel_resolution),
    static_cast<float>(_voxel_resolution));

  filter.filter(*_downsampled_cloud);


  if (_downsampled_cloud->empty())
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Downsampled point cloud is empty.");

    return;
  }


  // --------------------------------------------------------------------------
  // Find map bounds
  // --------------------------------------------------------------------------

  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();

  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  float max_z = std::numeric_limits<float>::lowest();


  for (const auto & point : *_downsampled_cloud)
  {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    min_z = std::min(min_z, point.z);

    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    max_z = std::max(max_z, point.z);
  }


  // --------------------------------------------------------------------------
  // Align origin to voxel boundaries
  // --------------------------------------------------------------------------

  min_x =
    std::floor(min_x / _voxel_resolution)
    * _voxel_resolution;

  min_y =
    std::floor(min_y / _voxel_resolution)
    * _voxel_resolution;

  min_z =
    std::floor(min_z / _voxel_resolution)
    * _voxel_resolution;


  max_x =
    std::ceil(max_x / _voxel_resolution)
    * _voxel_resolution;

  max_y =
    std::ceil(max_y / _voxel_resolution)
    * _voxel_resolution;

  max_z =
    std::ceil(max_z / _voxel_resolution)
    * _voxel_resolution;


  // --------------------------------------------------------------------------
  // Number of voxels
  // --------------------------------------------------------------------------

  const uint32_t size_x =
    static_cast<uint32_t>(
      std::ceil(
        (max_x - min_x) / _voxel_resolution));

  const uint32_t size_y =
    static_cast<uint32_t>(
      std::ceil(
        (max_y - min_y) / _voxel_resolution));

  const uint32_t size_z =
    static_cast<uint32_t>(
      std::ceil(
        (max_z - min_z) / _voxel_resolution));


  // --------------------------------------------------------------------------
  // Construct message
  // --------------------------------------------------------------------------

  _voxel_grid.header.stamp = this->now();
  _voxel_grid.header.frame_id = "map";

  _voxel_grid.origin.x = min_x;
  _voxel_grid.origin.y = min_y;
  _voxel_grid.origin.z = min_z;

  _voxel_grid.resolutions.x = _voxel_resolution;
  _voxel_grid.resolutions.y = _voxel_resolution;
  _voxel_grid.resolutions.z = _voxel_resolution;

  _voxel_grid.size_x = size_x;
  _voxel_grid.size_y = size_y;
  _voxel_grid.size_z = size_z;

  _voxel_grid.data.assign(
    static_cast<std::size_t>(size_x) *
    size_y *
    size_z,
    255);

  // --------------------------------------------------------------------------
  // Mark occupied voxels
  // --------------------------------------------------------------------------

  for (const auto & point : *_downsampled_cloud)
  {
    const int x =
      static_cast<int>(
        std::floor(
          (point.x - min_x)
          / _voxel_resolution));

    const int y =
      static_cast<int>(
        std::floor(
          (point.y - min_y)
          / _voxel_resolution));

    const int z =
      static_cast<int>(
        std::floor(
          (point.z - min_z)
          / _voxel_resolution));


    if (!is_inside(x, y, z))
      continue;


    const auto index =
      voxel_index(x, y, z);


    _voxel_grid.data[index] = 254;
  }


  RCLCPP_INFO(
    this->get_logger(),
    "Voxel grid: %u x %u x %u = %zu voxels",
    size_x,
    size_y,
    size_z,
    total_voxels);
}

std::size_t Mapper::voxel_index(
  int x,
  int y,
  int z) const
{
  return
    static_cast<std::size_t>(x)
    + static_cast<std::size_t>(_voxel_grid.size_x)
      * (
        static_cast<std::size_t>(y)
        + static_cast<std::size_t>(_voxel_grid.size_y)
          * static_cast<std::size_t>(z)
      );
}

bool Mapper::is_inside(
  int x,
  int y,
  int z) const
{
  return
    x >= 0 &&
    y >= 0 &&
    z >= 0 &&
    x < static_cast<int>(_voxel_grid.size_x) &&
    y < static_cast<int>(_voxel_grid.size_y) &&
    z < static_cast<int>(_voxel_grid.size_z);
}

// TODO: Do the inflation like this: construct an "inflation element" - this would mean the the cost values of neighbour around the singular obstacle element. Then we loop through every obstacle element, and inflate its neighbor using the inflation element. During this phase we can only overwrite the cost of an element with a larger value than already set. The inflation element depends on costmap settings - drone radius, inflation radius, ... 
void Mapper::inflate_voxel_grid()
{
  const int radius_voxels =
    static_cast<int>(
      std::ceil(
        _inflation_radius /
        _voxel_resolution));


  // Keep the original occupied map.
  const auto original =
    _voxel_grid.data;


  for (int z = 0;
       z < static_cast<int>(_voxel_grid.size_z);
       ++z)
  {
    for (int y = 0;
         y < static_cast<int>(_voxel_grid.size_y);
         ++y)
    {
      for (int x = 0;
           x < static_cast<int>(_voxel_grid.size_x);
           ++x)
      {
        const auto current_index =
          voxel_index(x, y, z);


        // Don't modify occupied voxels.
        if (original[current_index] == 254)
          continue;


        double nearest_distance =
          std::numeric_limits<double>::max();


        bool found_obstacle = false;


        for (int dz = -radius_voxels;
             dz <= radius_voxels;
             ++dz)
        {
          for (int dy = -radius_voxels;
               dy <= radius_voxels;
               ++dy)
          {
            for (int dx = -radius_voxels;
                 dx <= radius_voxels;
                 ++dx)
            {
              const double distance =
                std::sqrt(
                  static_cast<double>(dx * dx)
                  + static_cast<double>(dy * dy)
                  + static_cast<double>(dz * dz));


              if (distance > radius_voxels)
                continue;


              const int nx = x + dx;
              const int ny = y + dy;
              const int nz = z + dz;


              if (!is_inside(nx, ny, nz))
                continue;


              if (original[voxel_index(nx, ny, nz)] == 254)
              {
                found_obstacle = true;

                nearest_distance =
                  std::min(
                    nearest_distance,
                    distance);
              }
            }
          }
        }


        if (!found_obstacle)
          continue;


        const double metric_distance =
          nearest_distance * _voxel_resolution;


        if (metric_distance <= _voxel_resolution)
        {
          _voxel_grid.data[current_index] = 253;
        }
        else
        {
          const double normalized =
            1.0 -
            metric_distance / _inflation_radius;


          const uint8_t cost =
            static_cast<uint8_t>(
              std::clamp(
                normalized * 252.0,
                1.0,
                252.0));


          _voxel_grid.data[current_index] = cost;
        }
      }
    }
  }
}

bool Mapper::world_to_voxel(
  const geometry_msgs::msg::Point & point,
  int & x,
  int & y,
  int & z) const
{
  const double ox =
    _voxel_grid.origin.position.x;

  const double oy =
    _voxel_grid.origin.position.y;

  const double oz =
    _voxel_grid.origin.position.z;


  x = static_cast<int>(
    std::floor(
      (point.x - ox)
      / _voxel_grid.resolution));

  y = static_cast<int>(
    std::floor(
      (point.y - oy)
      / _voxel_grid.resolution));

  z = static_cast<int>(
    std::floor(
      (point.z - oz)
      / _voxel_grid.resolution));


  return is_inside(x, y, z);
}

void Mapper::check_collision(
  const std::shared_ptr<
    uav_navigation_msgs::srv::CheckCollision::Request> request,
  std::shared_ptr<
    uav_navigation_msgs::srv::CheckCollision::Response> response)
{
  int x;
  int y;
  int z;


  if (!world_to_voxel(
        request->point,
        x,
        y,
        z))
  {
    // Outside the known map is unknown.
    response->cost = 255;
    response->unknown = true;

    // Conservative navigation policy:
    // unknown space is considered unsafe.
    response->collision = true;

    return;
  }


  const uint8_t cost =
    voxel_cost(x, y, z);


  response->cost = cost;

  response->unknown =
    cost == 255;


  response->collision =
    cost >= 253;
}

uint8_t Mapper::voxel_cost(int x, int y, int z) const
{
  if (!is_inside(x, y, z))
    return 255;

  return static_cast<uint8_t>(
    _voxel_grid.data[voxel_index(x, y, z)] & 0xFFu);
}

void Mapper::set_voxel_cost(
  int x, int y, int z, uint8_t cost)
{
  if (!is_inside(x, y, z))
    return;

  _voxel_grid.data[voxel_index(x, y, z)] =
    static_cast<uint32_t>(cost);
}

void Mapper::publish_map()
{
  _voxel_grid.header.stamp = this->now();

  _voxel_grid_pub_->publish(_voxel_grid);

  RCLCPP_INFO(
    this->get_logger(),
    "Published voxel grid.");
}

void Mapper::publish_downsampled_cloud()
{
  sensor_msgs::msg::PointCloud2 msg;

  pcl::toROSMsg(
    *_downsampled_cloud,
    msg);


  msg.header.stamp = this->now();
  msg.header.frame_id = "map";


  _map_cloud_pub_->publish(msg);


  RCLCPP_INFO(
    this->get_logger(),
    "Published downsampled map cloud with %zu points.",
    _downsampled_cloud->size());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mapper>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}