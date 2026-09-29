#include <cstdio>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class Mapper : public rclcpp::Node
{
private:
  std::string _map_path;

  pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;

  bool load_map();

public:
  Mapper()
  : Node("mapper_node"),
    _cloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>())
  {
    this->declare_parameter(
      "map_path",
      "/home/user/LRS-URK/worlds/map.pcd");

    _map_path = this->get_parameter("map_path").as_string();

    RCLCPP_INFO(
      this->get_logger(),
      "Loading map from path %s",
      _map_path.c_str());

    if (load_map()) {
      RCLCPP_INFO(this->get_logger(), "Map loaded successfully!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map!");
    }
  }
};

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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Mapper>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}