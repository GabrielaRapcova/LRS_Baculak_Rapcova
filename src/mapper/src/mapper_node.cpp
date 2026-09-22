#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class Mapper : public rclcpp::Node
{

private:
  std::string _map_path;

  bool load_map();
public:
  Mapper() : Node("mapper_node")
  {
    this->declare_parameter("map_path", "/home/user/LRS-URK/worlds/fei_lrs_gazebo.world");
    std::string map_path = this->get_parameter("map_path").as_string();
    RCLCPP_INFO(this->get_logger(), "Loading map from path %s", map_path.c_str());
    _map_path = map_path;
    
    bool success = load_map();
    if(success)
      RCLCPP_INFO(this->get_logger(), "Map loaded successfully!");
    else
      RCLCPP_INFO(this->get_logger(), "Failed to load map!");
  }
};

bool Mapper::load_map()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (_map_path, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return false;
  }
  RCLCPP_INFO(
    this->get_logger(),
    "Loaded %u data points from the map .pcd",
    cloud->width * cloud->height
  );
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
