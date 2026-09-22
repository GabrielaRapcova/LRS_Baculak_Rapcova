#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <string>

class Mapper : public rclcpp::Node
{
  public:
  Mapper() : Node("mapper_node")
  {
    this->declare_parameter("map_path", "/home/user/LRS-URK/worlds/fei_lrs_gazebo.world");
    std::string map_path = this->get_parameter("map_path").as_string();
    RCLCPP_INFO(this->get_logger(), "Loading map from path %s", map_path.c_str());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Mapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
