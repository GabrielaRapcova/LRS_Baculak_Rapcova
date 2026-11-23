#include "mission_executor/outdoor_mission_node.hpp"

#include <exception>
#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<mission_executor::OutdoorMissionNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("outdoor_mission_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
