cmake_minimum_required(VERSION 3.8)
project(uav_navigation_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VoxelBlock.msg"
  "msg/VoxelMap.msg"
  "msg/VoxelMapUpdate.msg"
  "msg/VoxelCostmap.msg"
  "msg/VoxelCostmapUpdate.msg"
  "srv/CheckCollision.srv"
  "action/ComputePath3D.action"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
