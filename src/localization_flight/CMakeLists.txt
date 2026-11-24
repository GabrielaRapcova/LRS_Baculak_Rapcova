cmake_minimum_required(VERSION 3.8)
project(localization_flight)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Route following on the estimate (no ROS) — unit- and closed-loop tested.
add_library(route_controller STATIC src/route_controller.cpp)
target_include_directories(route_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(route_controller PUBLIC Eigen3::Eigen)
ament_target_dependencies(route_controller PUBLIC angles)

add_executable(estimated_mission_node src/estimated_mission_node.cpp)
target_link_libraries(estimated_mission_node route_controller)
ament_target_dependencies(estimated_mission_node
  rclcpp nav_msgs mavros_msgs tf2 tf2_geometry_msgs)

install(TARGETS estimated_mission_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(drone_localization REQUIRED)
  # Closed loop: the real Localizer + this controller on a simulated vehicle.
  ament_add_gtest(test_closed_loop test/test_closed_loop.cpp TIMEOUT 300)
  target_link_libraries(test_closed_loop route_controller
    drone_localization::drone_localization_core)
endif()

ament_package()
