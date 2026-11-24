cmake_minimum_required(VERSION 3.8)
project(drone_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)

# Estimator core (EKF + sensor handling), shared by the node, the replay tool,
# the unit tests and localization_flight's closed-loop test.
add_library(drone_localization_core STATIC src/ekf.cpp src/localizer.cpp)
set_target_properties(drone_localization_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(drone_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(drone_localization_core PUBLIC Eigen3::Eigen)
ament_target_dependencies(drone_localization_core PUBLIC rclcpp angles)

add_executable(estimator_node src/estimator_node.cpp)
target_link_libraries(estimator_node drone_localization_core)
ament_target_dependencies(estimator_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs)

add_executable(sensor_check_node src/sensor_check_node.cpp)
ament_target_dependencies(sensor_check_node
  rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_geometry_msgs)

add_executable(ekf_replay src/ekf_replay.cpp)
target_link_libraries(ekf_replay drone_localization_core)

install(TARGETS estimator_node sensor_check_node ekf_replay
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS drone_localization_core EXPORT export_drone_localization
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_export_targets(export_drone_localization HAS_LIBRARY_TARGET)
ament_export_dependencies(eigen3_cmake_module Eigen3 rclcpp angles)

# Python tooling: bag analysis, offline simulation/replay, evaluation.
ament_python_install_package(drone_localization_tools)
install(PROGRAMS
  scripts/characterize_sensors.py
  scripts/characterisation_flight.py
  scripts/record_characterisation.sh
  scripts/evaluate_replay.py
  scripts/estimate_evaluator.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ekf test/test_ekf.cpp)
  target_link_libraries(test_ekf drone_localization_core)
endif()

ament_package()
