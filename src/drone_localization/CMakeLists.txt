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

# --- §1 sensor acquisition / characterisation node -------------------------
add_executable(sensor_check_node src/sensor_check_node.cpp)
ament_target_dependencies(sensor_check_node
  rclcpp geometry_msgs nav_msgs sensor_msgs)

install(TARGETS sensor_check_node DESTINATION lib/${PROJECT_NAME})

# --- Python tooling (bag analysis, characterisation flight) ----------------
ament_python_install_package(drone_localization_tools)
install(PROGRAMS
  scripts/characterize_sensors.py
  scripts/characterisation_flight.py
  scripts/record_characterisation.sh
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
