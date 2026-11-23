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

# --- Estimator core: plain C++/Eigen, no ROS (shared by node, replay, tests) -
add_library(drone_localization_core
  src/ekf.cpp
  src/localizer.cpp)
target_include_directories(drone_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(drone_localization_core PUBLIC Eigen3::Eigen)

# Offline replay of CSV sensor events through the same core.
add_executable(ekf_replay src/ekf_replay.cpp)
target_link_libraries(ekf_replay drone_localization_core)

# --- §1 sensor acquisition / characterisation node -------------------------
add_executable(sensor_check_node src/sensor_check_node.cpp)
ament_target_dependencies(sensor_check_node
  rclcpp geometry_msgs nav_msgs sensor_msgs)

install(TARGETS sensor_check_node ekf_replay DESTINATION lib/${PROJECT_NAME})
install(TARGETS drone_localization_core
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# --- Python tooling (bag analysis, characterisation flight) ----------------
ament_python_install_package(drone_localization_tools)
install(PROGRAMS
  scripts/characterize_sensors.py
  scripts/evaluate_replay.py
  scripts/characterisation_flight.py
  scripts/record_characterisation.sh
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ekf test/test_ekf.cpp)
  target_link_libraries(test_ekf drone_localization_core)
endif()

ament_package()
