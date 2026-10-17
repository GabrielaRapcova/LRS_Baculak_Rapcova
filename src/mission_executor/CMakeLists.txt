cmake_minimum_required(VERSION 3.8)
project(mission_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(mission_executor_node
  src/mission_executor_node.cpp
  src/mission_parser.cpp
  src/position_controller.cpp
)

target_include_directories(mission_executor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mission_executor_node
  rclcpp
  geometry_msgs
  mavros_msgs
)

install(TARGETS
  mission_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
  add_executable(test_position_controller
    test/test_position_controller.cpp
    src/position_controller.cpp
  )

  target_include_directories(test_position_controller PRIVATE
    include
  )

  ament_target_dependencies(test_position_controller
    geometry_msgs
  )

add_test(
    NAME test_position_controller
    COMMAND $<TARGET_FILE:test_position_controller>
  )

endif()

ament_package()
