cmake_minimum_required(VERSION 3.5)
project(track_robot_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(track_robot_interfaces REQUIRED)
find_package(bunker_msgs REQUIRED)

add_executable(local_obstacle_map_node src/local_obstacle_map_node.cpp)
ament_target_dependencies(
  local_obstacle_map_node
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs std_msgs tf2_ros)

add_executable(local_trajectory_planner_node src/local_trajectory_planner_node.cpp)
ament_target_dependencies(
  local_trajectory_planner_node
  rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs track_robot_interfaces)

add_executable(motion_safety_supervisor_node src/motion_safety_supervisor_node.cpp)
ament_target_dependencies(
  motion_safety_supervisor_node
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs std_msgs std_srvs
  track_robot_interfaces bunker_msgs)

install(TARGETS
  local_obstacle_map_node
  local_trajectory_planner_node
  motion_safety_supervisor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_nav2_safety_contract
    test/test_nav2_safety_contract.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
