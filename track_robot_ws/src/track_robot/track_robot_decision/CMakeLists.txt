cmake_minimum_required(VERSION 3.5)
project(track_robot_decision)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(track_robot_interfaces REQUIRED)

add_executable(perception_health_monitor_node src/perception_health_monitor_node.cpp)
ament_target_dependencies(perception_health_monitor_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros track_robot_interfaces)

add_executable(follow_behavior_tree_node src/follow_behavior_tree_node.cpp)
ament_target_dependencies(follow_behavior_tree_node
  ament_index_cpp rclcpp std_msgs std_srvs visualization_msgs track_robot_interfaces behaviortree_cpp_v3)

install(TARGETS perception_health_monitor_node follow_behavior_tree_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_follow_decision_launch.py TIMEOUT 30)
endif()

ament_package()
