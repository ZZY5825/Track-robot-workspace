cmake_minimum_required(VERSION 3.5)
project(track_robot_drivers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 后面我们加 node 时就启用：
# add_executable(lidar_bridge_node src/lidar_bridge_node.cpp)
# ament_target_dependencies(lidar_bridge_node rclcpp sensor_msgs)
# install(TARGETS lidar_bridge_node DESTINATION lib/${PROJECT_NAME})

ament_package()
