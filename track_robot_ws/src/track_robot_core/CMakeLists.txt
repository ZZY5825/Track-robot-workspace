cmake_minimum_required(VERSION 3.5)
project(track_robot_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Foxy 默认 C++14 就够用
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cmd_vel_gate src/cmd_vel_gate.cpp)
ament_target_dependencies(cmd_vel_gate rclcpp geometry_msgs)

add_executable(lidar_sub src/lidar_sub.cpp)
ament_target_dependencies(lidar_sub rclcpp sensor_msgs)

install(TARGETS
  cmd_vel_gate
  lidar_sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
