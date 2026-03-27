cmake_minimum_required(VERSION 3.5)
project(track_robot_control)

# Default to C++14 (Foxy 常用)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(track_robot_interfaces REQUIRED)

# 先不加可执行文件（我们暂时不写 node）
# 后面加 node 时，大概是这样：
# add_executable(track_robot_control_node src/xxx.cpp)
# ament_target_dependencies(track_robot_control_node rclcpp std_msgs geometry_msgs track_robot_interfaces)
# install(TARGETS track_robot_control_node DESTINATION lib/${PROJECT_NAME})

ament_package()
