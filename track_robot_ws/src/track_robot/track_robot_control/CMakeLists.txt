cmake_minimum_required(VERSION 3.5)
project(track_robot_control)

# Default to C++14 (Foxy 常用)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(track_robot_interfaces REQUIRED)
find_package(bunker_msgs REQUIRED)

add_executable(target_follow_controller_node src/target_follow_controller.cpp)
ament_target_dependencies(
  target_follow_controller_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  track_robot_interfaces
  bunker_msgs
)

install(TARGETS
  target_follow_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
