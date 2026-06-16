cmake_minimum_required(VERSION 3.5)
project(lidar_mos_filter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(range_image_mos_filter src/range_image_mos_filter.cpp)
ament_target_dependencies(range_image_mos_filter
  rclcpp
  sensor_msgs
  nav_msgs
)

install(TARGETS range_image_mos_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
