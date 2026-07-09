cmake_minimum_required(VERSION 3.5)
project(track_robot_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14 (Foxy 默认常用)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/GestureState.msg"
  "msg/HumanDetection2D.msg"
  "msg/HumanDetection2DArray.msg"
  "msg/LidarCluster.msg"
  "msg/LidarClusterArray.msg"
  "msg/LidarTracklet.msg"
  "msg/LidarTrackletArray.msg"
  "msg/SelectedLidarTracklet.msg"
  "msg/TargetState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_package()
