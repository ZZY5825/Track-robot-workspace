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

# 先留空：后面你加 msg/srv/action 文件后，在这里登记
# 示例：
# set(msg_files
#   "msg/RobotState.msg"
# )
# set(srv_files
#   "srv/SetMode.srv"
# )
# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   ${srv_files}
#   DEPENDENCIES builtin_interfaces
# )

ament_package()
