cmake_minimum_required(VERSION 3.5)
project(track_robot_bringup)

find_package(ament_cmake REQUIRED)

# 后面你会放这些目录：
#   launch/
#   config/
#   rviz/
# 先把 install 规则写好（即使目录暂时为空也没问题）
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
