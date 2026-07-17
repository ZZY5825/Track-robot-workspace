cmake_minimum_required(VERSION 3.5)
project(track_robot_lidar_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(track_robot_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(lidar_tracklet_manager_node src/lidar_tracklet_manager_node.cpp)
ament_target_dependencies(
  lidar_tracklet_manager_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  track_robot_interfaces
)
target_link_libraries(lidar_tracklet_manager_node Eigen3::Eigen)

add_executable(selected_human_target_tracker_node src/selected_human_target_tracker_node.cpp)
ament_target_dependencies(
  selected_human_target_tracker_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  track_robot_interfaces
)
target_link_libraries(selected_human_target_tracker_node Eigen3::Eigen)

if(BUILD_TESTING)
  # Foxy ships a gtest_vendor CMake project that predates CMake 4 policy rules.
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_source_epoch test/test_source_epoch.cpp)
  target_include_directories(test_source_epoch PRIVATE include)
  add_test(
    NAME test_semantic_launch_contract
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_semantic_launch_contract.py
  )
endif()

install(TARGETS
  lidar_tracklet_manager_node
  selected_human_target_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
