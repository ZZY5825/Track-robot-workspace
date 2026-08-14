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
find_package(bunker_msgs REQUIRED)

add_library(human_following_session_policy
  src/human_following_session_policy.cpp)
target_include_directories(human_following_session_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(perception_health_monitor_node src/perception_health_monitor_node.cpp)
ament_target_dependencies(perception_health_monitor_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros track_robot_interfaces)

add_executable(follow_behavior_tree_node src/follow_behavior_tree_node.cpp)
ament_target_dependencies(follow_behavior_tree_node
  ament_index_cpp rclcpp std_msgs std_srvs visualization_msgs track_robot_interfaces behaviortree_cpp_v3)

add_executable(human_following_supervisor_node src/human_following_supervisor_node.cpp)
ament_target_dependencies(human_following_supervisor_node
  bunker_msgs rclcpp std_msgs std_srvs visualization_msgs track_robot_interfaces)
target_link_libraries(human_following_supervisor_node human_following_session_policy)

install(TARGETS
  perception_health_monitor_node
  follow_behavior_tree_node
  human_following_supervisor_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS human_following_session_policy
  EXPORT export_human_following_session_policy
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config docs DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_human_following_session_policy HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(
    test_human_following_session_policy
    test/test_human_following_session_policy.cpp)
  target_link_libraries(
    test_human_following_session_policy
    human_following_session_policy)
  add_launch_test(test/test_follow_decision_launch.py TIMEOUT 30)
  add_launch_test(test/test_human_following_supervisor_launch.py TIMEOUT 60)
endif()

ament_package()
