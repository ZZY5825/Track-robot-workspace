cmake_minimum_required(VERSION 3.5)
project(track_robot_semantic_memory)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(track_robot_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(semantic_memory_primitives
  src/appearance_memory.cpp
  src/association_calibration.cpp
  src/association_confirmation.cpp
  src/association_terms.cpp
  src/camera_lidar_projector.cpp
  src/cross_modal_associator.cpp
  src/deterministic_replay.cpp
  src/hungarian_assignment.cpp
  src/lifecycle_policy.cpp
  src/memory_core.cpp
  src/memory_clock.cpp
  src/memory_domain.cpp
  src/memory_services.cpp
  src/motion_classifier.cpp
  src/multisensor_update.cpp
  src/ros_conversions.cpp
  src/reidentification.cpp
  src/reidentification_calibration.cpp
  src/runtime_association.cpp
  src/runtime_task_services.cpp
  src/source_time_buffer.cpp
  src/task_relevance_scorer.cpp
  src/visualization.cpp
)
target_include_directories(semantic_memory_primitives PUBLIC include)
ament_target_dependencies(
  semantic_memory_primitives
  geometry_msgs
  sensor_msgs
  std_msgs
  track_robot_interfaces
  visualization_msgs
)
target_link_libraries(semantic_memory_primitives nlohmann_json::nlohmann_json)

add_executable(semantic_memory_replay src/semantic_memory_replay.cpp)
target_include_directories(semantic_memory_replay PRIVATE include)
target_link_libraries(
  semantic_memory_replay
  semantic_memory_primitives
  nlohmann_json::nlohmann_json
)

add_executable(semantic_memory_node src/semantic_memory_node.cpp)
target_include_directories(semantic_memory_node PRIVATE include)
ament_target_dependencies(
  semantic_memory_node
  ament_index_cpp
  rclcpp
  tf2_ros
  geometry_msgs
  sensor_msgs
  visualization_msgs
  diagnostic_msgs
  track_robot_interfaces
)
target_link_libraries(semantic_memory_node semantic_memory_primitives)

add_executable(
  semantic_memory_visualizer_node
  src/semantic_memory_visualizer_node.cpp
)
target_include_directories(semantic_memory_visualizer_node PRIVATE include)
ament_target_dependencies(
  semantic_memory_visualizer_node
  rclcpp
  track_robot_interfaces
  visualization_msgs
)
target_link_libraries(
  semantic_memory_visualizer_node
  semantic_memory_primitives
)

if(BUILD_TESTING)
  # Foxy's gtest_vendor predates the CMake 4 policy floor.
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_id_types test/test_id_types.cpp)
  target_include_directories(test_id_types PRIVATE include)
  target_link_libraries(test_id_types semantic_memory_primitives)

  ament_add_gtest(test_memory_domain test/test_memory_domain.cpp)
  target_include_directories(test_memory_domain PRIVATE include)
  target_link_libraries(test_memory_domain semantic_memory_primitives)

  ament_add_gtest(test_source_time_buffer test/test_source_time_buffer.cpp)
  target_include_directories(test_source_time_buffer PRIVATE include)
  target_link_libraries(test_source_time_buffer semantic_memory_primitives)

  ament_add_gtest(test_lifecycle_policy test/test_lifecycle_policy.cpp)
  target_include_directories(test_lifecycle_policy PRIVATE include)
  target_link_libraries(test_lifecycle_policy semantic_memory_primitives)

  ament_add_gtest(test_motion_classifier test/test_motion_classifier.cpp)
  target_include_directories(test_motion_classifier PRIVATE include)
  target_link_libraries(test_motion_classifier semantic_memory_primitives)

  ament_add_gtest(test_memory_core test/test_memory_core.cpp)
  target_include_directories(test_memory_core PRIVATE include)
  target_link_libraries(test_memory_core semantic_memory_primitives)

  ament_add_gtest(test_ros_conversions test/test_ros_conversions.cpp)
  target_include_directories(test_ros_conversions PRIVATE include)
  ament_target_dependencies(
    test_ros_conversions
    geometry_msgs
    std_msgs
    track_robot_interfaces
  )
  target_link_libraries(test_ros_conversions semantic_memory_primitives)

  ament_add_gtest(test_visualization test/test_visualization.cpp)
  target_include_directories(test_visualization PRIVATE include)
  ament_target_dependencies(
    test_visualization
    track_robot_interfaces
    visualization_msgs
  )
  target_link_libraries(test_visualization semantic_memory_primitives)

  ament_add_gtest(
    test_camera_lidar_projector
    test/test_camera_lidar_projector.cpp
  )
  target_include_directories(test_camera_lidar_projector PRIVATE include)
  target_link_libraries(
    test_camera_lidar_projector
    semantic_memory_primitives
  )

  ament_add_gtest(test_association_terms test/test_association_terms.cpp)
  target_include_directories(test_association_terms PRIVATE include)
  target_link_libraries(test_association_terms semantic_memory_primitives)

  ament_add_gtest(
    test_cross_modal_associator
    test/test_cross_modal_associator.cpp
  )
  target_include_directories(test_cross_modal_associator PRIVATE include)
  target_link_libraries(
    test_cross_modal_associator
    semantic_memory_primitives
  )

  ament_add_gtest(
    test_hungarian_assignment
    test/test_hungarian_assignment.cpp
  )
  target_include_directories(test_hungarian_assignment PRIVATE include)
  target_link_libraries(
    test_hungarian_assignment
    semantic_memory_primitives
  )

  ament_add_gtest(
    test_association_confirmation
    test/test_association_confirmation.cpp
  )

  ament_add_gtest(
    test_association_calibration
    test/test_association_calibration.cpp
  )
  target_include_directories(test_association_calibration PRIVATE include)
  target_link_libraries(
    test_association_calibration
    semantic_memory_primitives
    nlohmann_json::nlohmann_json
  )
  target_include_directories(test_association_confirmation PRIVATE include)
  target_link_libraries(
    test_association_confirmation
    semantic_memory_primitives
  )

  ament_add_gtest(test_runtime_association test/test_runtime_association.cpp)
  target_include_directories(test_runtime_association PRIVATE include)
  target_link_libraries(test_runtime_association semantic_memory_primitives)

  ament_add_gtest(test_multisensor_update test/test_multisensor_update.cpp)
  target_include_directories(test_multisensor_update PRIVATE include)
  target_link_libraries(test_multisensor_update semantic_memory_primitives)

  ament_add_gtest(test_appearance_memory test/test_appearance_memory.cpp)
  target_include_directories(test_appearance_memory PRIVATE include)
  target_link_libraries(test_appearance_memory semantic_memory_primitives)

  ament_add_gtest(test_reidentification test/test_reidentification.cpp)
  target_include_directories(test_reidentification PRIVATE include)
  target_link_libraries(test_reidentification semantic_memory_primitives)

  ament_add_gtest(
    test_reidentification_calibration
    test/test_reidentification_calibration.cpp
  )
  target_include_directories(test_reidentification_calibration PRIVATE include)
  target_link_libraries(
    test_reidentification_calibration
    semantic_memory_primitives
    nlohmann_json::nlohmann_json
  )

  ament_add_gtest(
    test_task_relevance_scorer
    test/test_task_relevance_scorer.cpp
  )
  target_include_directories(test_task_relevance_scorer PRIVATE include)
  target_link_libraries(
    test_task_relevance_scorer
    semantic_memory_primitives
  )

  ament_add_gtest(test_memory_services test/test_memory_services.cpp)
  target_include_directories(test_memory_services PRIVATE include)
  target_link_libraries(test_memory_services semantic_memory_primitives)

  ament_add_gtest(
    test_runtime_task_services
    test/test_runtime_task_services.cpp
  )
  target_include_directories(test_runtime_task_services PRIVATE include)
  target_link_libraries(
    test_runtime_task_services
    semantic_memory_primitives
  )

  ament_add_gtest(
    test_deterministic_replay
    test/test_deterministic_replay.cpp
  )
  target_include_directories(test_deterministic_replay PRIVATE include)
  target_link_libraries(
    test_deterministic_replay
    semantic_memory_primitives
    nlohmann_json::nlohmann_json
  )

  ament_add_gtest(
    test_stage2b_acceptance
    test/test_stage2b_acceptance.cpp
  )
  target_include_directories(test_stage2b_acceptance PRIVATE include)
  ament_target_dependencies(
    test_stage2b_acceptance
    geometry_msgs
    track_robot_interfaces
  )
  target_link_libraries(test_stage2b_acceptance semantic_memory_primitives)

  # The host has an incompatible globally installed typeguard pytest plugin.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(
    test_launch_contract
    test/test_launch_contract.py
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_ros_runtime
    test/test_ros_runtime.py
    TIMEOUT 30
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

install(TARGETS
  semantic_memory_replay
  semantic_memory_node
  semantic_memory_visualizer_node
  semantic_memory_primitives
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../artifacts/semantic_search/reports/phase2_association_calibration_2026-07-16.json
  DESTINATION share/${PROJECT_NAME}/reports
)

ament_export_include_directories(include)
ament_export_libraries(semantic_memory_primitives)

ament_package()
