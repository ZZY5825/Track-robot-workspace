cmake_minimum_required(VERSION 3.5)
project(track_robot_interfaces)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/GestureState.msg"
  "msg/CameraTarget.msg"
  "msg/HumanDetection2D.msg"
  "msg/HumanDetection2DArray.msg"
  "msg/LidarCluster.msg"
  "msg/LidarClusterArray.msg"
  "msg/LidarTracklet.msg"
  "msg/LidarTrackletArray.msg"
  "msg/SelectedLidarTracklet.msg"
  "msg/TargetState.msg"
  "msg/SafetyState.msg"
  "msg/AvoidanceState.msg"
  "msg/FollowDecision.msg"
  "msg/PerceptionHealth.msg"
  "msg/SemanticRegion.msg"
  "msg/SemanticRegionArray.msg"
  "msg/ObjectObservation3D.msg"
  "msg/ObjectObservation3DArray.msg"
  "msg/TrackedSemanticObject.msg"
  "msg/TrackedSemanticObjectArray.msg"
  "msg/SearchMotionIntent.msg"
  "msg/VisualDescriptor.msg"
  "msg/SemanticLabelEvidence.msg"
  "msg/VisualProposal.msg"
  "msg/VisualProposalArray.msg"
  "msg/SemanticLocalizationState.msg"
  "msg/SemanticTask.msg"
  "msg/SemanticObservation.msg"
  "msg/SemanticObservationArray.msg"
  "msg/SemanticLidarTrackletArray.msg"
  "msg/SemanticObjectHistorySample.msg"
  "msg/SemanticObject.msg"
  "msg/SemanticObjectArray.msg"
  "msg/AssociationTerm.msg"
  "msg/AssociationDebug.msg"
  "msg/SemanticMemoryEvent.msg"
)

set(srv_files
  "srv/AuthorizeSemanticApproach.srv"
  "srv/GetSemanticObject.srv"
  "srv/QuerySemanticObjects.srv"
  "srv/MarkSemanticObjectInspected.srv"
  "srv/ResetSemanticMemory.srv"
)

set(action_files
  "action/SearchForObject.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The host has a globally installed typeguard plugin that is incompatible
  # with the pytest version shipped by ROS 2 Foxy.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} 1)
  ament_add_pytest_test(
    test_semantic_interfaces
    test/test_semantic_interfaces.py
    TIMEOUT 30
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
  ament_add_pytest_test(
    test_phase2_interfaces
    test/test_phase2_interfaces.py
    TIMEOUT 30
    ENV PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
  )
endif()

ament_package()
