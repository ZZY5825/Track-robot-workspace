# Semantic Search Phase 0 Contracts, Replay, and Profiling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (- [ ]) syntax for tracking.

**Goal:** Build the model-independent Phase 0 foundation for language-conditioned semantic search: stable ROS contracts, dataset manifests, localisation-mode health logic, deterministic replay evaluation, a passive launch, and a versioned baseline report.

**Architecture:** Add the approved semantic-search messages and action to the existing interface package, then create an opt-in ament_python package containing pure-Python validation/evaluation libraries wrapped by thin ROS nodes. Phase 0 launches no perception model, action server, controller, planner, or motion bridge; old bags are explicitly classified as legacy observation-only evidence.

**Tech Stack:** ROS 2 Foxy, Python 3.8, rclpy, tf2_ros, ament_python, rosidl/ament_cmake, std_msgs, sensor_msgs, geometry_msgs, nav_msgs, diagnostic_msgs, PyYAML 5.3, psutil, pytest 4.6, rosbag2 sqlite3, Jetson tegrastats.

## Global Constraints

- Authoritative design: docs/architecture/semantic-search/2026-07-13-language-conditioned-multimodal-semantic-search-design.md.
- Platform remains JetPack 5.0.2 / L4T R35.1, Ubuntu 20.04, ROS 2 Foxy, Python 3.8, PyTorch 1.13 NVIDIA, CUDA 11.4, and TensorRT 8.4.
- Do not upgrade or replace the working Python, ROS, PyTorch, CUDA, TensorRT, or JetPack environment in place.
- This plan implements package 1 only: contracts, replay, manifests, localisation health, and profiling. It does not implement DINO changes, a text encoder, semantic inference, 3D fusion, memory association, active rotation, navigation, or motion output.
- All new ROS names use /semantic_search. New launches are opt-in and default off.
- No file under the existing human-tracking launch/config/control/safety path is modified.
- The Phase 0 launch may publish diagnostics and reports only. It must not publish geometry_msgs/Twist, FollowDecision, SearchMotionIntent, /cmd_vel, or any motion-permission message.
- The current four bags lack IMU and odometry. Their manifest capability flags must force OBSERVATION_ONLY; they cannot pass world/local-session or active-motion gates.
- World memory is disabled by default. Enabling it later requires fresh world pose, finite covariance, stable samples, and no jump/reset.
- Large bags, weights, and raw telemetry stay outside Git. Small manifests, schemas, reports, and checksums are versioned.
- The user authorized a local-only Git history on 2026-07-14. Keep it local:
  configure no remote, never push automatically, track only source/config/docs
  and small evidence, and ignore build/install/log/model/dataset/raw-bag data.

---

## Scope-Locked File Map

### Modify

- src/track_robot/track_robot_interfaces/CMakeLists.txt — register semantic messages/action, dependencies, and contract tests.
- src/track_robot/track_robot_interfaces/package.xml — declare action_msgs, sensor_msgs, and test dependencies.

### Create: ROS interfaces

- src/track_robot/track_robot_interfaces/action/SearchForObject.action
- src/track_robot/track_robot_interfaces/msg/SemanticRegion.msg
- src/track_robot/track_robot_interfaces/msg/SemanticRegionArray.msg
- src/track_robot/track_robot_interfaces/msg/ObjectObservation3D.msg
- src/track_robot/track_robot_interfaces/msg/ObjectObservation3DArray.msg
- src/track_robot/track_robot_interfaces/msg/TrackedSemanticObject.msg
- src/track_robot/track_robot_interfaces/msg/TrackedSemanticObjectArray.msg
- src/track_robot/track_robot_interfaces/msg/SearchMotionIntent.msg
- src/track_robot/track_robot_interfaces/test/test_semantic_interfaces.py

### Create: runtime/offline Phase 0 package

- src/track_robot_semantic_search/package.xml
- src/track_robot_semantic_search/setup.py
- src/track_robot_semantic_search/setup.cfg
- src/track_robot_semantic_search/resource/track_robot_semantic_search
- src/track_robot_semantic_search/README.md
- src/track_robot_semantic_search/track_robot_semantic_search/__init__.py
- src/track_robot_semantic_search/track_robot_semantic_search/manifest.py
- src/track_robot_semantic_search/track_robot_semantic_search/manifest_cli.py
- src/track_robot_semantic_search/track_robot_semantic_search/localization_health.py
- src/track_robot_semantic_search/track_robot_semantic_search/localization_health_node.py
- src/track_robot_semantic_search/track_robot_semantic_search/evaluation.py
- src/track_robot_semantic_search/track_robot_semantic_search/evaluator_node.py
- src/track_robot_semantic_search/track_robot_semantic_search/compare_reports.py
- src/track_robot_semantic_search/config/semantic_search_phase0.yaml
- src/track_robot_semantic_search/launch/semantic_search_phase0.launch.py
- src/track_robot_semantic_search/schemas/dataset_manifest.schema.json
- src/track_robot_semantic_search/schemas/annotation.schema.json
- src/track_robot_semantic_search/schemas/evaluation_report.schema.json
- src/track_robot_semantic_search/test/test_manifest.py
- src/track_robot_semantic_search/test/test_localization_health.py
- src/track_robot_semantic_search/test/test_evaluation.py
- src/track_robot_semantic_search/test/test_launch_contract.py

### Create: data/replay documentation and evidence

- docs/guides/semantic-search/rosbag-workflow.md
- artifacts/semantic_search/manifests/README.md
- artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json
- artifacts/semantic_search/reports/README.md
- artifacts/semantic_search/reports/phase0_baseline_2026-07-14.json

No research/semantic_search model or training file is created in this package.

---

## Execution Preflight

- [ ] **Step 1: Verify the authorized local Git state**

Run:

~~~bash
cd /home/track-robot/track_robot_ws
git rev-parse --show-toplevel
git status --short
~~~

Expected: from the isolated implementation worktree, the first command prints
that worktree's root, the branch is feature/semantic-search-phase0, and no
remote is configured. The initial local-only baseline is created once before
this preflight; preserve unrelated user files and never push automatically.

- [ ] **Step 2: Record protected human-stack hashes**

Run:

~~~bash
sha256sum \
  src/track_robot_perception/launch/human_tracking_simplified.launch.py \
  src/track_robot_perception/launch/human_tracking_rosbag_replay.launch.py \
  src/track_robot_perception/config/human_tracking.yaml \
  src/track_robot/track_robot_decision/config/outdoor_decision.yaml \
  src/track_robot/track_robot_safety/config/motion_safety_supervisor.yaml \
  > /tmp/semantic_search_phase0_protected.sha256
~~~

Expected: exit 0 and five checksum lines. Task 7 rechecks this file.

- [ ] **Step 3: Verify the existing ROS build environment**

Run:

~~~bash
bash -lc 'source /opt/ros/foxy/setup.bash && cd /home/track-robot/track_robot_ws && colcon list | rg "track_robot_interfaces|track_robot_perception|track_robot_decision"'
~~~

Expected: all three packages are listed. Do not install missing framework versions in place.

---

### Task 1: Freeze the Semantic-Search ROS Contracts

**Files:**

- Create: src/track_robot/track_robot_interfaces/action/SearchForObject.action
- Create: the seven semantic-search .msg files listed in the file map
- Create: src/track_robot/track_robot_interfaces/test/test_semantic_interfaces.py
- Modify: src/track_robot/track_robot_interfaces/CMakeLists.txt:14-40
- Modify: src/track_robot/track_robot_interfaces/package.xml:10-21

**Interfaces:**

- Consumes: builtin_interfaces/Time and Duration, std_msgs/Header, geometry_msgs/Point and Vector3, sensor_msgs/RegionOfInterest.
- Produces: SearchForObject, SemanticRegion(Array), ObjectObservation3D(Array), TrackedSemanticObject(Array), and SearchMotionIntent.
- Invariants: ID zero is invalid for uint64 semantic IDs; validity booleans govern optional IDs/geometry; tensors are never fields; SearchMotionIntent.forward_permitted is false in Phase 0 and all first-release paths.

- [ ] **Step 1: Write the failing interface contract test**

Create src/track_robot/track_robot_interfaces/test/test_semantic_interfaces.py:

~~~python
from rclpy.serialization import deserialize_message, serialize_message

from track_robot_interfaces.action import SearchForObject
from track_robot_interfaces.msg import (
    ObjectObservation3D,
    ObjectObservation3DArray,
    SearchMotionIntent,
    SemanticRegion,
    SemanticRegionArray,
    TrackedSemanticObject,
    TrackedSemanticObjectArray,
)


def round_trip(message):
    return deserialize_message(serialize_message(message), type(message))


def test_action_constants_and_safe_defaults():
    goal = SearchForObject.Goal()
    result = SearchForObject.Result()
    feedback = SearchForObject.Feedback()
    assert goal.allow_rotation is False
    assert goal.maximum_rotation_angle == 0.0
    assert result.CONFIRMED == 0
    assert result.TIMEOUT == 7
    assert feedback.INITIALISING == 0
    assert feedback.TERMINAL == 4


def test_semantic_messages_round_trip_without_tensor_fields():
    region = SemanticRegion(query_id=1, query_version=1, observation_id=1)
    observation = ObjectObservation3D(
        query_id=1, query_version=1, observation_id=1, region=region)
    tracked = TrackedSemanticObject(global_object_id=1)
    intent = SearchMotionIntent(query_id=1)
    for message in (
            region,
            SemanticRegionArray(query_id=1, query_version=1, regions=[region]),
            observation,
            ObjectObservation3DArray(
                query_id=1, query_version=1, observations=[observation]),
            tracked,
            TrackedSemanticObjectArray(objects=[tracked]),
            intent):
        restored = round_trip(message)
        fields = restored.get_fields_and_field_types()
        assert not any(
            forbidden in name
            for name in fields
            for forbidden in ('tensor', 'embedding', 'feature_grid'))
    assert round_trip(intent).forward_permitted is False


def test_identifier_and_provenance_constants_are_distinct():
    assert ObjectObservation3D.EVIDENCE_CAMERA == 1
    assert ObjectObservation3D.EVIDENCE_LIDAR == 2
    assert ObjectObservation3D.EVIDENCE_STEREO_DEPTH == 4
    assert TrackedSemanticObject.MEMORY_OBSERVATION_ONLY == 0
    assert TrackedSemanticObject.MEMORY_LOCAL_SESSION == 1
    assert TrackedSemanticObject.MEMORY_WORLD == 2
    assert SearchMotionIntent.INTENT_ROTATE_VERIFY == 1
    assert SearchMotionIntent.INTENT_STOP == 2
~~~

- [ ] **Step 2: Run the test to verify the contracts do not exist**

Run:

~~~bash
bash -lc 'source /opt/ros/foxy/setup.bash && cd /home/track-robot/track_robot_ws && colcon test --packages-select track_robot_interfaces --event-handlers console_direct+'
~~~

Expected: FAIL because SearchForObject and the semantic message modules have not been generated.

- [ ] **Step 3: Add the exact message definitions**

Create SemanticRegion.msg:

~~~text
uint8 MASK_NONE=0
uint8 MASK_PNG=1
uint8 MASK_COCO_RLE_JSON=2

std_msgs/Header header
uint32 image_width
uint32 image_height
uint64 query_id
uint64 query_version
uint64 observation_id
sensor_msgs/RegionOfInterest roi
uint8 mask_encoding
uint8[] compressed_mask
float32 appearance_score
float32 language_score
float32 localization_score
float32 fused_score
float32 preprocessing_scale
uint32 padding_left
uint32 padding_top
uint32 model_input_width
uint32 model_input_height
string image_encoder_id
string text_encoder_id
string checkpoint_id
~~~

Create SemanticRegionArray.msg:

~~~text
std_msgs/Header header
uint64 query_id
uint64 query_version
track_robot_interfaces/SemanticRegion[] regions
~~~

Create ObjectObservation3D.msg:

~~~text
uint32 EVIDENCE_CAMERA=1
uint32 EVIDENCE_LIDAR=2
uint32 EVIDENCE_STEREO_DEPTH=4
uint32 EVIDENCE_PREDICTION=8
uint32 EVIDENCE_STATIC_FALLBACK_TF=16

std_msgs/Header header
uint64 query_id
uint64 query_version
uint64 observation_id
track_robot_interfaces/SemanticRegion region
bool position_base_valid
geometry_msgs/Point position_base
bool position_local_session_valid
string local_session_frame_id
geometry_msgs/Point position_local_session
bool position_world_valid
string world_frame_id
geometry_msgs/Point position_world
bool extent_valid
geometry_msgs/Vector3 extent
bool extent_is_partial
float32[9] position_covariance
float32 range
float32 bearing
uint32 projected_point_count
bool lidar_tracklet_id_valid
int64 lidar_tracklet_id
uint64 localization_epoch_id
builtin_interfaces/Duration image_age
builtin_interfaces/Duration lidar_age
builtin_interfaces/Duration tf_age
builtin_interfaces/Duration pose_age
uint32 evidence_flags
float32 identity_confidence
float32 semantic_confidence
float32 geometry_confidence
float32 overall_confidence
string geometry_source
string calibration_id
~~~

Create ObjectObservation3DArray.msg:

~~~text
std_msgs/Header header
uint64 query_id
uint64 query_version
track_robot_interfaces/ObjectObservation3D[] observations
~~~

Create TrackedSemanticObject.msg:

~~~text
uint8 MEMORY_OBSERVATION_ONLY=0
uint8 MEMORY_LOCAL_SESSION=1
uint8 MEMORY_WORLD=2

uint8 ASSOCIATION_TENTATIVE=0
uint8 ASSOCIATION_CONFIRMED=1
uint8 ASSOCIATION_AMBIGUOUS=2
uint8 ASSOCIATION_PREDICTION=3
uint8 ASSOCIATION_LOST=4

uint8 VISIBILITY_VISIBLE=0
uint8 VISIBILITY_OCCLUDED=1
uint8 VISIBILITY_OUT_OF_FOV=2
uint8 VISIBILITY_STALE=3

uint8 REID_NOT_REQUIRED=0
uint8 REID_PENDING=1
uint8 REID_CONFIRMED=2
uint8 REID_REJECTED=3

uint8 INSPECTION_NOT_INSPECTED=0
uint8 INSPECTION_REQUESTED=1
uint8 INSPECTION_COMPLETE=2

uint8 SUPPORT_NONE=0
uint8 SUPPORT_SENSOR=1
uint8 SUPPORT_PREDICTION_ONLY=2

std_msgs/Header header
uint64 global_object_id
uint64[] active_query_ids
float32[] query_relevance
uint64[] raw_observation_ids
bool camera_track_id_valid
int64 camera_track_id
bool lidar_tracklet_id_valid
int64 lidar_tracklet_id
uint8 memory_mode
uint64 localization_epoch_id
string position_frame_id
bool position_valid
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
bool extent_valid
geometry_msgs/Vector3 extent
float32[9] position_covariance
string appearance_summary_id
string geometry_summary_id
string semantic_summary_id
string[] semantic_labels
float32[] semantic_label_scores
string[] semantic_label_provenance
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
builtin_interfaces/Time last_camera_seen
builtin_interfaces/Time last_lidar_seen
uint32 observation_count
builtin_interfaces/Time[] history_stamps
geometry_msgs/Point[] position_history
float32[] uncertainty_history
uint8 association_state
uint8 visibility_state
uint8 reidentification_state
uint8 inspection_state
uint8 support_state
float32 association_confidence
float32 duplicate_score
float32 uncertainty
float32 task_relevance
float32 inspection_priority
bool anomaly_score_valid
float32 anomaly_score
~~~

Create TrackedSemanticObjectArray.msg:

~~~text
std_msgs/Header header
uint64 localization_epoch_id
uint8 memory_mode
track_robot_interfaces/TrackedSemanticObject[] objects
~~~

Create SearchMotionIntent.msg:

~~~text
uint8 INTENT_NONE=0
uint8 INTENT_ROTATE_VERIFY=1
uint8 INTENT_STOP=2

std_msgs/Header header
uint64 query_id
uint8 intent
float32 target_bearing
float32 maximum_rotation_angle
float32 maximum_angular_speed
builtin_interfaces/Time deadline
bool rotation_permitted
bool forward_permitted
string reason
~~~

- [ ] **Step 4: Add the exact action definition**

Create SearchForObject.action:

~~~text
# Goal
string query_text
builtin_interfaces/Duration timeout
bool allow_rotation
float32 maximum_rotation_angle
string client_request_id
---
# Result
uint8 CONFIRMED=0
uint8 NOT_FOUND=1
uint8 UNCERTAIN=2
uint8 CANCELLED=3
uint8 SAFETY_REJECTED=4
uint8 SENSOR_UNAVAILABLE=5
uint8 MODEL_UNAVAILABLE=6
uint8 TIMEOUT=7

uint8 status
uint64 query_id
bool selected_object_valid
uint64 selected_global_object_id
bool selected_observation_valid
track_robot_interfaces/ObjectObservation3D selected_observation
string evidence_summary
---
# Feedback
uint8 INITIALISING=0
uint8 PASSIVE_OBSERVATION=1
uint8 ROTATION_VERIFICATION=2
uint8 FINALISING=3
uint8 TERMINAL=4

uint64 query_id
uint8 phase
builtin_interfaces/Duration elapsed
float32 searched_angle
uint32 candidate_count
float32 best_candidate_score
string current_reason
~~~

- [ ] **Step 5: Register generation and contract tests**

Replace src/track_robot/track_robot_interfaces/CMakeLists.txt with:

~~~cmake
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
)

set(action_files
  "action/SearchForObject.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
  DEPENDENCIES action_msgs builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_semantic_interfaces
    test/test_semantic_interfaces.py
    TIMEOUT 30
  )
endif()

ament_package()
~~~

Update package.xml dependencies without changing its existing version, maintainer, or license fields:

~~~xml
  <depend>action_msgs</depend>
  <depend>builtin_interfaces</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_cmake_pytest</test_depend>
  <test_depend>rclpy</test_depend>
~~~

Remove the old duplicate builtin_interfaces/std_msgs/geometry_msgs dependency lines when inserting this block.

- [ ] **Step 6: Build and run the interface tests**

Run:

~~~bash
bash -lc 'source /opt/ros/foxy/setup.bash && cd /home/track-robot/track_robot_ws && colcon build --packages-select track_robot_interfaces --symlink-install --event-handlers console_direct+'
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && cd /home/track-robot/track_robot_ws && colcon test --packages-select track_robot_interfaces --event-handlers console_direct+'
bash -lc 'cd /home/track-robot/track_robot_ws && colcon test-result --verbose'
~~~

Expected: build succeeds; test_semantic_interfaces passes; test-result reports zero failures.

- [ ] **Step 7: Inspect the generated wire contract**

Run:

~~~bash
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && ros2 interface show track_robot_interfaces/action/SearchForObject'
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && ros2 interface show track_robot_interfaces/msg/SearchMotionIntent'
~~~

Expected: the action contains five goal fields, eight result statuses, and five feedback phases; SearchMotionIntent contains forward_permitted and no linear velocity field.

- [ ] **Step 8: Commit the interface contract**

~~~bash
git add \
  src/track_robot/track_robot_interfaces/CMakeLists.txt \
  src/track_robot/track_robot_interfaces/package.xml \
  src/track_robot/track_robot_interfaces/action/SearchForObject.action \
  src/track_robot/track_robot_interfaces/msg/SemanticRegion.msg \
  src/track_robot/track_robot_interfaces/msg/SemanticRegionArray.msg \
  src/track_robot/track_robot_interfaces/msg/ObjectObservation3D.msg \
  src/track_robot/track_robot_interfaces/msg/ObjectObservation3DArray.msg \
  src/track_robot/track_robot_interfaces/msg/TrackedSemanticObject.msg \
  src/track_robot/track_robot_interfaces/msg/TrackedSemanticObjectArray.msg \
  src/track_robot/track_robot_interfaces/msg/SearchMotionIntent.msg \
  src/track_robot/track_robot_interfaces/test/test_semantic_interfaces.py
git commit -m "feat: add semantic search interface contracts"
~~~

### Task 2: Create the Package and Versioned Dataset Manifest

**Files:**

- Create: src/track_robot_semantic_search/package.xml
- Create: src/track_robot_semantic_search/setup.py
- Create: src/track_robot_semantic_search/setup.cfg
- Create: src/track_robot_semantic_search/resource/track_robot_semantic_search
- Create: src/track_robot_semantic_search/README.md
- Create: src/track_robot_semantic_search/track_robot_semantic_search/__init__.py
- Create: src/track_robot_semantic_search/track_robot_semantic_search/manifest.py
- Create: src/track_robot_semantic_search/track_robot_semantic_search/manifest_cli.py
- Create: src/track_robot_semantic_search/schemas/dataset_manifest.schema.json
- Create: src/track_robot_semantic_search/schemas/annotation.schema.json
- Create: src/track_robot_semantic_search/schemas/evaluation_report.schema.json
- Create: src/track_robot_semantic_search/test/test_manifest.py

**Interfaces:**

- Produces: load_manifest(path) -> dict, validate_manifest(payload) -> None,
  add_query_event(payload, event) -> dict,
  add_object(payload, object_record) -> dict,
  add_trial(payload, trial_record) -> dict,
  add_annotation_file(payload, path, workspace_root) -> dict,
  sha256_tree(path) -> str, and manifest builders that require an explicit
  workspace_root for safe relative paths.
- CLI: semantic_search_manifest validate MANIFEST; create-legacy BAG_DIR
  OUTPUT; create-field BAG_DIR OUTPUT with explicit workspace root plus
  split/site/calibration arguments; add-query MANIFEST --query-id ID
  --stamp-ns NS --text TEXT; add-object and add-trial with explicit metadata;
  and add-annotations MANIFEST JSONL with an explicit workspace root.
- Dataset rule: a “recording” is a bag directory plus an adjacent manifest/annotation bundle. Foxy rosbag2 does not reliably preserve action service goal payloads, so query text and timestamps are authoritative in the manifest rather than inferred from action feedback.

- [ ] **Step 1: Write the failing manifest tests**

Create src/track_robot_semantic_search/test/test_manifest.py:

~~~python
import json

import pytest
import yaml

from track_robot_semantic_search.manifest import (
    ManifestError,
    add_annotation_file,
    add_object,
    add_query_event,
    add_trial,
    build_field_manifest,
    build_legacy_manifest,
    load_manifest,
    sha256_tree,
    validate_manifest,
    write_json_atomic,
)


def valid_manifest():
    return {
        'schema_version': '1.0.0',
        'dataset_id': 'unit_test_bag',
        'split': 'legacy_replay_only',
        'bag': {
            'relative_path': 'bags/unit_test_bag',
            'sha256': '0' * 64,
            'storage_id': 'sqlite3',
            'start_time_ns': 1,
            'duration_ns': 10,
            'topics': [
                {'name': '/camera', 'type': 'sensor_msgs/msg/Image', 'count': 2},
            ],
        },
        'capabilities': {
            'camera': True,
            'lidar': False,
            'imu': False,
            'local_pose': False,
            'world_pose': False,
            'query_events': False,
            'annotations': False,
            'active_motion': False,
        },
        'calibration': {
            'camera_intrinsics_id': 'unknown',
            'camera_lidar_extrinsics_id': 'unknown',
            'lidar_imu_extrinsics_id': 'unknown',
            'localization_config_id': 'none',
        },
        'environment': {
            'site_id': 'legacy_unknown',
            'session_id': 'unit',
            'lighting': 'unknown',
            'surface': 'unknown',
            'weather': 'unknown',
        },
        'queries': [],
        'annotation_files': [],
        'objects': [],
        'trials': [],
        'provenance': {
            'created_at': '2026-07-13T00:00:00Z',
            'created_by': 'unit_test',
            'notes': 'legacy replay only',
        },
    }


def test_valid_manifest_round_trip(tmp_path):
    payload = valid_manifest()
    validate_manifest(payload)
    path = tmp_path / 'manifest.json'
    write_json_atomic(path, payload)
    assert load_manifest(path) == payload


@pytest.mark.parametrize(
    'mutate',
    [
        lambda payload: payload.update(schema_version='2.0.0'),
        lambda payload: payload['bag'].update(relative_path='/absolute/path'),
        lambda payload: payload['bag'].update(sha256='bad'),
        lambda payload: payload.update(split='random'),
        lambda payload: payload.update(split='validation'),
        lambda payload: payload['capabilities'].pop('imu'),
        lambda payload: payload['capabilities'].update(annotations=True),
    ],
)
def test_invalid_manifest_is_rejected(mutate):
    payload = valid_manifest()
    mutate(payload)
    with pytest.raises(ManifestError):
        validate_manifest(payload)


def test_query_events_are_positive_unique_and_timestamped():
    payload = valid_manifest()
    event = {
        'query_id': 1,
        'stamp_ns': 5,
        'text': 'fallen branch blocking the path',
        'language': 'en',
        'client_request_id': 'field-001',
    }
    updated = add_query_event(payload, event)
    assert updated['capabilities']['query_events'] is True
    assert updated['queries'] == [event]
    with pytest.raises(ManifestError):
        add_query_event(updated, event)
    outside = dict(event, query_id=2, stamp_ns=100)
    with pytest.raises(ManifestError):
        add_query_event(updated, outside)


def test_annotation_file_is_validated_hashed_and_registered(tmp_path):
    payload = add_query_event(valid_manifest(), {
        'query_id': 1,
        'stamp_ns': 5,
        'text': 'fallen branch blocking the path',
        'language': 'en',
        'client_request_id': 'field-001',
    })
    payload = add_object(payload, {
        'object_id': 'object-1',
        'physical_object_id': 'branch-1',
        'labels': ['fallen branch'],
        'site_id': 'site-a',
        'acquisition_date': '2026-07-13',
        'source': 'robot',
        'provenance': 'human-labelled',
    })
    payload = add_trial(payload, {
        'trial_id': 'trial-1',
        'query_id': 1,
        'target_object_id': 'object-1',
        'is_positive': True,
        'start_stamp_ns': 2,
        'end_stamp_ns': 10,
        'nominal_distance_m': 2.0,
        'observation_stage': 'passive',
        'site_id': 'site-a',
        'session_id': 'unit',
    })
    annotation = tmp_path / 'annotations' / 'trial-1.jsonl'
    annotation.parent.mkdir()
    annotation.write_text(json.dumps({
        'schema_version': '1.0.0',
        'dataset_id': 'unit_test_bag',
        'trial_id': 'trial-1',
        'stamp_ns': 5,
        'query_id': 1,
        'object_id': 'object-1',
        'bbox_xywh': [1.0, 2.0, 3.0, 4.0],
        'mask_path': None,
        'position_base_m': [1.0, 0.0, 0.0],
        'extent_m': [0.5, 0.2, 0.2],
        'visibility': 'visible',
        'label_source': 'human',
        'confidence': 1.0,
    }) + '\n', encoding='utf-8')
    updated = add_annotation_file(payload, annotation, tmp_path)
    assert updated['capabilities']['annotations'] is True
    assert updated['annotation_files'][0]['relative_path'] == (
        'annotations/trial-1.jsonl')
    assert len(updated['annotation_files'][0]['sha256']) == 64


def test_build_legacy_manifest_reads_rosbag_metadata(tmp_path):
    bag = tmp_path / 'rosbags' / 'legacy_bag'
    bag.mkdir(parents=True)
    (bag / 'legacy_bag_0.db3').write_bytes(b'sqlite')
    metadata = {
        'rosbag2_bagfile_information': {
            'storage_identifier': 'sqlite3',
            'starting_time': {'nanoseconds_since_epoch': 12},
            'duration': {'nanoseconds': 34},
            'topics_with_message_count': [{
                'topic_metadata': {
                    'name': '/camera',
                    'type': 'sensor_msgs/msg/Image',
                },
                'message_count': 5,
            }],
        },
    }
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
    payload = build_legacy_manifest(bag, 'legacy_bag', tmp_path)
    validate_manifest(payload)
    assert payload['bag']['relative_path'] == 'rosbags/legacy_bag'
    assert payload['bag']['sha256'] == sha256_tree(bag)
    assert payload['capabilities']['imu'] is False
    assert payload['split'] == 'legacy_replay_only'


def test_bag_checksum_ignores_volatile_sqlite_sidecars(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'bag_0.db3').write_bytes(b'stable')
    (bag / 'metadata.yaml').write_text('stable', encoding='utf-8')
    (bag / 'bag_0.db3-wal').write_bytes(b'first')
    (bag / 'bag_0.db3-shm').write_bytes(b'first')
    first = sha256_tree(bag)
    (bag / 'bag_0.db3-wal').write_bytes(b'second')
    (bag / 'bag_0.db3-shm').write_bytes(b'second')
    assert sha256_tree(bag) == first


def test_build_field_manifest_requires_declared_split_and_calibration(tmp_path):
    bag = tmp_path / 'rosbags' / 'semantic_search' / 'raw' / 'field_bag'
    bag.mkdir(parents=True)
    (bag / 'field_bag_0.db3').write_bytes(b'sqlite')
    metadata = {
        'rosbag2_bagfile_information': {
            'storage_identifier': 'sqlite3',
            'starting_time': {'nanoseconds_since_epoch': 12},
            'duration': {'nanoseconds': 34},
            'topics_with_message_count': [{
                'topic_metadata': {
                    'name': '/odom',
                    'type': 'nav_msgs/msg/Odometry',
                },
                'message_count': 5,
            }, {
                'topic_metadata': {
                    'name': '/imu/data_raw',
                    'type': 'sensor_msgs/msg/Imu',
                },
                'message_count': 5,
            }],
        },
    }
    (bag / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
    payload = build_field_manifest(
        bag_dir=bag,
        dataset_id='field_bag',
        workspace_root=tmp_path,
        split='validation',
        environment={
            'site_id': 'site_a',
            'session_id': 'session_a',
            'lighting': 'day',
            'surface': 'path',
            'weather': 'dry',
        },
        calibration={
            'camera_intrinsics_id': 'camera-sha',
            'camera_lidar_extrinsics_id': 'camera-lidar-sha',
            'lidar_imu_extrinsics_id': 'lidar-imu-sha',
            'localization_config_id': 'localization-sha',
        },
        world_pose=False,
        active_motion=False,
    )
    assert payload['split'] == 'validation'
    assert payload['bag']['relative_path'] == (
        'rosbags/semantic_search/recordings/field_bag')
    assert payload['capabilities']['imu'] is True
    assert payload['capabilities']['local_pose'] is True
~~~

- [ ] **Step 2: Run the tests to verify the package does not exist**

Run:

~~~bash
cd /home/track-robot/track_robot_ws
PYTHONPATH=src/track_robot_semantic_search pytest -q \
  src/track_robot_semantic_search/test/test_manifest.py
~~~

Expected: collection FAIL with ModuleNotFoundError: track_robot_semantic_search.

- [ ] **Step 3: Add the package metadata**

Create package.xml:

~~~xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
  schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>track_robot_semantic_search</name>
  <version>0.1.0</version>
  <description>Phase 0 contracts, replay evaluation, and health tools for semantic search.</description>
  <maintainer email="track-robot@example.com">track-robot</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>diagnostic_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>python3-psutil</exec_depend>
  <exec_depend>python3-yaml</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>track_robot_interfaces</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
~~~

Create setup.py:

~~~python
from glob import glob
from setuptools import setup


package_name = 'track_robot_semantic_search'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/schemas', glob('schemas/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='track-robot',
    maintainer_email='track-robot@example.com',
    description='Semantic-search Phase 0 contracts and replay tools.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_search_manifest = '
            'track_robot_semantic_search.manifest_cli:main',
            'semantic_search_localization_health = '
            'track_robot_semantic_search.localization_health_node:main',
            'semantic_search_evaluator = '
            'track_robot_semantic_search.evaluator_node:main',
            'semantic_search_compare_reports = '
            'track_robot_semantic_search.compare_reports:main',
        ],
    },
)
~~~

Create setup.cfg:

~~~ini
[develop]
script_dir=$base/lib/track_robot_semantic_search
[install]
install_scripts=$base/lib/track_robot_semantic_search
~~~

Create resource/track_robot_semantic_search as an empty file. Create track_robot_semantic_search/__init__.py:

~~~python
"""Model-independent Phase 0 utilities for semantic search."""

__version__ = '0.1.0'
~~~

Create README.md:

~~~markdown
# Track Robot Semantic Search

This opt-in ROS 2 Foxy package owns Phase 0 semantic-search manifests,
localisation-mode health, replay evaluation, and passive diagnostics.

It deliberately contains no perception model, action server, controller,
planner, or cmd_vel publisher. See the approved design and the semantic-search
rosbag guide before enabling later phases.
~~~

- [ ] **Step 4: Implement strict manifest validation**

Create track_robot_semantic_search/manifest.py:

~~~python
import copy
import hashlib
import json
import math
import os
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Mapping

import yaml


SCHEMA_VERSION = '1.0.0'
SPLITS = {'train', 'validation', 'test', 'extension', 'legacy_replay_only'}
CAPABILITY_KEYS = {
    'camera', 'lidar', 'imu', 'local_pose', 'world_pose',
    'query_events', 'annotations', 'active_motion',
}


class ManifestError(ValueError):
    """Raised when a dataset manifest violates the Phase 0 contract."""


def _require_mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ManifestError('{} must be an object'.format(name))
    return value


def _require_keys(value: Mapping[str, Any], keys, name: str) -> None:
    missing = sorted(set(keys) - set(value))
    if missing:
        raise ManifestError('{} missing {}'.format(name, ', '.join(missing)))


def _relative_path(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ManifestError('{} must be a non-empty string'.format(name))
    path = Path(value)
    if path.is_absolute() or '..' in path.parts:
        raise ManifestError('{} must be a safe relative path'.format(name))
    return value


def sha256_tree(path: Path) -> str:
    path = Path(path)
    digest = hashlib.sha256()
    files = sorted(
        item for item in path.rglob('*')
        if item.is_file() and
        item.name not in ('.DS_Store',) and
        not item.name.endswith(('.db3-wal', '.db3-shm')))
    for item in files:
        relative = item.relative_to(path).as_posix().encode('utf-8')
        digest.update(len(relative).to_bytes(8, 'big'))
        digest.update(relative)
        with item.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    return digest.hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def workspace_relative_path(path: Path, workspace_root: Path) -> str:
    path = Path(path).resolve()
    workspace_root = Path(workspace_root).resolve()
    try:
        relative = path.relative_to(workspace_root)
    except ValueError as error:
        raise ManifestError(
            'bag directory must be inside workspace_root') from error
    return _relative_path(relative.as_posix(), 'bag.relative_path')


def validate_manifest(payload: Mapping[str, Any]) -> None:
    payload = _require_mapping(payload, 'manifest')
    manifest_keys = {
        'schema_version', 'dataset_id', 'split', 'bag', 'capabilities',
        'calibration', 'environment', 'queries', 'annotation_files',
        'objects', 'trials', 'provenance',
    }
    _require_keys(payload, manifest_keys, 'manifest')
    if set(payload) != manifest_keys:
        raise ManifestError('manifest contains unknown fields')
    if payload['schema_version'] != SCHEMA_VERSION:
        raise ManifestError('unsupported schema_version')
    if not isinstance(payload['dataset_id'], str) or not payload['dataset_id']:
        raise ManifestError('dataset_id must be non-empty')
    if payload['split'] not in SPLITS:
        raise ManifestError('unsupported split')

    bag = _require_mapping(payload['bag'], 'bag')
    _require_keys(
        bag,
        {
            'relative_path', 'sha256', 'storage_id', 'start_time_ns',
            'duration_ns', 'topics',
        },
        'bag',
    )
    if set(bag) != {
            'relative_path', 'sha256', 'storage_id', 'start_time_ns',
            'duration_ns', 'topics'}:
        raise ManifestError('bag contains unknown fields')
    _relative_path(bag['relative_path'], 'bag.relative_path')
    checksum = bag['sha256']
    if (not isinstance(checksum, str) or len(checksum) != 64 or
            any(char not in '0123456789abcdef' for char in checksum)):
        raise ManifestError('bag.sha256 must be 64 lowercase hex characters')
    for key in ('start_time_ns', 'duration_ns'):
        if not isinstance(bag[key], int) or bag[key] < 0:
            raise ManifestError('bag.{} must be a non-negative integer'.format(key))
    if not isinstance(bag['topics'], list):
        raise ManifestError('bag.topics must be an array')
    if not isinstance(bag['storage_id'], str) or not bag['storage_id']:
        raise ManifestError('bag.storage_id must be non-empty')
    topic_names = set()
    topic_types = {}
    for topic in bag['topics']:
        topic = _require_mapping(topic, 'bag topic')
        _require_keys(topic, {'name', 'type', 'count'}, 'bag topic')
        if set(topic) != {'name', 'type', 'count'}:
            raise ManifestError('bag topic contains unknown fields')
        if not isinstance(topic['name'], str) or not topic['name']:
            raise ManifestError('topic name must be non-empty')
        if not isinstance(topic['type'], str) or not topic['type']:
            raise ManifestError('topic type must be non-empty')
        if topic['name'] in topic_names:
            raise ManifestError('duplicate topic {}'.format(topic['name']))
        topic_names.add(topic['name'])
        topic_types[topic['name']] = topic['type']
        if not isinstance(topic['count'], int) or topic['count'] < 0:
            raise ManifestError('topic count must be non-negative')

    capabilities = _require_mapping(payload['capabilities'], 'capabilities')
    _require_keys(capabilities, CAPABILITY_KEYS, 'capabilities')
    unknown_capabilities = sorted(set(capabilities) - CAPABILITY_KEYS)
    if unknown_capabilities:
        raise ManifestError(
            'unknown capabilities {}'.format(
                ', '.join(unknown_capabilities)))
    if any(not isinstance(capabilities[key], bool) for key in CAPABILITY_KEYS):
        raise ManifestError('capabilities must be boolean')
    if capabilities['world_pose'] and not (
            capabilities['local_pose'] and capabilities['imu']):
        raise ManifestError('world_pose requires local_pose and imu')
    if capabilities['active_motion'] and not (
            capabilities['imu'] and capabilities['local_pose']):
        raise ManifestError('active_motion requires imu and local_pose')
    sensed_capabilities = {
        'camera': any(
            value == 'sensor_msgs/msg/Image'
            for value in topic_types.values()),
        'lidar': any(
            value == 'sensor_msgs/msg/PointCloud2'
            for value in topic_types.values()),
        'imu': any(
            value == 'sensor_msgs/msg/Imu'
            for value in topic_types.values()),
        'local_pose': topic_types.get('/odom') == 'nav_msgs/msg/Odometry',
    }
    for name, present in sensed_capabilities.items():
        if capabilities[name] != present:
            raise ManifestError(
                '{} capability contradicts bag topics'.format(name))
    if capabilities['world_pose'] and (
            topic_types.get('/localization/odometry') !=
            'nav_msgs/msg/Odometry'):
        raise ManifestError(
            'world_pose capability requires localization odometry evidence')
    active_evidence = {
        '/semantic_search/motion_intent':
        'track_robot_interfaces/msg/SearchMotionIntent',
        '/safety/state': 'track_robot_interfaces/msg/SafetyState',
        '/follow/cmd_vel_planned': 'geometry_msgs/msg/Twist',
        '/follow/cmd_vel_safe': 'geometry_msgs/msg/Twist',
    }
    if capabilities['active_motion'] and any(
            topic_types.get(name) != message_type
            for name, message_type in active_evidence.items()):
        raise ManifestError(
            'active_motion requires intent and safety-chain evidence topics')

    calibration_keys = {
        'camera_intrinsics_id',
        'camera_lidar_extrinsics_id',
        'lidar_imu_extrinsics_id',
        'localization_config_id',
    }
    calibration = _require_mapping(
        payload['calibration'], 'calibration')
    _require_keys(calibration, calibration_keys, 'calibration')
    if set(calibration) != calibration_keys:
        raise ManifestError('calibration contains unknown fields')
    if any(
            not isinstance(calibration[key], str) or
            not calibration[key].strip()
            for key in calibration_keys):
        raise ManifestError('calibration IDs must be non-empty strings')
    if payload['split'] != 'legacy_replay_only' and any(
            calibration[key].strip().lower() in {
                'unknown', 'none', 'unverified_legacy_tf'}
            for key in calibration_keys):
        raise ManifestError(
            'field manifests require verified calibration IDs')

    environment_keys = {
        'site_id', 'session_id', 'lighting', 'surface', 'weather'}
    environment = _require_mapping(
        payload['environment'], 'environment')
    _require_keys(environment, environment_keys, 'environment')
    if set(environment) != environment_keys:
        raise ManifestError('environment contains unknown fields')
    if any(
            not isinstance(environment[key], str) or
            not environment[key].strip()
            for key in environment_keys):
        raise ManifestError(
            'environment values must be non-empty strings')
    if payload['split'] != 'legacy_replay_only' and any(
            environment[key].strip().lower() in {
                'unknown', 'legacy_unknown'}
            for key in environment_keys):
        raise ManifestError(
            'field manifests require known environment values')

    for collection in ('queries', 'annotation_files', 'objects', 'trials'):
        if not isinstance(payload[collection], list):
            raise ManifestError('{} must be an array'.format(collection))
    query_ids = set()
    for query in payload['queries']:
        query = _require_mapping(query, 'query event')
        query_keys = {
            'query_id', 'stamp_ns', 'text', 'language', 'client_request_id'}
        _require_keys(query, query_keys, 'query event')
        if set(query) != query_keys:
            raise ManifestError('query event contains unknown fields')
        if not isinstance(query['query_id'], int) or query['query_id'] <= 0:
            raise ManifestError('query_id must be positive')
        if query['query_id'] in query_ids:
            raise ManifestError('duplicate query_id')
        query_ids.add(query['query_id'])
        if not isinstance(query['stamp_ns'], int) or query['stamp_ns'] < 0:
            raise ManifestError('query stamp_ns must be non-negative')
        if not (
                bag['start_time_ns'] <= query['stamp_ns'] <=
                bag['start_time_ns'] + bag['duration_ns']):
            raise ManifestError('query stamp_ns is outside the bag interval')
        if not isinstance(query['text'], str) or not query['text'].strip():
            raise ManifestError('query text must be non-empty')
        if not isinstance(query['language'], str) or not \
                query['language'].strip():
            raise ManifestError('query language must be non-empty')
        if not isinstance(query['client_request_id'], str):
            raise ManifestError('client_request_id must be a string')
    if bool(payload['queries']) != capabilities['query_events']:
        raise ManifestError('query_events capability must match queries')

    annotation_paths = set()
    annotation_keys = {
        'relative_path', 'sha256', 'format', 'schema_version'}
    for annotation in payload['annotation_files']:
        annotation = _require_mapping(annotation, 'annotation file')
        _require_keys(annotation, annotation_keys, 'annotation file')
        if set(annotation) != annotation_keys:
            raise ManifestError('annotation file contains unknown fields')
        path = _relative_path(
            annotation['relative_path'], 'annotation relative_path')
        if path in annotation_paths:
            raise ManifestError('duplicate annotation relative_path')
        annotation_paths.add(path)
        if not re.fullmatch(r'[0-9a-f]{64}', str(annotation['sha256'])):
            raise ManifestError('annotation sha256 must be lowercase SHA-256')
        if annotation['format'] != 'jsonl':
            raise ManifestError('annotation format must be jsonl')
        if annotation['schema_version'] != SCHEMA_VERSION:
            raise ManifestError('annotation schema_version is unsupported')
    if bool(payload['annotation_files']) != capabilities['annotations']:
        raise ManifestError(
            'annotations capability must match annotation_files')

    object_ids = set()
    object_keys = {
        'object_id', 'physical_object_id', 'labels', 'site_id',
        'acquisition_date', 'source', 'provenance'}
    for item in payload['objects']:
        item = _require_mapping(item, 'object')
        _require_keys(item, object_keys, 'object')
        if set(item) != object_keys:
            raise ManifestError('object contains unknown fields')
        for key in (
                'object_id', 'physical_object_id', 'site_id',
                'acquisition_date', 'provenance'):
            if not isinstance(item[key], str) or not item[key].strip():
                raise ManifestError('object {} must be non-empty'.format(key))
        if item['object_id'] in object_ids:
            raise ManifestError('duplicate object_id')
        object_ids.add(item['object_id'])
        try:
            datetime.strptime(item['acquisition_date'], '%Y-%m-%d')
        except ValueError as error:
            raise ManifestError(
                'object acquisition_date must be YYYY-MM-DD') from error
        if not isinstance(item['labels'], list) or not item['labels'] or any(
                not isinstance(label, str) or not label.strip()
                for label in item['labels']):
            raise ManifestError('object labels must be non-empty strings')
        if item['source'] not in {'robot', 'public', 'synthetic'}:
            raise ManifestError('object source is unsupported')

    trial_ids = set()
    trial_keys = {
        'trial_id', 'query_id', 'target_object_id', 'is_positive',
        'start_stamp_ns', 'end_stamp_ns', 'nominal_distance_m',
        'observation_stage', 'site_id', 'session_id'}
    for trial in payload['trials']:
        trial = _require_mapping(trial, 'trial')
        _require_keys(trial, trial_keys, 'trial')
        if set(trial) != trial_keys:
            raise ManifestError('trial contains unknown fields')
        if not isinstance(trial['trial_id'], str) or not trial['trial_id']:
            raise ManifestError('trial_id must be non-empty')
        if trial['trial_id'] in trial_ids:
            raise ManifestError('duplicate trial_id')
        trial_ids.add(trial['trial_id'])
        if trial['query_id'] not in query_ids:
            raise ManifestError('trial query_id is not declared')
        if not isinstance(trial['is_positive'], bool):
            raise ManifestError('trial is_positive must be boolean')
        target = trial['target_object_id']
        if trial['is_positive'] and target not in object_ids:
            raise ManifestError('positive trial target is not declared')
        if not trial['is_positive'] and target:
            raise ManifestError('negative trial target_object_id must be empty')
        start = trial['start_stamp_ns']
        end = trial['end_stamp_ns']
        if not isinstance(start, int) or not isinstance(end, int) or not (
                bag['start_time_ns'] <= start <= end <=
                bag['start_time_ns'] + bag['duration_ns']):
            raise ManifestError('trial interval is outside the bag')
        distance = trial['nominal_distance_m']
        if not isinstance(distance, (int, float)) or not \
                math.isfinite(distance) or distance < 0.0:
            raise ManifestError('trial nominal_distance_m is invalid')
        if trial['observation_stage'] not in {
                'passive', 'pre_rotation', 'post_rotation'}:
            raise ManifestError('trial observation_stage is unsupported')
        for key in ('site_id', 'session_id'):
            if not isinstance(trial[key], str) or not trial[key].strip():
                raise ManifestError('trial {} must be non-empty'.format(key))

    provenance = _require_mapping(payload['provenance'], 'provenance')
    _require_keys(
        provenance, {'created_at', 'created_by', 'notes'}, 'provenance')
    if set(provenance) != {'created_at', 'created_by', 'notes'}:
        raise ManifestError('provenance contains unknown fields')
    if not isinstance(provenance['created_at'], str) or not \
            provenance['created_at'].strip():
        raise ManifestError('provenance.created_at must be non-empty')
    if not isinstance(provenance['created_by'], str) or not \
            provenance['created_by'].strip():
        raise ManifestError('provenance.created_by must be non-empty')
    if not isinstance(provenance['notes'], str):
        raise ManifestError('provenance.notes must be a string')


def load_manifest(path: Path) -> Dict[str, Any]:
    with Path(path).open('r', encoding='utf-8') as stream:
        payload = json.load(stream)
    validate_manifest(payload)
    return payload


def write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, sort_keys=True)
        stream.write('\n')
    os.replace(str(temporary), str(path))


def add_query_event(
        payload: Mapping[str, Any], event: Mapping[str, Any]) -> Dict[str, Any]:
    updated = copy.deepcopy(dict(payload))
    updated['queries'].append(dict(event))
    updated['capabilities']['query_events'] = True
    validate_manifest(updated)
    return updated


def add_object(
        payload: Mapping[str, Any],
        object_record: Mapping[str, Any]) -> Dict[str, Any]:
    updated = copy.deepcopy(dict(payload))
    updated['objects'].append(dict(object_record))
    validate_manifest(updated)
    return updated


def add_trial(
        payload: Mapping[str, Any],
        trial_record: Mapping[str, Any]) -> Dict[str, Any]:
    updated = copy.deepcopy(dict(payload))
    updated['trials'].append(dict(trial_record))
    validate_manifest(updated)
    return updated


def validate_annotation_record(
        record: Mapping[str, Any], manifest: Mapping[str, Any]) -> None:
    record = _require_mapping(record, 'annotation record')
    required = {
        'schema_version', 'dataset_id', 'trial_id', 'stamp_ns', 'query_id',
        'object_id', 'visibility', 'label_source', 'confidence'}
    optional = {
        'bbox_xywh', 'mask_path', 'position_base_m', 'extent_m'}
    _require_keys(record, required, 'annotation record')
    if not set(record) <= required | optional:
        raise ManifestError('annotation record contains unknown fields')
    if record['schema_version'] != SCHEMA_VERSION:
        raise ManifestError('annotation schema_version is unsupported')
    if record['dataset_id'] != manifest['dataset_id']:
        raise ManifestError('annotation dataset_id does not match manifest')
    if not isinstance(record['trial_id'], str) or not record['trial_id']:
        raise ManifestError('annotation trial_id must be non-empty')
    if not isinstance(record['query_id'], int) or record['query_id'] <= 0:
        raise ManifestError('annotation query_id must be positive')
    if not isinstance(record['object_id'], str) or not record['object_id']:
        raise ManifestError('annotation object_id must be non-empty')
    trials = {
        item['trial_id']: item for item in manifest['trials']}
    trial = trials.get(record['trial_id'])
    if trial is None:
        raise ManifestError('annotation trial_id is not declared')
    if record['query_id'] != trial['query_id']:
        raise ManifestError('annotation query_id does not match trial')
    if record['object_id'] not in {
            item['object_id'] for item in manifest['objects']}:
        raise ManifestError('annotation object_id is not declared')
    stamp = record['stamp_ns']
    if not isinstance(stamp, int) or not (
            trial['start_stamp_ns'] <= stamp <= trial['end_stamp_ns']):
        raise ManifestError('annotation stamp_ns is outside the trial')
    if record['visibility'] not in {
            'visible', 'partial', 'occluded', 'out_of_fov'}:
        raise ManifestError('annotation visibility is unsupported')
    if record['label_source'] not in {
            'human', 'teacher', 'synthetic', 'temporal_pseudo'}:
        raise ManifestError('annotation label_source is unsupported')
    confidence = record['confidence']
    if not isinstance(confidence, (int, float)) or not \
            math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ManifestError('annotation confidence is invalid')

    def vector(name, length, non_negative=False):
        value = record.get(name)
        if value is None:
            return
        if not isinstance(value, list) or len(value) != length or any(
                not isinstance(item, (int, float)) or not math.isfinite(item)
                for item in value):
            raise ManifestError('{} is invalid'.format(name))
        if non_negative and any(item < 0.0 for item in value):
            raise ManifestError('{} must be non-negative'.format(name))

    vector('bbox_xywh', 4)
    if record.get('bbox_xywh') is not None and any(
            value < 0.0 for value in record['bbox_xywh'][2:]):
        raise ManifestError('bbox width/height must be non-negative')
    vector('position_base_m', 3)
    vector('extent_m', 3, non_negative=True)
    mask_path = record.get('mask_path')
    if mask_path is not None:
        _relative_path(mask_path, 'annotation mask_path')


def add_annotation_file(
        payload: Mapping[str, Any], annotation_path: Path,
        workspace_root: Path) -> Dict[str, Any]:
    validate_manifest(payload)
    annotation_path = Path(annotation_path)
    if annotation_path.suffix != '.jsonl':
        raise ManifestError('annotation file must use .jsonl')
    record_count = 0
    with annotation_path.open('r', encoding='utf-8') as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ManifestError(
                    'invalid annotation JSON on line {}'.format(
                        line_number)) from error
            validate_annotation_record(record, payload)
            record_count += 1
    if record_count == 0:
        raise ManifestError('annotation file contains no records')
    updated = copy.deepcopy(dict(payload))
    updated['annotation_files'].append({
        'relative_path': workspace_relative_path(
            annotation_path, workspace_root),
        'sha256': sha256_file(annotation_path),
        'format': 'jsonl',
        'schema_version': SCHEMA_VERSION,
    })
    updated['capabilities']['annotations'] = True
    validate_manifest(updated)
    return updated


def build_legacy_manifest(
        bag_dir: Path,
        dataset_id: str,
        workspace_root: Path) -> Dict[str, Any]:
    bag_dir = Path(bag_dir)
    with (bag_dir / 'metadata.yaml').open('r', encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)['rosbag2_bagfile_information']
    topics = []
    for item in metadata.get('topics_with_message_count', []):
        topic = item['topic_metadata']
        topics.append({
            'name': topic['name'],
            'type': topic['type'],
            'count': int(item['message_count']),
        })
    payload = {
        'schema_version': SCHEMA_VERSION,
        'dataset_id': dataset_id,
        'split': 'legacy_replay_only',
        'bag': {
            'relative_path': workspace_relative_path(
                bag_dir, workspace_root),
            'sha256': sha256_tree(bag_dir),
            'storage_id': metadata['storage_identifier'],
            'start_time_ns': int(
                metadata['starting_time']['nanoseconds_since_epoch']),
            'duration_ns': int(metadata['duration']['nanoseconds']),
            'topics': sorted(topics, key=lambda value: value['name']),
        },
        'capabilities': {
            'camera': any(item['type'] == 'sensor_msgs/msg/Image' for item in topics),
            'lidar': any(item['type'] == 'sensor_msgs/msg/PointCloud2' for item in topics),
            'imu': any(item['type'] == 'sensor_msgs/msg/Imu' for item in topics),
            'local_pose': any(
                item['name'] == '/odom' and
                item['type'] == 'nav_msgs/msg/Odometry'
                for item in topics),
            'world_pose': False,
            'query_events': False,
            'annotations': False,
            'active_motion': False,
        },
        'calibration': {
            'camera_intrinsics_id': (
                'recorded_camera_info'
                if any(
                    item['type'] == 'sensor_msgs/msg/CameraInfo'
                    for item in topics)
                else 'unknown'),
            'camera_lidar_extrinsics_id': 'unverified_legacy_tf',
            'lidar_imu_extrinsics_id': 'unknown',
            'localization_config_id': 'none',
        },
        'environment': {
            'site_id': 'legacy_unknown',
            'session_id': dataset_id,
            'lighting': 'unknown',
            'surface': 'unknown',
            'weather': 'unknown',
        },
        'queries': [],
        'annotation_files': [],
        'objects': [],
        'trials': [],
        'provenance': {
            'created_at': datetime.now(
                timezone.utc).isoformat().replace('+00:00', 'Z'),
            'created_by': 'semantic_search_manifest',
            'notes': 'Legacy human-tracking bag; observation-only replay evidence.',
        },
    }
    validate_manifest(payload)
    return payload


def build_field_manifest(
        bag_dir: Path,
        dataset_id: str,
        workspace_root: Path,
        split: str,
        environment: Mapping[str, str],
        calibration: Mapping[str, str],
        world_pose: bool,
        active_motion: bool) -> Dict[str, Any]:
    if split not in SPLITS - {'legacy_replay_only'}:
        raise ManifestError('field split must be train/validation/test/extension')
    payload = build_legacy_manifest(
        bag_dir, dataset_id, workspace_root)
    payload['split'] = split
    payload['environment'] = dict(environment)
    payload['calibration'] = dict(calibration)
    forbidden_ids = {'unknown', 'none', 'unverified_legacy_tf'}
    if any(
            str(value).strip().lower() in forbidden_ids
            for value in payload['calibration'].values()):
        raise ManifestError(
            'field manifests require verified calibration IDs')
    topic_names = {
        topic['name'] for topic in payload['bag']['topics']}
    if world_pose and '/localization/odometry' not in topic_names:
        raise ManifestError(
            'world_pose requires /localization/odometry in the bag')
    payload['capabilities']['world_pose'] = bool(world_pose)
    payload['capabilities']['active_motion'] = bool(active_motion)
    payload['provenance'] = {
        'created_at': datetime.now(
            timezone.utc).isoformat().replace('+00:00', 'Z'),
        'created_by': 'semantic_search_manifest create-field',
        'notes': 'Field dataset bundle generated from closed rosbag metadata.',
    }
    validate_manifest(payload)
    return payload
~~~

- [ ] **Step 5: Implement the manifest CLI**

Create track_robot_semantic_search/manifest_cli.py:

~~~python
import argparse
import json
import sys
from pathlib import Path

from .manifest import (
    ManifestError,
    add_annotation_file,
    add_object,
    add_query_event,
    add_trial,
    build_field_manifest,
    build_legacy_manifest,
    load_manifest,
    write_json_atomic,
)


def parser():
    root = argparse.ArgumentParser()
    commands = root.add_subparsers(dest='command', required=True)
    validate = commands.add_parser('validate')
    validate.add_argument('manifest', type=Path)
    legacy = commands.add_parser('create-legacy')
    legacy.add_argument('bag_dir', type=Path)
    legacy.add_argument('output', type=Path)
    legacy.add_argument('--dataset-id', default='')
    legacy.add_argument('--workspace-root', type=Path, required=True)
    field = commands.add_parser('create-field')
    field.add_argument('bag_dir', type=Path)
    field.add_argument('output', type=Path)
    field.add_argument('--dataset-id', required=True)
    field.add_argument('--workspace-root', type=Path, required=True)
    field.add_argument(
        '--split',
        choices=('train', 'validation', 'test', 'extension'),
        required=True)
    field.add_argument('--site-id', required=True)
    field.add_argument('--session-id', required=True)
    field.add_argument('--lighting', required=True)
    field.add_argument('--surface', required=True)
    field.add_argument('--weather', required=True)
    field.add_argument('--camera-intrinsics-id', required=True)
    field.add_argument('--camera-lidar-extrinsics-id', required=True)
    field.add_argument('--lidar-imu-extrinsics-id', required=True)
    field.add_argument('--localization-config-id', required=True)
    field.add_argument('--world-pose', action='store_true')
    field.add_argument('--active-motion', action='store_true')
    query = commands.add_parser('add-query')
    query.add_argument('manifest', type=Path)
    query.add_argument('--query-id', type=int, required=True)
    query.add_argument('--stamp-ns', type=int, required=True)
    query.add_argument('--text', required=True)
    query.add_argument('--language', default='en')
    query.add_argument('--client-request-id', default='')
    object_parser = commands.add_parser('add-object')
    object_parser.add_argument('manifest', type=Path)
    object_parser.add_argument('--object-id', required=True)
    object_parser.add_argument('--physical-object-id', required=True)
    object_parser.add_argument('--label', action='append', required=True)
    object_parser.add_argument('--site-id', required=True)
    object_parser.add_argument('--acquisition-date', required=True)
    object_parser.add_argument(
        '--source', choices=('robot', 'public', 'synthetic'), required=True)
    object_parser.add_argument('--provenance', required=True)
    trial = commands.add_parser('add-trial')
    trial.add_argument('manifest', type=Path)
    trial.add_argument('--trial-id', required=True)
    trial.add_argument('--query-id', type=int, required=True)
    trial.add_argument('--target-object-id', default='')
    trial.add_argument('--positive', action='store_true')
    trial.add_argument('--start-stamp-ns', type=int, required=True)
    trial.add_argument('--end-stamp-ns', type=int, required=True)
    trial.add_argument('--nominal-distance-m', type=float, required=True)
    trial.add_argument(
        '--observation-stage',
        choices=('passive', 'pre_rotation', 'post_rotation'),
        required=True)
    trial.add_argument('--site-id', required=True)
    trial.add_argument('--session-id', required=True)
    annotations = commands.add_parser('add-annotations')
    annotations.add_argument('manifest', type=Path)
    annotations.add_argument('annotation_file', type=Path)
    annotations.add_argument('--workspace-root', type=Path, required=True)
    return root


def run(arguments) -> int:
    if arguments.command == 'validate':
        payload = load_manifest(arguments.manifest)
        print(json.dumps({
            'dataset_id': payload['dataset_id'],
            'schema_version': payload['schema_version'],
            'valid': True,
        }, sort_keys=True))
        return 0
    if arguments.command == 'create-legacy':
        dataset_id = arguments.dataset_id or arguments.bag_dir.name
        payload = build_legacy_manifest(
            arguments.bag_dir, dataset_id, arguments.workspace_root)
        write_json_atomic(arguments.output, payload)
        print(str(arguments.output))
        return 0
    if arguments.command == 'create-field':
        payload = build_field_manifest(
            bag_dir=arguments.bag_dir,
            dataset_id=arguments.dataset_id,
            workspace_root=arguments.workspace_root,
            split=arguments.split,
            environment={
                'site_id': arguments.site_id,
                'session_id': arguments.session_id,
                'lighting': arguments.lighting,
                'surface': arguments.surface,
                'weather': arguments.weather,
            },
            calibration={
                'camera_intrinsics_id': arguments.camera_intrinsics_id,
                'camera_lidar_extrinsics_id':
                arguments.camera_lidar_extrinsics_id,
                'lidar_imu_extrinsics_id':
                arguments.lidar_imu_extrinsics_id,
                'localization_config_id':
                arguments.localization_config_id,
            },
            world_pose=arguments.world_pose,
            active_motion=arguments.active_motion,
        )
        write_json_atomic(arguments.output, payload)
        print(str(arguments.output))
        return 0
    payload = load_manifest(arguments.manifest)
    if arguments.command == 'add-query':
        event = {
            'query_id': arguments.query_id,
            'stamp_ns': arguments.stamp_ns,
            'text': arguments.text,
            'language': arguments.language,
            'client_request_id': arguments.client_request_id,
        }
        updated = add_query_event(payload, event)
    elif arguments.command == 'add-object':
        updated = add_object(payload, {
            'object_id': arguments.object_id,
            'physical_object_id': arguments.physical_object_id,
            'labels': arguments.label,
            'site_id': arguments.site_id,
            'acquisition_date': arguments.acquisition_date,
            'source': arguments.source,
            'provenance': arguments.provenance,
        })
    elif arguments.command == 'add-trial':
        updated = add_trial(payload, {
            'trial_id': arguments.trial_id,
            'query_id': arguments.query_id,
            'target_object_id': arguments.target_object_id,
            'is_positive': arguments.positive,
            'start_stamp_ns': arguments.start_stamp_ns,
            'end_stamp_ns': arguments.end_stamp_ns,
            'nominal_distance_m': arguments.nominal_distance_m,
            'observation_stage': arguments.observation_stage,
            'site_id': arguments.site_id,
            'session_id': arguments.session_id,
        })
    else:
        updated = add_annotation_file(
            payload, arguments.annotation_file, arguments.workspace_root)
    write_json_atomic(arguments.manifest, updated)
    print(str(arguments.manifest))
    return 0


def main(argv=None):
    try:
        return run(parser().parse_args(argv))
    except (ManifestError, OSError, ValueError) as error:
        print('manifest error: {}'.format(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
~~~

- [ ] **Step 6: Add the three machine-readable schemas**

Create dataset_manifest.schema.json:

~~~json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Semantic Search Dataset Manifest",
  "type": "object",
  "additionalProperties": false,
  "required": [
    "schema_version",
    "dataset_id",
    "split",
    "bag",
    "capabilities",
    "calibration",
    "environment",
    "queries",
    "annotation_files",
    "objects",
    "trials",
    "provenance"
  ],
  "properties": {
    "schema_version": {"const": "1.0.0"},
    "dataset_id": {"type": "string", "minLength": 1},
    "split": {
      "enum": [
        "train",
        "validation",
        "test",
        "extension",
        "legacy_replay_only"
      ]
    },
    "bag": {
      "type": "object",
      "additionalProperties": false,
      "required": [
        "relative_path",
        "sha256",
        "storage_id",
        "start_time_ns",
        "duration_ns",
        "topics"
      ],
      "properties": {
        "relative_path": {
          "type": "string",
          "minLength": 1,
          "not": {"pattern": "^/"}
        },
        "sha256": {
          "type": "string",
          "pattern": "^[0-9a-f]{64}$"
        },
        "storage_id": {"type": "string", "minLength": 1},
        "start_time_ns": {"type": "integer", "minimum": 0},
        "duration_ns": {"type": "integer", "minimum": 0},
        "topics": {
          "type": "array",
          "items": {
            "type": "object",
            "additionalProperties": false,
            "required": ["name", "type", "count"],
            "properties": {
              "name": {"type": "string", "minLength": 1},
              "type": {"type": "string", "minLength": 1},
              "count": {"type": "integer", "minimum": 0}
            }
          }
        }
      }
    },
    "capabilities": {
      "type": "object",
      "additionalProperties": false,
      "required": [
        "camera",
        "lidar",
        "imu",
        "local_pose",
        "world_pose",
        "query_events",
        "annotations",
        "active_motion"
      ],
      "properties": {
        "camera": {"type": "boolean"},
        "lidar": {"type": "boolean"},
        "imu": {"type": "boolean"},
        "local_pose": {"type": "boolean"},
        "world_pose": {"type": "boolean"},
        "query_events": {"type": "boolean"},
        "annotations": {"type": "boolean"},
        "active_motion": {"type": "boolean"}
      }
    },
    "calibration": {
      "type": "object",
      "additionalProperties": false,
      "required": [
        "camera_intrinsics_id",
        "camera_lidar_extrinsics_id",
        "lidar_imu_extrinsics_id",
        "localization_config_id"
      ],
      "properties": {
        "camera_intrinsics_id": {"type": "string"},
        "camera_lidar_extrinsics_id": {"type": "string"},
        "lidar_imu_extrinsics_id": {"type": "string"},
        "localization_config_id": {"type": "string"}
      }
    },
    "environment": {
      "type": "object",
      "additionalProperties": false,
      "required": [
        "site_id",
        "session_id",
        "lighting",
        "surface",
        "weather"
      ],
      "properties": {
        "site_id": {"type": "string"},
        "session_id": {"type": "string"},
        "lighting": {"type": "string"},
        "surface": {"type": "string"},
        "weather": {"type": "string"}
      }
    },
    "queries": {
      "type": "array",
      "items": {
        "type": "object",
        "additionalProperties": false,
        "required": [
          "query_id",
          "stamp_ns",
          "text",
          "language",
          "client_request_id"
        ],
        "properties": {
          "query_id": {"type": "integer", "minimum": 1},
          "stamp_ns": {"type": "integer", "minimum": 0},
          "text": {"type": "string", "minLength": 1},
          "language": {"type": "string", "minLength": 1},
          "client_request_id": {"type": "string"}
        }
      }
    },
    "annotation_files": {
      "type": "array",
      "items": {
        "type": "object",
        "additionalProperties": false,
        "required": ["relative_path", "sha256", "format", "schema_version"],
        "properties": {
          "relative_path": {
            "type": "string",
            "minLength": 1,
            "not": {"pattern": "^/"}
          },
          "sha256": {
            "type": "string",
            "pattern": "^[0-9a-f]{64}$"
          },
          "format": {"const": "jsonl"},
          "schema_version": {"const": "1.0.0"}
        }
      }
    },
    "objects": {
      "type": "array",
      "items": {
        "type": "object",
        "additionalProperties": false,
        "required": [
          "object_id",
          "physical_object_id",
          "labels",
          "site_id",
          "acquisition_date",
          "source",
          "provenance"
        ],
        "properties": {
          "object_id": {"type": "string", "minLength": 1},
          "physical_object_id": {"type": "string", "minLength": 1},
          "labels": {
            "type": "array",
            "minItems": 1,
            "items": {"type": "string", "minLength": 1}
          },
          "site_id": {"type": "string", "minLength": 1},
          "acquisition_date": {
            "type": "string",
            "pattern": "^[0-9]{4}-[0-9]{2}-[0-9]{2}$"
          },
          "source": {"enum": ["robot", "public", "synthetic"]},
          "provenance": {"type": "string", "minLength": 1}
        }
      }
    },
    "trials": {
      "type": "array",
      "items": {
        "type": "object",
        "additionalProperties": false,
        "required": [
          "trial_id",
          "query_id",
          "target_object_id",
          "is_positive",
          "start_stamp_ns",
          "end_stamp_ns",
          "nominal_distance_m",
          "observation_stage",
          "site_id",
          "session_id"
        ],
        "properties": {
          "trial_id": {"type": "string", "minLength": 1},
          "query_id": {"type": "integer", "minimum": 1},
          "target_object_id": {"type": "string"},
          "is_positive": {"type": "boolean"},
          "start_stamp_ns": {"type": "integer", "minimum": 0},
          "end_stamp_ns": {"type": "integer", "minimum": 0},
          "nominal_distance_m": {"type": "number", "minimum": 0.0},
          "observation_stage": {
            "enum": ["passive", "pre_rotation", "post_rotation"]
          },
          "site_id": {"type": "string", "minLength": 1},
          "session_id": {"type": "string", "minLength": 1}
        }
      }
    },
    "provenance": {
      "type": "object",
      "additionalProperties": false,
      "required": ["created_at", "created_by", "notes"],
      "properties": {
        "created_at": {"type": "string", "minLength": 1},
        "created_by": {"type": "string", "minLength": 1},
        "notes": {"type": "string"}
      }
    }
  }
}
~~~

Create annotation.schema.json as newline-record schema with these exact required fields:

~~~json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Semantic Search Frame Annotation",
  "type": "object",
  "additionalProperties": false,
  "required": [
    "schema_version", "dataset_id", "trial_id", "stamp_ns", "query_id",
    "object_id", "visibility", "label_source", "confidence"
  ],
  "properties": {
    "schema_version": {"const": "1.0.0"},
    "dataset_id": {"type": "string", "minLength": 1},
    "trial_id": {"type": "string", "minLength": 1},
    "stamp_ns": {"type": "integer", "minimum": 0},
    "query_id": {"type": "integer", "minimum": 1},
    "object_id": {"type": "string", "minLength": 1},
    "bbox_xywh": {
      "type": ["array", "null"],
      "items": {"type": "number"},
      "minItems": 4,
      "maxItems": 4
    },
    "mask_path": {"type": ["string", "null"]},
    "position_base_m": {
      "type": ["array", "null"],
      "items": {"type": "number"},
      "minItems": 3,
      "maxItems": 3
    },
    "extent_m": {
      "type": ["array", "null"],
      "items": {"type": "number"},
      "minItems": 3,
      "maxItems": 3
    },
    "visibility": {
      "enum": ["visible", "partial", "occluded", "out_of_fov"]
    },
    "label_source": {
      "enum": ["human", "teacher", "synthetic", "temporal_pseudo"]
    },
    "confidence": {"type": "number", "minimum": 0.0, "maximum": 1.0}
  }
}
~~~

Create evaluation_report.schema.json:

~~~json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "Semantic Search Evaluation Report",
  "type": "object",
  "additionalProperties": false,
  "required": [
    "schema_version",
    "dataset_id",
    "manifest_sha256",
    "run",
    "artifacts",
    "coverage",
    "topic_metrics",
    "synchronization",
    "latency_metrics",
    "localization",
    "semantic_counts",
    "resources",
    "safety",
    "gates",
    "passed"
  ],
  "properties": {
    "schema_version": {"const": "1.0.0"},
    "dataset_id": {"type": "string", "minLength": 1},
    "manifest_sha256": {
      "type": "string",
      "pattern": "^[0-9a-f]{64}$"
    },
    "run": {
      "type": "object",
      "required": ["run_id", "phase", "replay_rate"],
      "properties": {
        "run_id": {"type": "string", "minLength": 1},
        "phase": {"const": "phase0"},
        "replay_rate": {"type": "number", "exclusiveMinimum": 0.0}
      },
      "additionalProperties": false
    },
    "artifacts": {
      "type": "object",
      "required": ["software_revision", "config_sha256", "model_exports"],
      "properties": {
        "software_revision": {"type": "string", "minLength": 1},
        "config_sha256": {
          "type": "string",
          "pattern": "^[0-9a-f]{64}$"
        },
        "model_exports": {
          "type": "array",
          "maxItems": 0
        }
      },
      "additionalProperties": false
    },
    "coverage": {
      "type": "object",
      "required": ["accuracy", "identity", "active_search"],
      "properties": {
        "accuracy": {"const": "not_applicable_phase0_no_model"},
        "identity": {"const": "not_applicable_phase0_no_tracker"},
        "active_search": {"const": "not_applicable_phase0_passive_only"}
      },
      "additionalProperties": false
    },
    "topic_metrics": {
      "type": "object",
      "additionalProperties": true
    },
    "synchronization": {
      "type": "object",
      "additionalProperties": true
    },
    "latency_metrics": {
      "type": "object",
      "additionalProperties": true
    },
    "localization": {
      "type": "object",
      "additionalProperties": true
    },
    "semantic_counts": {
      "type": "object",
      "additionalProperties": true
    },
    "resources": {
      "type": "object",
      "additionalProperties": true
    },
    "safety": {
      "type": "object",
      "additionalProperties": true
    },
    "gates": {
      "type": "object",
      "additionalProperties": {"type": "boolean"}
    },
    "passed": {"type": "boolean"}
  }
}
~~~

The dataset schema is documentation/tooling evidence; the Python validator remains authoritative on the robot because jsonschema is not installed and must not be added to the ROS environment.

- [ ] **Step 7: Run the manifest test and package smoke test**

Run:

~~~bash
cd /home/track-robot/track_robot_ws
PYTHONPATH=src/track_robot_semantic_search pytest -q \
  src/track_robot_semantic_search/test/test_manifest.py
bash -lc 'source /opt/ros/foxy/setup.bash && cd /home/track-robot/track_robot_ws && colcon build --packages-select track_robot_interfaces track_robot_semantic_search --symlink-install --event-handlers console_direct+'
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && ros2 run track_robot_semantic_search semantic_search_manifest --help'
~~~

Expected: manifest tests pass; both packages build; CLI help lists validate,
create-legacy, create-field, add-query, add-object, add-trial, and
add-annotations.

- [ ] **Step 8: Commit the manifest foundation**

~~~bash
git add src/track_robot_semantic_search
git commit -m "feat: add semantic search manifest foundation"
~~~

### Task 3: Implement Dual-Mode Localisation Health and Epoch Logic

**Files:**

- Create: src/track_robot_semantic_search/track_robot_semantic_search/localization_health.py
- Create: src/track_robot_semantic_search/track_robot_semantic_search/localization_health_node.py
- Create: src/track_robot_semantic_search/config/semantic_search_phase0.yaml
- Create: src/track_robot_semantic_search/test/test_localization_health.py

**Interfaces:**

- LocalizationSample carries timestamp, local-pose/IMU/TF health, optional world pose/covariance, and pose values.
- LocalizationHealthEvaluator.update(sample) -> LocalizationDecision.
- Modes are OBSERVATION_ONLY=0, LOCAL_SESSION=1, WORLD=2 and match TrackedSemanticObject constants.
- The ROS node publishes diagnostic_msgs/DiagnosticArray on /semantic_search/localization_diagnostics for operators/evaluation only. Future object memory imports the evaluator directly; consumers do not parse diagnostics as a control contract.

- [ ] **Step 1: Write failing state-machine tests**

Create test/test_localization_health.py:

~~~python
import math

from track_robot_semantic_search.localization_health import (
    LocalizationHealthEvaluator,
    LocalizationSample,
    MemoryMode,
)


def sample(stamp_ns, **overrides):
    values = {
        'stamp_ns': stamp_ns,
        'local_pose_fresh': True,
        'imu_fresh': True,
        'local_tf_available': True,
        'world_pose_fresh': False,
        'world_tf_available': False,
        'world_pose_stamp_ns': -1,
        'world_covariance_xy_m2': math.inf,
        'world_yaw_variance_rad2': math.inf,
        'world_x': math.nan,
        'world_y': math.nan,
        'world_yaw': math.nan,
    }
    values.update(overrides)
    return LocalizationSample(**values)


def world_sample(stamp_ns, x=0.0, yaw=0.0):
    return sample(
        stamp_ns,
        world_pose_fresh=True,
        world_tf_available=True,
        world_pose_stamp_ns=stamp_ns,
        world_covariance_xy_m2=0.04,
        world_yaw_variance_rad2=0.01,
        world_x=x,
        world_y=0.0,
        world_yaw=yaw,
    )


def evaluator(world_enabled=True):
    return LocalizationHealthEvaluator(
        world_enabled=world_enabled,
        world_stable_samples=3,
        maximum_world_xy_variance_m2=0.25,
        maximum_world_yaw_variance_rad2=0.12,
        maximum_world_jump_m=0.50,
        maximum_world_yaw_jump_rad=0.26,
    )


def test_missing_local_pose_is_observation_only():
    decision = evaluator().update(
        sample(1, local_pose_fresh=False))
    assert decision.mode == MemoryMode.OBSERVATION_ONLY
    assert decision.reason == 'local_pose_stale'


def test_healthy_local_pose_without_world_is_local_session():
    decision = evaluator().update(sample(1))
    assert decision.mode == MemoryMode.LOCAL_SESSION
    assert decision.epoch_id == 1


def test_world_requires_explicit_enable_and_three_stable_samples():
    disabled = evaluator(world_enabled=False)
    assert disabled.update(world_sample(1)).mode == MemoryMode.LOCAL_SESSION
    enabled = evaluator()
    assert enabled.update(world_sample(1)).reason == 'world_stabilizing'
    assert enabled.update(world_sample(1)).mode == MemoryMode.LOCAL_SESSION
    assert enabled.update(world_sample(2)).mode == MemoryMode.LOCAL_SESSION
    assert enabled.update(world_sample(3)).mode == MemoryMode.WORLD


def test_world_jump_closes_epoch_and_does_not_auto_associate():
    health = evaluator()
    for stamp in (1, 2, 3):
        decision = health.update(world_sample(stamp))
    assert decision.mode == MemoryMode.WORLD
    jumped = health.update(world_sample(4, x=1.0))
    assert jumped.mode == MemoryMode.LOCAL_SESSION
    assert jumped.epoch_changed is True
    assert jumped.epoch_id == 2
    assert jumped.reason == 'world_pose_jump'


def test_timestamp_rollback_forces_one_observation_only_sample():
    health = evaluator()
    assert health.update(sample(10)).mode == MemoryMode.LOCAL_SESSION
    rolled = health.update(sample(9))
    assert rolled.mode == MemoryMode.OBSERVATION_ONLY
    assert rolled.epoch_id == 2
    assert rolled.epoch_changed is True
    assert rolled.reason == 'timestamp_rollback'
~~~

- [ ] **Step 2: Run tests to verify the evaluator is absent**

Run:

~~~bash
cd /home/track-robot/track_robot_ws
PYTHONPATH=src/track_robot_semantic_search pytest -q \
  src/track_robot_semantic_search/test/test_localization_health.py
~~~

Expected: FAIL with ModuleNotFoundError for localization_health.

- [ ] **Step 3: Implement the pure localisation evaluator**

Create track_robot_semantic_search/localization_health.py:

~~~python
import math
from dataclasses import dataclass
from enum import IntEnum


class MemoryMode(IntEnum):
    OBSERVATION_ONLY = 0
    LOCAL_SESSION = 1
    WORLD = 2


@dataclass(frozen=True)
class LocalizationSample:
    stamp_ns: int
    local_pose_fresh: bool
    imu_fresh: bool
    local_tf_available: bool
    world_pose_fresh: bool
    world_tf_available: bool
    world_pose_stamp_ns: int
    world_covariance_xy_m2: float
    world_yaw_variance_rad2: float
    world_x: float
    world_y: float
    world_yaw: float


@dataclass(frozen=True)
class LocalizationDecision:
    mode: MemoryMode
    epoch_id: int
    epoch_changed: bool
    reason: str


def angle_difference(first: float, second: float) -> float:
    return math.atan2(
        math.sin(first - second), math.cos(first - second))


class LocalizationHealthEvaluator:
    def __init__(
            self,
            world_enabled: bool,
            world_stable_samples: int,
            maximum_world_xy_variance_m2: float,
            maximum_world_yaw_variance_rad2: float,
            maximum_world_jump_m: float,
            maximum_world_yaw_jump_rad: float):
        self.world_enabled = bool(world_enabled)
        self.world_stable_samples = max(1, int(world_stable_samples))
        self.maximum_world_xy_variance_m2 = float(
            maximum_world_xy_variance_m2)
        self.maximum_world_yaw_variance_rad2 = float(
            maximum_world_yaw_variance_rad2)
        self.maximum_world_jump_m = float(maximum_world_jump_m)
        self.maximum_world_yaw_jump_rad = float(
            maximum_world_yaw_jump_rad)
        self.epoch_id = 1
        self.previous_stamp_ns = None
        self.previous_world_pose = None
        self.previous_world_stamp_ns = None
        self.previous_mode = MemoryMode.OBSERVATION_ONLY
        self.world_stable_count = 0

    def _decision(
            self, mode: MemoryMode, reason: str,
            epoch_changed: bool = False) -> LocalizationDecision:
        self.previous_mode = mode
        return LocalizationDecision(
            mode=mode,
            epoch_id=self.epoch_id,
            epoch_changed=epoch_changed,
            reason=reason,
        )

    def _new_epoch(self) -> None:
        self.epoch_id += 1
        self.world_stable_count = 0
        self.previous_world_pose = None
        self.previous_world_stamp_ns = None

    def update(self, sample: LocalizationSample) -> LocalizationDecision:
        if sample.stamp_ns < 0:
            raise ValueError('stamp_ns must be non-negative')
        if (self.previous_stamp_ns is not None and
                sample.stamp_ns < self.previous_stamp_ns):
            self._new_epoch()
            self.previous_stamp_ns = sample.stamp_ns
            return self._decision(
                MemoryMode.OBSERVATION_ONLY,
                'timestamp_rollback',
                epoch_changed=True,
            )
        self.previous_stamp_ns = sample.stamp_ns

        local_checks = (
            (sample.local_pose_fresh, 'local_pose_stale'),
            (sample.imu_fresh, 'imu_stale'),
            (sample.local_tf_available, 'local_tf_unavailable'),
        )
        for healthy, reason in local_checks:
            if not healthy:
                changed = self.previous_mode != MemoryMode.OBSERVATION_ONLY
                if changed:
                    self._new_epoch()
                return self._decision(
                    MemoryMode.OBSERVATION_ONLY, reason, changed)

        world_values_finite = all(math.isfinite(value) for value in (
            sample.world_covariance_xy_m2,
            sample.world_yaw_variance_rad2,
            sample.world_x,
            sample.world_y,
            sample.world_yaw,
        ))
        world_healthy = (
            self.world_enabled and
            sample.world_pose_fresh and
            sample.world_tf_available and
            world_values_finite and
            sample.world_covariance_xy_m2 >= 0.0 and
            sample.world_yaw_variance_rad2 >= 0.0 and
            sample.world_covariance_xy_m2 <=
            self.maximum_world_xy_variance_m2 and
            sample.world_yaw_variance_rad2 <=
            self.maximum_world_yaw_variance_rad2
        )
        if not world_healthy:
            changed = self.previous_mode == MemoryMode.WORLD
            if changed:
                self._new_epoch()
            self.world_stable_count = 0
            self.previous_world_pose = None
            reason = (
                'world_disabled' if not self.world_enabled
                else 'world_pose_unhealthy')
            return self._decision(
                MemoryMode.LOCAL_SESSION, reason, changed)

        if sample.world_pose_stamp_ns < 0:
            return self._decision(
                MemoryMode.LOCAL_SESSION, 'world_stamp_invalid')
        if (self.previous_world_stamp_ns is not None and
                sample.world_pose_stamp_ns < self.previous_world_stamp_ns):
            self._new_epoch()
            return self._decision(
                MemoryMode.LOCAL_SESSION,
                'world_timestamp_rollback',
                epoch_changed=True,
            )
        new_world_sample = (
            sample.world_pose_stamp_ns != self.previous_world_stamp_ns)
        if not new_world_sample:
            if self.world_stable_count < self.world_stable_samples:
                return self._decision(
                    MemoryMode.LOCAL_SESSION, 'world_stabilizing')
            return self._decision(MemoryMode.WORLD, 'world_healthy')
        self.previous_world_stamp_ns = sample.world_pose_stamp_ns

        current_pose = (sample.world_x, sample.world_y, sample.world_yaw)
        if self.previous_world_pose is not None:
            distance = math.hypot(
                current_pose[0] - self.previous_world_pose[0],
                current_pose[1] - self.previous_world_pose[1],
            )
            yaw_step = abs(angle_difference(
                current_pose[2], self.previous_world_pose[2]))
            if (distance > self.maximum_world_jump_m or
                    yaw_step > self.maximum_world_yaw_jump_rad):
                self._new_epoch()
                self.previous_world_pose = current_pose
                return self._decision(
                    MemoryMode.LOCAL_SESSION,
                    'world_pose_jump',
                    epoch_changed=True,
                )

        self.previous_world_pose = current_pose
        self.world_stable_count += 1
        if self.world_stable_count < self.world_stable_samples:
            return self._decision(
                MemoryMode.LOCAL_SESSION, 'world_stabilizing')
        return self._decision(MemoryMode.WORLD, 'world_healthy')
~~~

- [ ] **Step 4: Add the Phase 0 configuration**

Create config/semantic_search_phase0.yaml:

~~~yaml
semantic_search_localization_health:
  ros__parameters:
    imu_topic: /imu/data_raw
    local_odometry_topic: /odom
    world_odometry_topic: /localization/odometry
    base_frame: base_link
    local_frame: odom
    world_frame: map
    diagnostics_topic: /semantic_search/localization_diagnostics
    publish_rate_hz: 10.0
    imu_timeout_sec: 0.25
    local_pose_timeout_sec: 0.30
    world_pose_timeout_sec: 0.30
    world_mode_enabled: false
    world_stable_samples: 3
    maximum_world_xy_variance_m2: 0.25
    maximum_world_yaw_variance_rad2: 0.12
    maximum_world_jump_m: 0.50
    maximum_world_yaw_jump_rad: 0.26

semantic_search_evaluator:
  ros__parameters:
    image_topic: /zed/zed_node/left/image_rect_color
    lidar_topic: /rslidar_points
    imu_topic: /imu/data_raw
    local_odometry_topic: /odom
    world_odometry_topic: /localization/odometry
    diagnostics_topic: /semantic_search/localization_diagnostics
    semantic_regions_topic: /semantic_search/regions
    observations_topic: /semantic_search/observations
    tracked_objects_topic: /semantic_search/tracked_objects
    motion_intent_topic: /semantic_search/motion_intent
    duration_sec: 30.0
    run_id: phase0
    replay_rate: 1.0
    software_revision: unversioned
    config_path: ''
    output_path: /tmp/semantic_search_phase0_report.json
    manifest_path: ''
    tegrastats_path: ''
~~~

World mode is intentionally false by default. The numeric world thresholds are provisional safety gates for tests; Phase 0 does not claim field localisation acceptance.

- [ ] **Step 5: Implement the thin diagnostic ROS node**

Create localization_health_node.py with this exact public structure:

~~~python
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener

from .localization_health import (
    LocalizationHealthEvaluator,
    LocalizationSample,
    MemoryMode,
)


def stamp_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


class LocalizationHealthNode(Node):
    def __init__(self):
        super().__init__('semantic_search_localization_health')
        self.imu_timeout = self.declare_parameter(
            'imu_timeout_sec', 0.25).value
        self.local_timeout = self.declare_parameter(
            'local_pose_timeout_sec', 0.30).value
        self.world_timeout = self.declare_parameter(
            'world_pose_timeout_sec', 0.30).value
        self.local_frame = self.declare_parameter(
            'local_frame', 'odom').value
        self.world_frame = self.declare_parameter(
            'world_frame', 'map').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_link').value
        self.evaluator = LocalizationHealthEvaluator(
            world_enabled=self.declare_parameter(
                'world_mode_enabled', False).value,
            world_stable_samples=self.declare_parameter(
                'world_stable_samples', 3).value,
            maximum_world_xy_variance_m2=self.declare_parameter(
                'maximum_world_xy_variance_m2', 0.25).value,
            maximum_world_yaw_variance_rad2=self.declare_parameter(
                'maximum_world_yaw_variance_rad2', 0.12).value,
            maximum_world_jump_m=self.declare_parameter(
                'maximum_world_jump_m', 0.50).value,
            maximum_world_yaw_jump_rad=self.declare_parameter(
                'maximum_world_yaw_jump_rad', 0.26).value,
        )
        self.imu = None
        self.local_pose = None
        self.world_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Imu,
            self.declare_parameter('imu_topic', '/imu/data_raw').value,
            self._imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.declare_parameter('local_odometry_topic', '/odom').value,
            self._local,
            10,
        )
        self.create_subscription(
            Odometry,
            self.declare_parameter(
                'world_odometry_topic', '/localization/odometry').value,
            self._world,
            10,
        )
        self.publisher = self.create_publisher(
            DiagnosticArray,
            self.declare_parameter(
                'diagnostics_topic',
                '/semantic_search/localization_diagnostics').value,
            10,
        )
        rate = max(1.0, float(self.declare_parameter(
            'publish_rate_hz', 10.0).value))
        self.create_timer(1.0 / rate, self._publish)

    def _imu(self, message):
        self.imu = message

    def _local(self, message):
        self.local_pose = message

    def _world(self, message):
        self.world_pose = message

    def _fresh(self, message, timeout):
        if message is None:
            return False
        age = self.get_clock().now().nanoseconds - stamp_ns(message.header.stamp)
        return 0 <= age <= int(timeout * 1000000000)

    def _world_values(self):
        if self.world_pose is None:
            return math.inf, math.inf, math.nan, math.nan, math.nan
        pose = self.world_pose.pose.pose
        covariance = self.world_pose.pose.covariance
        yaw = math.atan2(
            2.0 * (pose.orientation.w * pose.orientation.z +
                   pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (
                pose.orientation.y * pose.orientation.y +
                pose.orientation.z * pose.orientation.z),
        )
        return (
            max(float(covariance[0]), float(covariance[7])),
            float(covariance[35]),
            float(pose.position.x),
            float(pose.position.y),
            float(yaw),
        )

    def _publish(self):
        covariance, yaw_variance, x, y, yaw = self._world_values()
        local_tf = self.tf_buffer.can_transform(
            self.local_frame, self.base_frame, Time())
        world_tf = self.tf_buffer.can_transform(
            self.world_frame, self.base_frame, Time())
        now_ns = self.get_clock().now().nanoseconds
        decision = self.evaluator.update(LocalizationSample(
            stamp_ns=max(0, now_ns),
            local_pose_fresh=self._fresh(
                self.local_pose, self.local_timeout),
            imu_fresh=self._fresh(self.imu, self.imu_timeout),
            local_tf_available=local_tf,
            world_pose_fresh=self._fresh(
                self.world_pose, self.world_timeout),
            world_tf_available=world_tf,
            world_pose_stamp_ns=(
                stamp_ns(self.world_pose.header.stamp)
                if self.world_pose is not None else -1),
            world_covariance_xy_m2=covariance,
            world_yaw_variance_rad2=yaw_variance,
            world_x=x,
            world_y=y,
            world_yaw=yaw,
        ))
        status = DiagnosticStatus()
        status.name = 'semantic_search/localization'
        status.hardware_id = 'track_robot'
        status.message = decision.reason
        status.level = (
            DiagnosticStatus.OK if decision.mode == MemoryMode.WORLD
            else DiagnosticStatus.WARN
            if decision.mode == MemoryMode.LOCAL_SESSION
            else DiagnosticStatus.STALE)
        status.values = [
            KeyValue(key='memory_mode', value=decision.mode.name),
            KeyValue(key='epoch_id', value=str(decision.epoch_id)),
            KeyValue(
                key='epoch_changed',
                value=str(decision.epoch_changed).lower()),
            KeyValue(key='world_enabled', value=str(
                self.evaluator.world_enabled).lower()),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationHealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
~~~

The node deliberately validates frame IDs from Odometry rather than inventing a transform. A later memory implementation may add a verified TF source behind the pure evaluator contract.

- [ ] **Step 6: Run the localisation tests**

Run:

~~~bash
cd /home/track-robot/track_robot_ws
PYTHONPATH=src/track_robot_semantic_search pytest -q \
  src/track_robot_semantic_search/test/test_localization_health.py
~~~

Expected: 5 tests pass, including rollback and world-jump epoch changes.

- [ ] **Step 7: Commit localisation health**

~~~bash
git add \
  src/track_robot_semantic_search/config/semantic_search_phase0.yaml \
  src/track_robot_semantic_search/track_robot_semantic_search/localization_health.py \
  src/track_robot_semantic_search/track_robot_semantic_search/localization_health_node.py \
  src/track_robot_semantic_search/test/test_localization_health.py
git commit -m "feat: add semantic localization health modes"
~~~

### Task 4: Build the Model-Independent Replay and Resource Evaluator

**Files:**

- Create: src/track_robot_semantic_search/track_robot_semantic_search/evaluation.py
- Create: src/track_robot_semantic_search/track_robot_semantic_search/evaluator_node.py
- Create: src/track_robot_semantic_search/track_robot_semantic_search/compare_reports.py
- Create: src/track_robot_semantic_search/test/test_evaluation.py

**Interfaces:**

- TopicSeries.observe(source_stamp_ns, receive_stamp_ns) and report().
- EvaluationAccumulator records sensor stamps, nearest image/LiDAR offsets, evaluator callback latency, localisation modes, semantic counts, intent safety, CPU/RSS, and optional tegrastats.
- EvaluationAccumulator.finalize() returns the evaluation_report.schema.json shape and computes capability-aware gates.
- semantic_search_evaluator writes one report atomically after duration_sec; it publishes no ROS topic.
- semantic_search_compare_reports --manifest MANIFEST REPORT_05 REPORT_10 REPORT_20 validates exactly three formal reports, recomputes their gates, and returns nonzero for any contract, provenance, manifest, gate, or forward-motion failure.

- [ ] **Step 1: Write failing evaluator tests**

Create test/test_evaluation.py:

~~~python
import json

from track_robot_semantic_search.compare_reports import compare
from track_robot_semantic_search.evaluation import (
    EvaluationAccumulator,
    TopicSeries,
    parse_tegrastats_line,
    percentile,
)


def manifest(local_pose=False, world_pose=False):
    return {
        'schema_version': '1.0.0',
        'dataset_id': 'evaluation_test',
        'capabilities': {
            'camera': True,
            'lidar': True,
            'imu': local_pose,
            'local_pose': local_pose,
            'world_pose': world_pose,
            'query_events': False,
            'annotations': False,
            'active_motion': False,
        },
    }


def make_metrics(manifest_sha256, run_id, replay_rate=1.0):
    return EvaluationAccumulator(
        manifest=manifest(),
        manifest_sha256=manifest_sha256,
        run_id=run_id,
        software_revision='unit-test-revision',
        config_sha256='c' * 64,
        replay_rate=replay_rate,
    )


def test_percentile_and_source_rate_are_deterministic():
    assert percentile([1.0, 2.0, 3.0, 4.0], 0.5) == 2.5
    series = TopicSeries()
    for index in range(4):
        series.observe(index * 100000000, index * 200000000)
    report = series.report()
    assert report['count'] == 4
    assert report['source_rate_hz'] == 10.0
    assert report['receive_rate_hz'] == 5.0
    assert len(report['source_sequence_sha256']) == 64


def test_capability_aware_legacy_replay_passes_observation_only():
    metrics = make_metrics('a' * 64, 'rate-1.0')
    for index in range(10):
        image = index * 100000000
        cloud = image + 20000000
        metrics.observe_topic('image', image, image)
        metrics.observe_topic('lidar', cloud, cloud)
        metrics.observe_pair_offset(abs(cloud - image))
        metrics.observe_localization('OBSERVATION_ONLY', 1)
        metrics.observe_latency('image_callback', 0.001 + index * 0.0001)
    report = metrics.finalize()
    assert report['gates']['required_topics_present'] is True
    assert report['gates']['sync_p95_at_most_80_ms'] is True
    assert report['gates']['manifest_localization_mode_respected'] is True
    assert report['latency_metrics']['image_callback']['count'] == 10
    assert report['latency_metrics']['image_callback']['p95_sec'] < 0.002
    assert report['run']['replay_rate'] == 1.0
    assert report['artifacts']['model_exports'] == []
    assert report['passed'] is True


def test_forward_intent_is_a_hard_failure():
    metrics = make_metrics('b' * 64, 'unsafe')
    metrics.observe_topic('image', 1, 1)
    metrics.observe_topic('lidar', 1, 1)
    metrics.observe_pair_offset(0)
    metrics.observe_localization('OBSERVATION_ONLY', 1)
    metrics.observe_motion_intent(forward_permitted=True)
    report = metrics.finalize()
    assert report['safety']['forward_permission_violations'] == 1
    assert report['passed'] is False


def test_pose_capabilities_require_the_corresponding_healthy_mode():
    payload = manifest(local_pose=True, world_pose=True)
    metrics = EvaluationAccumulator(
        payload, 'd' * 64, 'world', 'unit-test-revision', 'c' * 64, 1.0)
    for name in ('image', 'lidar', 'imu', 'local_pose', 'world_pose'):
        metrics.observe_topic(name, 1, 1)
    metrics.observe_pair_offset(0)
    metrics.observe_localization('WORLD', 1)
    assert metrics.finalize()['passed'] is True

    degraded = EvaluationAccumulator(
        payload, 'd' * 64, 'degraded', 'unit-test-revision', 'c' * 64, 1.0)
    for name in ('image', 'lidar', 'imu', 'local_pose', 'world_pose'):
        degraded.observe_topic(name, 1, 1)
    degraded.observe_pair_offset(0)
    degraded.observe_localization('OBSERVATION_ONLY', 1)
    assert degraded.finalize()['passed'] is False


def test_tegrastats_parser_extracts_report_only_resources():
    values = parse_tegrastats_line(
        'RAM 4000/31919MB CPU [10%@1200,off] GR3D_FREQ 21% '
        'CPU@48.5C GPU@46.0C VDD_IN 12200mW/11800mW')
    assert values['ram_used_mb'] == 4000.0
    assert values['gpu_utilization_percent'] == 21.0
    assert values['cpu_temperature_c'] == 48.5
    assert values['input_power_mw'] == 12200.0


def test_compare_requires_same_dataset_and_manifest(tmp_path):
    first = make_metrics('a' * 64, 'first', replay_rate=0.5)
    second = make_metrics('b' * 64, 'second', replay_rate=2.0)
    for metrics in (first, second):
        metrics.observe_topic('image', 1, 1)
        metrics.observe_topic('lidar', 1, 1)
        metrics.observe_pair_offset(0)
        metrics.observe_localization('OBSERVATION_ONLY', 1)
    first_path = tmp_path / 'first.json'
    second_path = tmp_path / 'second.json'
    first_path.write_text(json.dumps(first.finalize()), encoding='utf-8')
    second_path.write_text(json.dumps(second.finalize()), encoding='utf-8')
    result = compare([first_path, second_path])
    assert result['passed'] is False
    assert 'different manifest_sha256 values' in result['failures']
~~~

- [ ] **Step 2: Run tests to verify evaluator code is absent**

Run:

~~~bash
cd /home/track-robot/track_robot_ws
PYTHONPATH=src/track_robot_semantic_search pytest -q \
  src/track_robot_semantic_search/test/test_evaluation.py
~~~

Expected: FAIL with ModuleNotFoundError for evaluation.

- [ ] **Step 3: Implement deterministic metrics and gates**

Create evaluation.py:

~~~python
import hashlib
import json
import math
import re
from collections import Counter
from pathlib import Path


REPORT_SCHEMA_VERSION = '1.0.0'


def percentile(values, quantile):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    position = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


class TopicSeries:
    def __init__(self):
        self.source_stamps = []
        self.receive_stamps = []

    def observe(self, source_stamp_ns, receive_stamp_ns):
        self.source_stamps.append(int(source_stamp_ns))
        self.receive_stamps.append(int(receive_stamp_ns))

    @staticmethod
    def _rate(stamps):
        if len(stamps) < 2:
            return 0.0
        duration = (max(stamps) - min(stamps)) / 1000000000.0
        return 0.0 if duration <= 0.0 else (len(stamps) - 1) / duration

    def report(self):
        serialized = json.dumps(
            self.source_stamps, separators=(',', ':')).encode('utf-8')
        return {
            'count': len(self.source_stamps),
            'source_rate_hz': round(self._rate(self.source_stamps), 3),
            'receive_rate_hz': round(self._rate(self.receive_stamps), 3),
            'source_sequence_sha256': hashlib.sha256(serialized).hexdigest(),
        }


def parse_tegrastats_line(line):
    patterns = {
        'ram_used_mb': r'RAM\s+([0-9.]+)/',
        'gpu_utilization_percent': r'GR3D_FREQ\s+([0-9.]+)%',
        'cpu_temperature_c': r'CPU@([0-9.]+)C',
        'gpu_temperature_c': r'GPU@([0-9.]+)C',
        'input_power_mw': r'VDD_IN\s+([0-9.]+)mW',
    }
    result = {}
    for name, pattern in patterns.items():
        match = re.search(pattern, line)
        if match:
            result[name] = float(match.group(1))
    return result


def summarize_tegrastats(path):
    path = Path(path)
    if not path.is_file():
        return {}
    fields = {}
    with path.open('r', encoding='utf-8', errors='replace') as stream:
        for line in stream:
            for name, value in parse_tegrastats_line(line).items():
                fields.setdefault(name, []).append(value)
    return {
        name: {
            'mean': round(sum(values) / len(values), 3),
            'p95': round(percentile(values, 0.95), 3),
            'maximum': round(max(values), 3),
        }
        for name, values in sorted(fields.items())
        if values
    }


class EvaluationAccumulator:
    def __init__(
            self, manifest, manifest_sha256, run_id, software_revision,
            config_sha256, replay_rate,
            tegrastats_path=''):
        self.manifest = manifest
        self.manifest_sha256 = manifest_sha256
        self.run_id = run_id
        self.software_revision = str(software_revision)
        self.config_sha256 = str(config_sha256)
        self.replay_rate = float(replay_rate)
        if not self.software_revision:
            raise ValueError('software_revision must be non-empty')
        if not re.fullmatch(r'[0-9a-f]{64}', self.config_sha256):
            raise ValueError('config_sha256 must be lowercase SHA-256')
        if not math.isfinite(self.replay_rate) or self.replay_rate <= 0.0:
            raise ValueError('replay_rate must be positive')
        self.tegrastats_path = tegrastats_path
        self.topics = {}
        self.pair_offsets_sec = []
        self.localization_modes = Counter()
        self.localization_epochs = []
        self.forward_permission_violations = 0
        self.motion_intent_count = 0
        self.semantic_region_count = 0
        self.observation_count = 0
        self.tracked_object_count = 0
        self.latencies_sec = {}
        self.process_cpu_percent = []
        self.process_rss_mb = []
        self.system_cpu_percent = []
        self.system_ram_used_mb = []

    def observe_topic(self, name, source_stamp_ns, receive_stamp_ns):
        self.topics.setdefault(name, TopicSeries()).observe(
            source_stamp_ns, receive_stamp_ns)

    def observe_pair_offset(self, offset_ns):
        self.pair_offsets_sec.append(abs(int(offset_ns)) / 1000000000.0)

    def observe_localization(self, mode, epoch_id):
        self.localization_modes[str(mode)] += 1
        self.localization_epochs.append(int(epoch_id))

    def observe_motion_intent(self, forward_permitted):
        self.motion_intent_count += 1
        if forward_permitted:
            self.forward_permission_violations += 1

    def observe_latency(self, name, duration_sec):
        duration_sec = float(duration_sec)
        if math.isfinite(duration_sec) and duration_sec >= 0.0:
            self.latencies_sec.setdefault(str(name), []).append(duration_sec)

    def observe_resource(
            self, process_cpu_percent, process_rss_mb,
            system_cpu_percent, system_ram_used_mb):
        samples = (
            (self.process_cpu_percent, process_cpu_percent),
            (self.process_rss_mb, process_rss_mb),
            (self.system_cpu_percent, system_cpu_percent),
            (self.system_ram_used_mb, system_ram_used_mb),
        )
        for output, value in samples:
            if math.isfinite(value):
                output.append(float(value))

    def _resource_series(self, values):
        if not values:
            return {}
        return {
            'mean': round(sum(values) / len(values), 3),
            'p95': round(percentile(values, 0.95), 3),
            'maximum': round(max(values), 3),
        }

    def _latency_series(self, values):
        return {
            'count': len(values),
            'mean_sec': round(sum(values) / len(values), 6),
            'p95_sec': round(percentile(values, 0.95), 6),
            'maximum_sec': round(max(values), 6),
        }

    def finalize(self):
        capabilities = self.manifest['capabilities']
        topic_reports = {
            name: series.report()
            for name, series in sorted(self.topics.items())
        }
        required = []
        if capabilities['camera']:
            required.append('image')
        if capabilities['lidar']:
            required.append('lidar')
        if capabilities['imu']:
            required.append('imu')
        if capabilities['local_pose']:
            required.append('local_pose')
        if capabilities['world_pose']:
            required.append('world_pose')
        required_topics_present = all(
            topic_reports.get(name, {}).get('count', 0) > 0
            for name in required)
        sync_p95 = percentile(self.pair_offsets_sec, 0.95)
        sync_gate = (
            True if not (capabilities['camera'] and capabilities['lidar'])
            else sync_p95 is not None and sync_p95 <= 0.08)
        modes = set(self.localization_modes)
        if not (capabilities['local_pose'] and capabilities['imu']):
            mode_gate = modes <= {'OBSERVATION_ONLY'} and bool(modes)
        elif not capabilities['world_pose']:
            mode_gate = (
                'LOCAL_SESSION' in modes and 'WORLD' not in modes)
        else:
            mode_gate = 'WORLD' in modes
        gates = {
            'required_topics_present': required_topics_present,
            'sync_p95_at_most_80_ms': sync_gate,
            'manifest_localization_mode_respected': mode_gate,
            'no_forward_permission': (
                self.forward_permission_violations == 0),
        }
        return {
            'schema_version': REPORT_SCHEMA_VERSION,
            'dataset_id': self.manifest['dataset_id'],
            'manifest_sha256': self.manifest_sha256,
            'run': {
                'run_id': self.run_id,
                'phase': 'phase0',
                'replay_rate': self.replay_rate,
            },
            'artifacts': {
                'software_revision': self.software_revision,
                'config_sha256': self.config_sha256,
                'model_exports': [],
            },
            'coverage': {
                'accuracy': 'not_applicable_phase0_no_model',
                'identity': 'not_applicable_phase0_no_tracker',
                'active_search': 'not_applicable_phase0_passive_only',
            },
            'topic_metrics': topic_reports,
            'synchronization': {
                'pair_count': len(self.pair_offsets_sec),
                'p50_sec': percentile(self.pair_offsets_sec, 0.50),
                'p95_sec': sync_p95,
                'maximum_sec': (
                    max(self.pair_offsets_sec)
                    if self.pair_offsets_sec else None),
            },
            'latency_metrics': {
                name: self._latency_series(values)
                for name, values in sorted(self.latencies_sec.items())
            },
            'localization': {
                'mode_counts': dict(sorted(self.localization_modes.items())),
                'epoch_ids': sorted(set(self.localization_epochs)),
            },
            'semantic_counts': {
                'regions': self.semantic_region_count,
                'observations': self.observation_count,
                'tracked_objects': self.tracked_object_count,
            },
            'resources': {
                'evaluator_cpu_percent': self._resource_series(
                    self.process_cpu_percent),
                'evaluator_rss_mb': self._resource_series(
                    self.process_rss_mb),
                'system_cpu_percent': self._resource_series(
                    self.system_cpu_percent),
                'system_ram_used_mb': self._resource_series(
                    self.system_ram_used_mb),
                'tegrastats': summarize_tegrastats(
                    self.tegrastats_path),
            },
            'safety': {
                'motion_intent_count': self.motion_intent_count,
                'forward_permission_violations':
                self.forward_permission_violations,
            },
            'gates': gates,
            'passed': all(gates.values()),
        }
~~~

- [ ] **Step 4: Implement the evaluator ROS adapter**

Create evaluator_node.py:

~~~python
import time
from collections import deque
from pathlib import Path

import psutil
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2
from track_robot_interfaces.msg import (
    ObjectObservation3DArray,
    SearchMotionIntent,
    SemanticRegionArray,
    TrackedSemanticObjectArray,
)

from .evaluation import EvaluationAccumulator
from .manifest import load_manifest, sha256_file, write_json_atomic


def stamp_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


class SemanticSearchEvaluatorNode(Node):
    def __init__(self):
        super().__init__('semantic_search_evaluator')
        manifest_path = str(self.declare_parameter(
            'manifest_path', '').value)
        self.output_path = Path(str(self.declare_parameter(
            'output_path',
            '/tmp/semantic_search_phase0_report.json').value))
        self.duration_sec = float(self.declare_parameter(
            'duration_sec', 30.0).value)
        run_id = str(self.declare_parameter(
            'run_id', 'phase0').value)
        replay_rate = float(self.declare_parameter(
            'replay_rate', 1.0).value)
        software_revision = str(self.declare_parameter(
            'software_revision', 'unversioned').value)
        config_path = str(self.declare_parameter(
            'config_path', '').value)
        tegrastats_path = str(self.declare_parameter(
            'tegrastats_path', '').value)
        if not manifest_path:
            raise ValueError('manifest_path is required')
        if not config_path:
            raise ValueError('config_path is required')
        if self.duration_sec <= 0.0:
            raise ValueError('duration_sec must be positive')
        manifest = load_manifest(Path(manifest_path))
        self.metrics = EvaluationAccumulator(
            manifest=manifest,
            manifest_sha256=sha256_file(manifest_path),
            run_id=run_id,
            software_revision=software_revision,
            config_sha256=sha256_file(config_path),
            replay_rate=replay_rate,
            tegrastats_path=tegrastats_path,
        )
        self.image_stamps = deque(maxlen=256)
        self.cloud_stamps = deque(maxlen=256)
        self.started_ros_ns = None
        self.finished = False
        self.process = psutil.Process()
        self.process.cpu_percent(interval=None)
        psutil.cpu_percent(interval=None)

        self.create_subscription(
            Image,
            self.declare_parameter(
                'image_topic',
                '/zed/zed_node/left/image_rect_color').value,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            self.declare_parameter(
                'lidar_topic', '/rslidar_points').value,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            self.declare_parameter(
                'imu_topic', '/imu/data_raw').value,
            lambda message: self._observe('imu', message.header.stamp),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.declare_parameter(
                'local_odometry_topic', '/odom').value,
            lambda message: self._observe(
                'local_pose', message.header.stamp),
            10,
        )
        self.create_subscription(
            Odometry,
            self.declare_parameter(
                'world_odometry_topic',
                '/localization/odometry').value,
            lambda message: self._observe(
                'world_pose', message.header.stamp),
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            self.declare_parameter(
                'diagnostics_topic',
                '/semantic_search/localization_diagnostics').value,
            self.diagnostic_callback,
            10,
        )
        self.create_subscription(
            SemanticRegionArray,
            self.declare_parameter(
                'semantic_regions_topic',
                '/semantic_search/regions').value,
            self.region_callback,
            10,
        )
        self.create_subscription(
            ObjectObservation3DArray,
            self.declare_parameter(
                'observations_topic',
                '/semantic_search/observations').value,
            self.observation_callback,
            10,
        )
        self.create_subscription(
            TrackedSemanticObjectArray,
            self.declare_parameter(
                'tracked_objects_topic',
                '/semantic_search/tracked_objects').value,
            self.tracked_callback,
            10,
        )
        self.create_subscription(
            SearchMotionIntent,
            self.declare_parameter(
                'motion_intent_topic',
                '/semantic_search/motion_intent').value,
            self.intent_callback,
            10,
        )
        self.create_timer(1.0, self.resource_callback)
        self.create_timer(0.1, self.finish_callback)

    def _observe(self, name, stamp):
        source_ns = stamp_ns(stamp)
        self.metrics.observe_topic(
            name, source_ns, time.monotonic_ns())
        if self.started_ros_ns is None:
            self.started_ros_ns = self.get_clock().now().nanoseconds

    def _pair_latest(self, stamp, candidates):
        if not candidates:
            return
        nearest = min(candidates, key=lambda value: abs(value - stamp))
        self.metrics.observe_pair_offset(abs(nearest - stamp))

    def image_callback(self, message):
        started_ns = time.perf_counter_ns()
        try:
            value = stamp_ns(message.header.stamp)
            self._observe('image', message.header.stamp)
            self.image_stamps.append(value)
            self._pair_latest(value, self.cloud_stamps)
        finally:
            self.metrics.observe_latency(
                'image_callback',
                (time.perf_counter_ns() - started_ns) / 1000000000.0,
            )

    def cloud_callback(self, message):
        started_ns = time.perf_counter_ns()
        try:
            value = stamp_ns(message.header.stamp)
            self._observe('lidar', message.header.stamp)
            self.cloud_stamps.append(value)
            self._pair_latest(value, self.image_stamps)
        finally:
            self.metrics.observe_latency(
                'lidar_callback',
                (time.perf_counter_ns() - started_ns) / 1000000000.0,
            )

    def diagnostic_callback(self, message):
        for status in message.status:
            if status.name != 'semantic_search/localization':
                continue
            values = {item.key: item.value for item in status.values}
            self.metrics.observe_localization(
                values.get('memory_mode', 'UNKNOWN'),
                int(values.get('epoch_id', '0')),
            )

    def region_callback(self, message):
        self._observe('semantic_regions', message.header.stamp)
        self.metrics.semantic_region_count += len(message.regions)

    def observation_callback(self, message):
        self._observe('observations', message.header.stamp)
        self.metrics.observation_count += len(message.observations)

    def tracked_callback(self, message):
        self._observe('tracked_objects', message.header.stamp)
        self.metrics.tracked_object_count += len(message.objects)

    def intent_callback(self, message):
        self.metrics.observe_motion_intent(
            message.forward_permitted)

    def resource_callback(self):
        virtual_memory = psutil.virtual_memory()
        self.metrics.observe_resource(
            self.process.cpu_percent(interval=None),
            self.process.memory_info().rss / (1024.0 * 1024.0),
            psutil.cpu_percent(interval=None),
            virtual_memory.used / (1024.0 * 1024.0),
        )

    def finish_callback(self):
        if self.finished or self.started_ros_ns is None:
            return
        elapsed_ns = (
            self.get_clock().now().nanoseconds - self.started_ros_ns)
        if elapsed_ns < int(self.duration_sec * 1000000000):
            return
        write_json_atomic(self.output_path, self.metrics.finalize())
        self.finished = True
        self.get_logger().info(
            'wrote {}'.format(self.output_path))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSearchEvaluatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
~~~

The goal duration begins on the first sensor callback in ROS time. This prevents a simulated-clock jump from zero to the first bag timestamp from immediately ending the run. The node has no publisher.

- [ ] **Step 5: Implement report comparison**

Create compare_reports.py:

~~~python
import argparse
import json
import sys
from pathlib import Path


def compare(paths):
    reports = []
    for path in paths:
        with Path(path).open('r', encoding='utf-8') as stream:
            reports.append((str(path), json.load(stream)))
    dataset_ids = {report['dataset_id'] for _, report in reports}
    manifest_hashes = {
        report['manifest_sha256'] for _, report in reports
    }
    config_hashes = {
        report['artifacts']['config_sha256'] for _, report in reports
    }
    failures = []
    if len(dataset_ids) != 1:
        failures.append('reports use different dataset_id values')
    if len(manifest_hashes) != 1:
        failures.append('different manifest_sha256 values')
    if len(config_hashes) != 1:
        failures.append('different config_sha256 values')
    for path, report in reports:
        if not report.get('passed', False):
            failures.append('{}: report gates failed'.format(path))
        if report.get('safety', {}).get(
                'forward_permission_violations', 0) != 0:
            failures.append('{}: forward permission violation'.format(path))
    return {
        'reports': [path for path, _ in reports],
        'dataset_id': (
            next(iter(dataset_ids)) if len(dataset_ids) == 1 else None),
        'replay_rates': {
            path: report['run']['replay_rate']
            for path, report in reports
        },
        'source_sequence_hashes': {
            path: {
                name: values.get('source_sequence_sha256')
                for name, values in report.get(
                    'topic_metrics', {}).items()
            }
            for path, report in reports
        },
        'failures': failures,
        'passed': not failures,
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('reports', nargs='+', type=Path)
    arguments = parser.parse_args(argv)
    result = compare(arguments.reports)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
~~~

Sequence hashes are reported but do not hard-fail a replay-rate comparison because DDS/latest-frame dropping can differ at 2× without changing the capability gates.

- [ ] **Step 6: Run evaluator tests**

Run:

~~~bash
cd /home/track-robot/track_robot_ws
PYTHONPATH=src/track_robot_semantic_search pytest -q \
  src/track_robot_semantic_search/test/test_evaluation.py
~~~

Expected: 6 tests pass; pose-capability degradation and unsafe forward
permission make passed false, and comparison rejects a different manifest
checksum.

- [ ] **Step 7: Commit replay evaluation**

~~~bash
git add \
  src/track_robot_semantic_search/track_robot_semantic_search/evaluation.py \
  src/track_robot_semantic_search/track_robot_semantic_search/evaluator_node.py \
  src/track_robot_semantic_search/track_robot_semantic_search/compare_reports.py \
  src/track_robot_semantic_search/test/test_evaluation.py
git commit -m "feat: add semantic replay evaluator"
~~~

### Task 5: Add the Passive, No-Motion Launch Contract

**Files:**

- Create: src/track_robot_semantic_search/launch/semantic_search_phase0.launch.py
- Create: src/track_robot_semantic_search/test/test_launch_contract.py

**Interfaces:**

- Always starts semantic_search_localization_health.
- Starts semantic_search_evaluator only when start_evaluator:=true.
- Accepts use_sim_time, config_file, manifest_path, output_path,
  tegrastats_path, duration_sec, run_id, replay_rate, software_revision, and
  start_evaluator.
- Starts no model, action server, decision, controller, planner, safety, base, or motion bridge node.

- [ ] **Step 1: Write the failing launch contract test**

Create test/test_launch_contract.py:

~~~python
import ast
from pathlib import Path


LAUNCH = (
    Path(__file__).resolve().parents[1] /
    'launch' /
    'semantic_search_phase0.launch.py'
)


def string_keyword(call, name):
    for keyword in call.keywords:
        if keyword.arg == name and isinstance(keyword.value, ast.Constant):
            return keyword.value.value
    return None


def test_phase0_launch_contains_only_diagnostic_and_evaluator_nodes():
    source = LAUNCH.read_text(encoding='utf-8')
    tree = ast.parse(source)
    node_calls = [
        item for item in ast.walk(tree)
        if isinstance(item, ast.Call) and
        isinstance(item.func, ast.Name) and item.func.id == 'Node'
    ]
    executables = {
        string_keyword(call, 'executable') for call in node_calls
    }
    assert executables == {
        'semantic_search_localization_health',
        'semantic_search_evaluator',
    }
    for forbidden in (
            'cmd_vel', 'SearchMotionIntent', 'FollowDecision',
            'controller', 'planner', 'motion_bridge'):
        assert forbidden not in source


def test_evaluator_is_conditionally_started():
    source = LAUNCH.read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('start_evaluator', default_value='false')" in source
    assert "condition=IfCondition(start_evaluator)" in source
~~~

- [ ] **Step 2: Run test to verify launch is absent**

Run:

~~~bash
cd /home/track-robot/track_robot_ws
pytest -q src/track_robot_semantic_search/test/test_launch_contract.py
~~~

Expected: FAIL with FileNotFoundError for semantic_search_phase0.launch.py.

- [ ] **Step 3: Implement the exact passive launch**

Create launch/semantic_search_phase0.launch.py:

~~~python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_evaluator = LaunchConfiguration('start_evaluator')
    config_file = LaunchConfiguration('config_file')
    manifest_path = LaunchConfiguration('manifest_path')
    output_path = LaunchConfiguration('output_path')
    tegrastats_path = LaunchConfiguration('tegrastats_path')
    duration_sec = LaunchConfiguration('duration_sec')
    run_id = LaunchConfiguration('run_id')
    replay_rate = LaunchConfiguration('replay_rate')
    software_revision = LaunchConfiguration('software_revision')

    localization = Node(
        package='track_robot_semantic_search',
        executable='semantic_search_localization_health',
        name='semantic_search_localization_health',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time},
        ],
    )
    evaluator = Node(
        package='track_robot_semantic_search',
        executable='semantic_search_evaluator',
        name='semantic_search_evaluator',
        output='screen',
        condition=IfCondition(start_evaluator),
        parameters=[
            config_file,
            {
                'use_sim_time': use_sim_time,
                'manifest_path': manifest_path,
                'output_path': output_path,
                'tegrastats_path': tegrastats_path,
                'duration_sec': duration_sec,
                'run_id': run_id,
                'replay_rate': replay_rate,
                'software_revision': software_revision,
                'config_path': config_file,
            },
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_evaluator', default_value='false'),
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('track_robot_semantic_search'),
                'config',
                'semantic_search_phase0.yaml',
            ])),
        DeclareLaunchArgument('manifest_path', default_value=''),
        DeclareLaunchArgument(
            'output_path',
            default_value='/tmp/semantic_search_phase0_report.json'),
        DeclareLaunchArgument('tegrastats_path', default_value=''),
        DeclareLaunchArgument('duration_sec', default_value='30.0'),
        DeclareLaunchArgument('run_id', default_value='phase0'),
        DeclareLaunchArgument('replay_rate', default_value='1.0'),
        DeclareLaunchArgument(
            'software_revision', default_value='unversioned'),
        localization,
        evaluator,
    ])
~~~

- [ ] **Step 4: Run static and installed launch checks**

Run:

~~~bash
cd /home/track-robot/track_robot_ws
pytest -q src/track_robot_semantic_search/test/test_launch_contract.py
python3 -m py_compile \
  src/track_robot_semantic_search/launch/semantic_search_phase0.launch.py
bash -lc 'source /opt/ros/foxy/setup.bash && cd /home/track-robot/track_robot_ws && colcon build --packages-select track_robot_interfaces track_robot_semantic_search --symlink-install --event-handlers console_direct+'
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && ros2 launch track_robot_semantic_search semantic_search_phase0.launch.py --show-args'
~~~

Expected: tests pass; compilation succeeds; launch arguments list start_evaluator with default false.

- [ ] **Step 5: Prove the new source contains no motion publisher**

Run:

~~~bash
rg -n 'create_publisher.*(Twist|FollowDecision|SearchMotionIntent)' \
  src/track_robot_semantic_search/track_robot_semantic_search \
  src/track_robot_semantic_search/launch
rg -n '/cmd_vel' src/track_robot_semantic_search/launch
~~~

Expected: both commands have no matches and exit 1. The SearchMotionIntent
subscription in evaluator_node.py is allowed. Manifest evidence-topic literals
are also allowed; the checks reject motion publisher creation anywhere in the
package and cmd_vel topic literals in the new launch contract.

- [ ] **Step 6: Commit the passive launch**

~~~bash
git add \
  src/track_robot_semantic_search/launch/semantic_search_phase0.launch.py \
  src/track_robot_semantic_search/test/test_launch_contract.py
git commit -m "feat: add passive semantic search launch"
~~~

### Task 6: Document Recording and Generate the Legacy Manifest

**Files:**

- Create: docs/guides/semantic-search/rosbag-workflow.md
- Create: artifacts/semantic_search/manifests/README.md
- Create: artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json

**Interfaces:**

- The guide defines the authoritative topic set, adjacent manifest/query-event workflow, recording stop/checksum order, replay rates, diagnostics, evaluator invocation, and pass/fail interpretation.
- The legacy manifest is generated from metadata.yaml and closed bag files; it is not hand-edited.

- [ ] **Step 1: Write the guide with exact recording and replay commands**

Create semantic_search_rosbag_guide.md with these required sections and commands:

~~~markdown
# Semantic Search Rosbag Guide

## Safety and Scope

Phase 0 is passive. Do not start the live camera/LiDAR drivers while replaying
on the same ROS_DOMAIN_ID. Do not start decision, controller, planner, safety
motion, base, or cmd_vel nodes from this guide.

## Dataset Bundle

A recording is:

1. a closed rosbag2 directory;
2. one JSON manifest;
3. optional JSONL frame annotations;
4. one evaluator report;
5. optional raw tegrastats outside Git.

ROS 2 Foxy rosbag2 does not reliably preserve action service goal payloads.
Therefore query text, query ID, and source timestamp are recorded in the
adjacent manifest with semantic_search_manifest add-query.

## Record a New Field Bag

Terminal 1:

    source /opt/ros/foxy/setup.bash
    source ~/track_robot_ws/install/setup.bash
    export ROS_DOMAIN_ID=21
    ros2 bag record -o ~/track_robot_ws/rosbags/semantic_search/recordings/field_001 \
      /zed/zed_node/left/image_rect_color \
      /zed/zed_node/left/camera_info \
      /rslidar_points \
      /imu/data_raw \
      /odom \
      /localization/odometry \
      /tf \
      /tf_static \
      /semantic_search/regions \
      /semantic_search/observations \
      /semantic_search/tracked_objects \
      /semantic_search/localization_diagnostics \
      /semantic_search/motion_intent \
      /safety/state \
      /follow/cmd_vel_planned \
      /follow/cmd_vel_avoiding \
      /follow/cmd_vel_safe

Stop with Ctrl+C and wait for ros2 bag record to exit before checksumming.
Never commit db3, db3-wal, db3-shm, or tegrastats logs.

Terminal 2:

    tegrastats --interval 1000 \
      --logfile /tmp/semantic_search_field_001_tegrastats.log

Stop telemetry after recording:

    tegrastats --stop

## Create the Field Manifest

Set all four variables to the immutable ID or checksum of the calibration
artifact actually used. The guard aborts if any value is missing:

    : "${CAMERA_INTRINSICS_ID:?set verified camera intrinsics ID}"
    : "${CAMERA_LIDAR_EXTRINSICS_ID:?set verified camera-LiDAR ID}"
    : "${LIDAR_IMU_EXTRINSICS_ID:?set verified LiDAR-IMU ID}"
    : "${LOCALIZATION_CONFIG_ID:?set verified localization config ID}"
    ros2 run track_robot_semantic_search semantic_search_manifest create-field \
      ~/track_robot_ws/rosbags/semantic_search/recordings/field_001 \
      ~/track_robot_ws/artifacts/semantic_search/manifests/field_001.json \
      --dataset-id field_001 \
      --workspace-root ~/track_robot_ws \
      --split validation \
      --site-id site_001 \
      --session-id session_001 \
      --lighting daylight \
      --surface outdoor_path \
      --weather dry \
      --camera-intrinsics-id "$CAMERA_INTRINSICS_ID" \
      --camera-lidar-extrinsics-id "$CAMERA_LIDAR_EXTRINSICS_ID" \
      --lidar-imu-extrinsics-id "$LIDAR_IMU_EXTRINSICS_ID" \
      --localization-config-id "$LOCALIZATION_CONFIG_ID"

Do not add --world-pose or --active-motion unless the corresponding topics,
calibration, and controlled-test authority are present. The validator rejects
unknown/unverified calibration IDs for field manifests.

## Add Query Events to the Manifest

Use the ROS source timestamp, not wall-clock time:

    : "${QUERY_STAMP_NS:?set the exact ROS event stamp in nanoseconds}"
    ros2 run track_robot_semantic_search semantic_search_manifest add-query \
      artifacts/semantic_search/manifests/field_001.json \
      --query-id 1 \
      --stamp-ns "$QUERY_STAMP_NS" \
      --text "fallen branch blocking the path" \
      --language en \
      --client-request-id field-001

Set QUERY_STAMP_NS from the operator/event log for this run. The guard prevents
a missing value from silently creating an invalid event.

## Register Objects, Trials, and Annotations

Declare physical-object grouping before a trial so that site/date/object splits
cannot be inferred from adjacent frames:

    : "${ACQUISITION_DATE:?set the recording date as YYYY-MM-DD}"
    ros2 run track_robot_semantic_search semantic_search_manifest add-object \
      artifacts/semantic_search/manifests/field_001.json \
      --object-id branch-001-field-001 \
      --physical-object-id branch-001 \
      --label "fallen branch" \
      --site-id site_001 \
      --acquisition-date "$ACQUISITION_DATE" \
      --source robot \
      --provenance human-labelled

Set the exact trial bounds from the event log, then register the positive
passive trial:

    : "${TRIAL_START_NS:?set the exact trial start stamp}"
    : "${TRIAL_END_NS:?set the exact trial end stamp}"
    : "${NOMINAL_DISTANCE_M:?set the measured nominal distance in metres}"
    ros2 run track_robot_semantic_search semantic_search_manifest add-trial \
      artifacts/semantic_search/manifests/field_001.json \
      --trial-id trial-001 \
      --query-id 1 \
      --target-object-id branch-001-field-001 \
      --positive \
      --start-stamp-ns "$TRIAL_START_NS" \
      --end-stamp-ns "$TRIAL_END_NS" \
      --nominal-distance-m "$NOMINAL_DISTANCE_M" \
      --observation-stage passive \
      --site-id site_001 \
      --session-id session_001

After the labelling tool writes one JSON object per line using
annotation.schema.json, validate every record and register its immutable hash:

    ros2 run track_robot_semantic_search semantic_search_manifest add-annotations \
      artifacts/semantic_search/manifests/field_001.json \
      artifacts/semantic_search/annotations/field_001.jsonl \
      --workspace-root ~/track_robot_ws

The command rejects undeclared query, object, or trial IDs, records outside the
trial interval, invalid geometry, duplicate annotation paths, and malformed
JSONL.

## Legacy Replay

The pinned Foxy rosbag2 0.3.11 player cannot publish a replay clock. Run this
legacy evaluation on wall time and scale evaluator duration inversely with
replay rate so each report targets exactly 45.0 seconds of source time. Formal
evidence uses report schema `1.1.0`, `foxy_wall_time_scaled` timing,
`arrival_monotonic` freshness, and at least `0.90` source-window coverage for
every required topic with at least two messages.

Terminal 1:

    source /opt/ros/foxy/setup.bash
    source ~/track_robot_ws/install/setup.bash
    export ROS_DOMAIN_ID=20
    ros2 launch track_robot_semantic_search semantic_search_phase0.launch.py \
      use_sim_time:=false \
      start_evaluator:=true \
      replay_rate:=1.0 \
      timing_policy:=foxy_wall_time_scaled \
      freshness_time_base:=arrival_monotonic \
      manifest_path:=$HOME/track_robot_ws/artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json \
      output_path:=/tmp/semantic_search_rate_10.json \
      duration_sec:=45.0

Terminal 2:

    source /opt/ros/foxy/setup.bash
    source ~/track_robot_ws/install/setup.bash
    export ROS_DOMAIN_ID=20
    ros2 bag play \
      ~/track_robot_ws/rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150711 \
      --rate 1.0

Use these exact rate and wall-duration pairs:

- `replay_rate:=0.5`, `--rate 0.5`, `duration_sec:=90.0`;
- `replay_rate:=1.0`, `--rate 1.0`, `duration_sec:=45.0`;
- `replay_rate:=2.0`, `--rate 2.0`, `duration_sec:=22.5`.

Change output names to rate_05, rate_10, and rate_20. Source rates and
synchronization use message header stamps; receive rates use evaluator arrival
times and therefore change with replay rate. Start the launch and bag from
fresh processes for every rate, and do not use `--loop`: timestamp rollback is
an epoch boundary and invalidates a single-run comparison. Reports produced
this way are comparable only within this pinned Foxy wall-time policy; do not
mix them with future native replay-clock reports.

Complete each rate in this order:

1. Wait for the evaluator's `wrote` log, then confirm the atomically replaced
   report is present and valid JSON. Set `REPORT_PATH` to that rate's output:

       REPORT_PATH=/tmp/semantic_search_rate_10.json
       test -s "$REPORT_PATH" && python3 -m json.tool "$REPORT_PATH" >/dev/null

2. In Terminal 2, stop that rate's `ros2 bag play` with Ctrl+C if it is still
   running, or wait if it is already finishing. Wait for the process to exit.
3. Stop and wait for the Terminal 3 diagnostics echo if it was started.
4. Stop and wait for that rate's tegrastats process if telemetry was started.
5. Press Ctrl+C in Terminal 1 and wait for both localization and evaluator
   processes to exit.
6. In a sourced shell on ROS_DOMAIN_ID 20, run these read-only checks:

       BAG_PATH="$HOME/track_robot_ws/rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150711"
       ps -eo pid,ppid,stat,cmd | \
         awk -v needle="ros2 bag play ${BAG_PATH}" \
           'NR > 1 && $4 != "awk" && index($0, needle)'
       ros2 node list | \
         grep -E '(^|/)(semantic_search_localization_health|semantic_search_evaluator)$'

Both checks must print no matches. Stop only the processes started for the
current rate; never use `pkill` or `killall`, and never signal another user's
ROS processes. Do not start the next rate until the bag player and both
semantic nodes are absent.

Compare:

    ros2 run track_robot_semantic_search semantic_search_compare_reports \
      --manifest "$HOME/track_robot_ws/artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json" \
      /tmp/semantic_search_rate_05.json \
      /tmp/semantic_search_rate_10.json \
      /tmp/semantic_search_rate_20.json

## Expected Legacy Result

- camera and LiDAR topics are present;
- P95 nearest image/cloud offset is at most 0.08 second;
- localisation mode is OBSERVATION_ONLY only;
- no forward-permission violation occurs;
- missing IMU/local/world pose remain declared capability gaps;
- the legacy baseline proves contracts, replay mechanics, and diagnostics
  only; it does not prove semantic perception, 3D object memory, language
  grounding, motion safety, or active-search safety.
~~~

- [ ] **Step 2: Add manifest policy**

Create manifests/README.md:

~~~markdown
# Semantic Search Manifests

Every manifest is immutable after a report references its SHA-256. Before
evaluation, query events, object records, trial records, and annotation-file
hashes may be appended only through semantic_search_manifest. Paths are
workspace-relative.
Train/validation/test splits are assigned by physical object, site, and date,
never adjacent frames. Legacy bags use legacy_replay_only and cannot be promoted
to a field-test split.

Generate and extend manifests with semantic_search_manifest; do not hand-edit
checksums or rosbag metadata. Annotation JSONL uses the installed annotation
schema and is validated before its hash is registered.
~~~

- [ ] **Step 3: Generate and validate the real legacy manifest**

Run only after Task 2 is installed:

~~~bash
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && cd /home/track-robot/track_robot_ws && ros2 run track_robot_semantic_search semantic_search_manifest create-legacy rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150711 artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json --dataset-id human_tracking_lidar_20260706_150711 --workspace-root /home/track-robot/track_robot_ws'
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && cd /home/track-robot/track_robot_ws && ros2 run track_robot_semantic_search semantic_search_manifest validate artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json'
~~~

Expected: validation prints dataset_id human_tracking_lidar_20260706_150711 and valid true. Inspect capabilities: camera/lidar true; IMU/local_pose/world_pose/query_events/annotations/active_motion false.

- [ ] **Step 4: Commit recording documentation and manifest**

~~~bash
git add \
  docs/guides/semantic-search/rosbag-workflow.md \
  artifacts/semantic_search/manifests/README.md \
  artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json
git commit -m "docs: add semantic search recording contract"
~~~

### Task 7: Run the Full Phase 0 Gate and Version the Baseline

**Files:**

- Create: artifacts/semantic_search/reports/README.md
- Create from evaluator output: artifacts/semantic_search/reports/phase0_baseline_2026-07-14.json
- Verify only: existing human-tracking, decision, control, and safety files

**Interfaces:**

- The committed baseline is the 1.0× legacy replay report for the exact manifest checksum.
- The 0.5× and 2.0× reports remain /tmp comparison artifacts unless deliberately promoted with their manifests.
- A passing legacy baseline proves contracts, replay mechanics, and diagnostics only; it does not prove semantic perception, 3D object memory, language grounding, motion safety, or active-search safety.

- [ ] **Step 1: Add the report policy**

Create reports/README.md:

~~~markdown
# Semantic Search Reports

Small JSON reports are versioned with the manifest and configuration checksums,
run ID, replay rate, software revision when available, source/receive rates, synchronization, evaluator
callback latency, localisation modes, resource samples, safety violations, and
explicit gates.

Formal legacy evidence uses report schema `1.1.0` and exactly three unique
reports: 0.5x for 90.0 wall seconds, 1.0x for 45.0 wall seconds, and 2.0x for
22.5 wall seconds. Each run uses `foxy_wall_time_scaled`,
`arrival_monotonic`, a 45.0-second source target, and minimum source coverage
`0.90`; every required topic also needs at least two messages. The comparator
is invoked with `--manifest` and recomputes all five hard gates.

Raw tegrastats, bags, images, masks, feature tensors, and model weights remain
external. The legacy baseline proves contracts, replay mechanics, and
diagnostics only; it does not prove semantic perception, 3D object memory,
language grounding, motion safety, or active-search safety.
~~~

- [ ] **Step 2: Build and run all Phase 0 unit/contract tests**

Run:

~~~bash
bash -lc 'source /opt/ros/foxy/setup.bash && cd /home/track-robot/track_robot_ws && colcon build --packages-select track_robot_interfaces track_robot_semantic_search --symlink-install --event-handlers console_direct+'
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && cd /home/track-robot/track_robot_ws && colcon test --packages-select track_robot_interfaces track_robot_semantic_search --event-handlers console_direct+'
bash -lc 'cd /home/track-robot/track_robot_ws && colcon test-result --verbose'
~~~

Expected: all interface, manifest, localisation, evaluator, and launch tests pass; zero failures.

- [ ] **Step 3: Run the 1.0× versioned baseline**

The pinned Foxy rosbag2 0.3.11 player cannot publish a replay clock. Use wall
time for these legacy runs and scale `duration_sec` inversely with replay rate
so every report targets approximately 45 seconds of source time. Source rates
and synchronization are computed from message header stamps; receive rates are
computed from evaluator arrival times and therefore change with replay rate.
Reports are comparable only within this pinned Foxy wall-time policy and must
not be mixed with future native replay-clock reports. Formal evidence uses
report schema `1.1.0`, requires at least `0.90` of the 45.0-second source target
on every required topic with at least two messages, and is accepted only as the
exact 0.5x/90.0, 1.0x/45.0, and 2.0x/22.5 three-report set.

Terminal A:

~~~bash
tegrastats --interval 1000 \
  --logfile /tmp/semantic_search_phase0_tegrastats.log
~~~

Terminal B:

~~~bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=22
ros2 launch track_robot_semantic_search semantic_search_phase0.launch.py \
  use_sim_time:=false \
  start_evaluator:=true \
  run_id:=legacy-rate-1.0 \
  replay_rate:=1.0 \
  timing_policy:=foxy_wall_time_scaled \
  freshness_time_base:=arrival_monotonic \
  software_revision:="$(git -C "$HOME/track_robot_ws" rev-parse HEAD)" \
  manifest_path:=$HOME/track_robot_ws/artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json \
  output_path:=$HOME/track_robot_ws/artifacts/semantic_search/reports/phase0_baseline_2026-07-14.json \
  tegrastats_path:=/tmp/semantic_search_phase0_tegrastats.log \
  duration_sec:=45.0
~~~

Terminal C:

~~~bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=22
ros2 bag play \
  ~/track_robot_ws/rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150711 \
  --rate 1.0
~~~

Complete the 1.0x run in this order:

1. Wait for the evaluator's `wrote` log, then confirm the atomically replaced
   report is present and valid JSON:

~~~bash
REPORT_PATH=$HOME/track_robot_ws/artifacts/semantic_search/reports/phase0_baseline_2026-07-14.json
test -s "$REPORT_PATH" && python3 -m json.tool "$REPORT_PATH" >/dev/null
~~~

2. In Terminal C, stop `ros2 bag play` with Ctrl+C if it is still running, or
   wait if it is already finishing. Wait for the process to exit.
3. Stop and wait for any diagnostics echo started for this rate.
4. Stop and wait for the corresponding Terminal A tegrastats process.
5. Press Ctrl+C in Terminal B and wait for both localization and evaluator
   processes to exit.
6. In a sourced shell on ROS_DOMAIN_ID 22, run these read-only checks:

~~~bash
BAG_PATH="$HOME/track_robot_ws/rosbags/human_tracking/recordings/human_tracking_lidar_20260706_150711"
ps -eo pid,ppid,stat,cmd | \
  awk -v needle="ros2 bag play ${BAG_PATH}" \
    'NR > 1 && $4 != "awk" && index($0, needle)'
ros2 node list | \
  grep -E '(^|/)(semantic_search_localization_health|semantic_search_evaluator)$'
~~~

Both checks must print no matches. Stop only the processes started for the
current rate; never use `pkill` or `killall`, and never signal another user's
ROS processes. Expected report facts:

- dataset_id is human_tracking_lidar_20260706_150711;
- image and lidar counts are positive;
- source rates are approximately 14–15 Hz and 19–20 Hz;
- localization mode counts contain OBSERVATION_ONLY only;
- forward_permission_violations is zero;
- P95 image/LiDAR offset is at most 0.08 s;
- image_callback and lidar_callback latency series are present (these measure
  Phase 0 evaluator overhead, not future model-inference latency);
- software_revision is the checked-out commit and config_sha256 is populated;
- passed is true;
- tegrastats fields are report-only and non-empty on Jetson.

- [ ] **Step 4: Validate the report shape and hard gates**

Run:

~~~bash
python3 -m json.tool \
  artifacts/semantic_search/reports/phase0_baseline_2026-07-14.json \
  > /tmp/phase0_baseline_pretty.json
python3 -c "import json; p='artifacts/semantic_search/reports/phase0_baseline_2026-07-14.json'; r=json.load(open(p)); assert r['passed']; assert r['safety']['forward_permission_violations']==0; assert set(r['localization']['mode_counts'])=={'OBSERVATION_ONLY'}; assert {'image_callback','lidar_callback'} <= set(r['latency_metrics']); assert r['artifacts']['software_revision'] != 'unversioned'; assert len(r['artifacts']['config_sha256']) == 64; print('phase0 baseline gates passed')"
~~~

Expected: “phase0 baseline gates passed”.

- [ ] **Step 5: Repeat at 0.5× and 2.0× from fresh processes**

Repeat Step 3 with these exact launch rate, bag rate, and wall-duration pairs:

- `replay_rate:=0.5`, `--rate 0.5`, `duration_sec:=90.0`;
- `replay_rate:=1.0`, `--rate 1.0`, `duration_sec:=45.0`;
- `replay_rate:=2.0`, `--rate 2.0`, `duration_sec:=22.5`.

Use run IDs legacy-rate-0.5 and legacy-rate-2.0 and outputs
/tmp/semantic_search_rate_05.json and /tmp/semantic_search_rate_20.json for the
additional runs. Restart the launch, bag, and telemetry processes between
runs. For each output, repeat the complete Step 3 teardown in the same order:
atomic report confirmation, bag exit, optional diagnostics exit, telemetry
exit, launch exit, then the read-only bag-process and node checks. Do not start
the next rate until both checks print no matches. Keep the same manifest,
configuration, and software revision; do not use `--loop`. Stop only the
processes started for that rate, without `pkill` or `killall`.

Then run:

~~~bash
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && ros2 run track_robot_semantic_search semantic_search_compare_reports --manifest artifacts/semantic_search/manifests/legacy/human_tracking_lidar_20260706_150711.json /tmp/semantic_search_rate_05.json artifacts/semantic_search/reports/phase0_baseline_2026-07-14.json /tmp/semantic_search_rate_20.json'
~~~

Expected: passed true, no failures. The comparator validates the manifest
binding, enforces the exact three-rate policy, and recomputes all five hard
gates; source sequence hashes are displayed but are not themselves hard gates.

- [ ] **Step 6: Run existing decision regression tests**

Run:

~~~bash
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && cd /home/track-robot/track_robot_ws && colcon test --packages-select track_robot_decision --event-handlers console_direct+'
bash -lc 'cd /home/track-robot/track_robot_ws && colcon test-result --verbose'
~~~

Expected: existing follow-decision launch tests pass. No semantic-search launch is included by a human-tracking launch.

- [ ] **Step 7: Verify protected files and no motion publisher**

Run:

~~~bash
sha256sum -c /tmp/semantic_search_phase0_protected.sha256
rg -n 'create_publisher.*(Twist|FollowDecision|SearchMotionIntent)' \
  src/track_robot_semantic_search/track_robot_semantic_search \
  src/track_robot_semantic_search/launch
rg -n '/cmd_vel' src/track_robot_semantic_search/launch
~~~

Expected: all five protected files report OK; both rg commands have no matches
and exit 1. A subscriber import and manifest evidence-topic literals are
permitted, but no motion publisher or cmd_vel launch literal is permitted.

- [ ] **Step 8: Verify package files install and launch remains opt-in**

Run:

~~~bash
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && ros2 pkg prefix track_robot_semantic_search'
bash -lc 'source /opt/ros/foxy/setup.bash && source /home/track-robot/track_robot_ws/install/setup.bash && ros2 launch track_robot_semantic_search semantic_search_phase0.launch.py --show-args'
rg -n 'track_robot_semantic_search|semantic_search_phase0' \
  src/track_robot_perception/launch \
  src/track_robot/track_robot_bringup/launch
~~~

Expected: package prefix resolves; start_evaluator defaults false; existing perception/bringup launches contain no semantic-search include and rg exits 1.

- [ ] **Step 9: Commit the passing Phase 0 evidence**

~~~bash
git add \
  artifacts/semantic_search/reports/README.md \
  artifacts/semantic_search/reports/phase0_baseline_2026-07-14.json
git commit -m "test: record semantic search phase zero baseline"
~~~

- [ ] **Step 10: Perform the final scope audit**

Run:

~~~bash
git diff --check
git status --short
rg -n -i 'T[B]D|T[O]DO|F[I]XME|[Pp]lace''holder|implement la[t]er|similar to ta[s]k' \
  src/track_robot_semantic_search \
  rosbags/semantic_search
~~~

Expected: git diff --check exits 0; status contains no uncommitted Phase 0 files; the incomplete-marker scan has no matches. Existing unrelated user changes may remain and must not be staged.

## Phase 0 Definition of Done

Phase 0 is complete only when all of the following are evidenced:

1. all seven messages and SearchForObject action build and round-trip;
2. no raw feature/tensor field exists in a ROS contract;
3. package and schemas install without adding a new Python dependency;
4. manifest validator rejects unsafe paths, invalid checksums, duplicate queries, and capability contradictions;
5. old bag manifest truthfully declares missing IMU/pose/query/annotation/action evidence;
6. localisation tests prove WORLD, LOCAL_SESSION, OBSERVATION_ONLY, epoch jump, and timestamp rollback behavior;
7. passive launch defaults evaluator off and starts no motion component;
8. 0.5×, 1.0×, and 2.0× reports pass capability-aware replay gates;
9. the versioned 1.0× report contains manifest/config/revision provenance, replay rate, rates, synchronization, evaluator callback latency, localisation, safety, process/system CPU and RAM, and Jetson telemetry;
10. all existing decision regression tests pass;
11. protected human launch/config/safety hashes are unchanged;
12. no Phase 1 model or runtime inference code was added.

## Deferred to Later Approved Plans

- DINO aspect-preserving preprocessing and model worker;
- text-model benchmark and query cache;
- region proposals and open-vocabulary scoring;
- generic semantic-search LiDAR configuration and 3D fusion;
- semantic object memory and association;
- trainable fusion, uncertainty, baselines, and ablations;
- action server, motion bridge, and bounded rotation;
- TensorRT/ONNX optimization.

## Execution Handoff

Plan execution begins from the authorized local-only baseline and isolated
feature/semantic-search-phase0 worktree because every task has a commit/review
gate. No remote repository is required.

1. **Subagent-Driven (recommended):** dispatch a fresh worker per task with review between tasks.
2. **Inline Execution:** execute in this session in batches with explicit checkpoints.

At completion, merge or export locally for the user's existing manual GitHub
Workspace release-upload process; do not create a remote or push automatically.
