# Phase 2 Depth-Backed Memory and Phase 4B Operator Gate Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Produce a repeatable Domain 20 Phase 0-4B supervised workflow in
which camera plus ZED depth creates the canonical Phase 2 target, dynamic Nav2
obstacles clear, and an operator must authorize the exact target in RViz before
motion.

**Architecture:** Enrich existing semantic observations with registered stereo
depth before the Phase 2 memory boundary, then teach MemoryCore to retain
camera-depth geometry without changing ID/lifecycle ownership. Split Nav2
marking and clearing sources, add exact-reference authorization to the semantic
navigation supervisor, expose that service through the RViz panel, and wrap the
workflow in one managed CLI command.

**Tech Stack:** ROS 2 Foxy, Python 3.8/rclpy, C++17/rclcpp, Qt5/RViz2, Nav2
0.4.7, ZED registered depth, pytest/gtest/ament, colcon.

## Global Constraints

- Use `ROS_DOMAIN_ID=20`.
- Do not use IMU.
- Use fused `task_relevance >= 0.30`; do not apply `0.30` to raw YOLO score.
- Preserve Phase 2 memory/global ID and lifecycle ownership.
- The RViz plugin must not publish velocity or create a Nav2 action client.
- Motion must remain Nav2 -> safety supervisor -> velocity gate -> Bunker.
- `SEMANTIC_ACTIVE` must remain unable to move without exact operator approval.
- RC override and E-stop cancel authorization; transient obstacle blocking may
  hold and resume the same approved target.
- Stop all ROS nodes and services after live tests.

---

### Task 1: Spatial semantic observation enrichment

**Files:**
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/spatial_observation.py`
- Create: `src/track_robot_semantic_search/track_robot_semantic_search/spatial_observation_node.py`
- Create: `src/track_robot_semantic_search/test/test_spatial_observation.py`
- Modify: `src/track_robot_semantic_search/setup.py`
- Modify: `src/track_robot_semantic_search/config/semantic_search_phase4a.yaml`
- Modify: `src/track_robot/track_robot_bringup/launch/semantic_search_phase4a.launch.py`
- Modify: `src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py`

**Interfaces:**
- Consumes: `/semantic_memory/observations`,
  `/zed/zed_node/depth/depth_registered`,
  `/zed/zed_node/left/camera_info`,
  `/semantic_search/phase4a/localization_state`, and timestamped TF.
- Produces: `/semantic_memory/spatial_observations` with the same
  `SemanticObservationArray` identity and ordering.
- Produces: pure
  `spatialize_observation(observation, depth, intrinsics, transform,
  localization_epoch_id, config)` behavior used by the ROS adapter.

- [ ] **Step 1: Write failing pure tests**

  Test that valid registered depth sets `position_valid`, `base_link`,
  covariance, localization epoch, pose/TF stamps and
  `EVIDENCE_STEREO_DEPTH`; invalid/stale depth returns an unchanged observation.

- [ ] **Step 2: Verify RED**

  Run:

  ```bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    pytest -q src/track_robot_semantic_search/test/test_spatial_observation.py
  ```

  Expected: import or assertion failure because the spatializer does not exist.

- [ ] **Step 3: Implement the pure spatializer and ROS node**

  Reuse `CameraIntrinsics`, `estimate_depth_point` and `transform_point` from
  `phase4a_depth.py`. Publish every input array exactly once; failed geometry
  leaves individual observations unchanged instead of dropping the array.

- [ ] **Step 4: Add launch and entry-point contracts**

  Register:

  ```python
  'semantic_search_spatial_observation = '
  'track_robot_semantic_search.spatial_observation_node:main'
  ```

  Launch it before semantic memory and configure memory's observation topic as
  `/semantic_memory/spatial_observations` only in the Phase 4A/4B profile.

- [ ] **Step 5: Verify GREEN**

  Run the focused pytest plus:

  ```bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
    src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py
  ```

- [ ] **Step 6: Commit**

  ```bash
  git add src/track_robot_semantic_search \
    src/track_robot/track_robot_bringup/launch/semantic_search_phase4a.launch.py \
    src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py
  git commit -m "feat(semantic-search): enrich observations with ZED depth"
  ```

### Task 2: Canonical camera-depth geometry in Phase 2

**Files:**
- Modify: `src/track_robot/track_robot_semantic_memory/include/track_robot_semantic_memory/memory_core.hpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/memory_core.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/src/ros_conversions.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_memory_core.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_ros_conversions.cpp`
- Modify: `src/track_robot/track_robot_semantic_memory/config/phase4a_test.yaml`
- Modify: `src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py`
- Modify: `src/track_robot_semantic_search/config/semantic_search_phase4a.yaml`
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/phase4a_selector_node.py`
- Modify: `src/track_robot_semantic_search/test/test_phase4a_selector.py`

**Interfaces:**
- Consumes: position-valid `SemanticObservation` with
  `EVIDENCE_STEREO_DEPTH`.
- Produces: canonical `/semantic_memory/active_objects` and
  `/semantic_memory/best_candidate` with a stable Phase 2 global ID and valid
  position even when `lidar_tracklet_id_valid=false`.

- [ ] **Step 1: Write failing conversion and MemoryCore tests**

  Assert that a valid camera-depth observation:

  - creates one camera-owned object;
  - becomes confirmed through the existing lifecycle hit count;
  - exports `position_valid=true`, `velocity_valid=false`,
    `extent_valid=false`;
  - retains the same global ID across depth updates;
  - retains that ID when later attached to LiDAR.

- [ ] **Step 2: Verify RED**

  Build and run:

  ```bash
  colcon build --packages-select track_robot_semantic_memory
  colcon test --packages-select track_robot_semantic_memory \
    --event-handlers console_direct+
  ```

  Expected: new camera-depth assertions fail because conversion currently
  ignores observation geometry.

- [ ] **Step 3: Extend internal observation and object geometry**

  Add explicit internal position validity and covariance independent of
  `lidar_key`. Validate finite geometry and frame/epoch metadata. Update camera
  position with bounded confidence weighting and never create a second ID for
  the same stable visual key.

- [ ] **Step 4: Enable the supervised fused threshold**

  In `phase4a_test.yaml`, leave the default production config unchanged and set:

  ```yaml
  best_candidate_threshold_calibrated: true
  best_candidate_minimum_relevance: 0.30
  ```

  Set the Phase 4A selector/planner fused relevance floor to `0.30`, and let a
  position-valid camera-only object satisfy the selector's `camera_depth`
  support condition.

- [ ] **Step 5: Verify GREEN and deterministic replay**

  Run all semantic-memory tests plus:

  ```bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
    src/track_robot_semantic_search/test/test_phase4a_selector.py
  ```

- [ ] **Step 6: Commit**

  ```bash
  git add src/track_robot/track_robot_semantic_memory \
    src/track_robot_semantic_search/config/semantic_search_phase4a.yaml \
    src/track_robot_semantic_search/track_robot_semantic_search/phase4a_selector_node.py \
    src/track_robot_semantic_search/test/test_phase4a_selector.py
  git commit -m "feat(semantic-memory): retain camera depth geometry"
  ```

### Task 3: Bounded dynamic-obstacle clearing

**Files:**
- Modify: `src/track_robot/track_robot_navigation/config/nav2_phase4b.yaml`
- Modify: `src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py`
- Create: `src/track_robot/track_robot_navigation/test/test_dynamic_obstacle_sources.py`
- Modify: `src/track_robot/track_robot_navigation/CMakeLists.txt`

**Interfaces:**
- Consumes clearing source `/rslidar_points`.
- Consumes marking source `/safety/filtered_obstacle_points`.
- Produces current global/local Nav2 costmaps without unbounded human trails.

- [ ] **Step 1: Write failing configuration tests**

  Require both costmaps to declare `raw_clear` and `filtered_mark`, zero
  observation persistence, clearing-only raw input, marking-only filtered
  input, and finite expected update rates.

- [ ] **Step 2: Verify RED**

  ```bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
    src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py \
    src/track_robot/track_robot_navigation/test/test_dynamic_obstacle_sources.py
  ```

- [ ] **Step 3: Split marking and clearing sources**

  Configure both `VoxelLayer` and `ObstacleLayer` with:

  ```yaml
  observation_sources: raw_clear filtered_mark
  raw_clear:
    topic: /rslidar_points
    clearing: true
    marking: false
    observation_persistence: 0.0
  filtered_mark:
    topic: /safety/filtered_obstacle_points
    clearing: false
    marking: true
    observation_persistence: 0.0
  ```

  Keep the existing 8 m range, height, footprint and inflation limits.

- [ ] **Step 4: Run config and Nav2 package regression**

  ```bash
  colcon test --packages-select track_robot_navigation \
    --event-handlers console_direct+
  ```

- [ ] **Step 5: Commit**

  ```bash
  git add src/track_robot/track_robot_navigation/config/nav2_phase4b.yaml \
    src/track_robot/track_robot_navigation/test
  git commit -m "fix(navigation): separate obstacle marking and clearing"
  ```

### Task 4: Exact-target operator authorization

**Files:**
- Create: `src/track_robot/track_robot_interfaces/srv/AuthorizeSemanticApproach.srv`
- Modify: `src/track_robot/track_robot_interfaces/CMakeLists.txt`
- Modify: `src/track_robot/track_robot_interfaces/test/test_semantic_interfaces.py`
- Modify: `src/track_robot/track_robot_navigation/track_robot_navigation/semantic_goal_policy.py`
- Modify: `src/track_robot/track_robot_navigation/track_robot_navigation/semantic_navigation_supervisor_node.py`
- Modify: `src/track_robot/track_robot_navigation/config/semantic_navigation.yaml`
- Modify: `src/track_robot/track_robot_navigation/test/test_semantic_goal_policy.py`
- Modify: `src/track_robot/track_robot_navigation/test/test_launch_contract.py`

**Interfaces:**
- Produces service `/semantic_navigation/authorize`.
- Produces service `/semantic_navigation/cancel_and_disarm`.
- Calls existing `/safety/arm` and `/safety/disarm`.
- Never creates a velocity publisher.

- [ ] **Step 1: Write failing interface and policy tests**

  Define request fields:

  ```text
  uint64 memory_epoch_id
  uint64 global_object_id
  uint64 localization_epoch_id
  uint64 query_id
  uint64 query_version
  uint64 snapshot_sequence
  ---
  bool accepted
  string<=256 reason
  ```

  Policy tests must prove active mode holds with
  `operator_authorized=false`, navigates with an exact approval, rejects a
  mismatched/stale approval, cancels on reference/query change, and holds
  through a transient obstacle.

- [ ] **Step 2: Verify RED**

  Run interface pytest and semantic policy pytest. Expected failures are the
  missing service and approval state.

- [ ] **Step 3: Implement one-shot approval in the pure policy**

  Add the exact six-field reference to the policy snapshot and require it only
  for `SEMANTIC_ACTIVE`. Shadow behavior remains unchanged.

- [ ] **Step 4: Implement supervisor services**

  The authorize callback captures no implicit target: it compares every
  request field with the current correlated snapshot, requests safety arm, and
  marks approval only after arm succeeds. Cancel clears approval, cancels the
  action and requests disarm.

- [ ] **Step 5: Verify no-motion regressions**

  Run interface/navigation tests and confirm source contracts contain no path
  that publishes final `/cmd_vel`.

- [ ] **Step 6: Commit**

  ```bash
  git add src/track_robot/track_robot_interfaces \
    src/track_robot/track_robot_navigation
  git commit -m "feat(navigation): require exact semantic target approval"
  ```

### Task 5: RViz Start Approach and Cancel & Disarm controls

**Files:**
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/semantic_search_panel.hpp`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/src/semantic_search_panel.cpp`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/CMakeLists.txt`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/package.xml`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_plugin_contract.py`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_operator_reference.cpp`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/operator_reference.hpp`
- Create: `src/track_robot/track_robot_semantic_search_rviz_plugins/src/operator_reference.cpp`

**Interfaces:**
- Consumes canonical best candidate and Phase 4A selected target.
- Calls `/semantic_navigation/authorize` and
  `/semantic_navigation/cancel_and_disarm`.
- Produces no motion topic or action.

- [ ] **Step 1: Write failing reference and source-contract tests**

  Require the button to remain disabled until canonical best and selected
  target have the same complete reference. Require query revision to clear the
  cached reference. Continue forbidding `cmd_vel`, `rclcpp_action` and Nav2
  action types.

- [ ] **Step 2: Verify RED**

  Run the plugin gtest and pytest contract.

- [ ] **Step 3: Implement the UI**

  Add a green `Start Approach` button, a red `Cancel & Disarm` button, exact
  target/reference/readiness labels and asynchronous service callbacks.
  Disable Start immediately after a request until a fresh accepted state is
  observed.

- [ ] **Step 4: Build and test the plugin**

  ```bash
  colcon build --packages-select \
    track_robot_semantic_search_rviz_plugins
  colcon test --packages-select \
    track_robot_semantic_search_rviz_plugins \
    --event-handlers console_direct+
  ```

- [ ] **Step 5: Commit**

  ```bash
  git add src/track_robot/track_robot_semantic_search_rviz_plugins
  git commit -m "feat(rviz): add supervised semantic approach controls"
  ```

### Task 6: One-command supervised test and consolidated regression

**Files:**
- Modify: `src/track_robot/track_robot_bringup/track_robot_bringup/control_cli.py`
- Modify: `src/track_robot/track_robot_bringup/track_robot_bringup/process_control.py`
- Modify: `src/track_robot/track_robot_bringup/test/test_control_cli.py`
- Modify: `src/track_robot/track_robot_bringup/test/test_process_control.py`
- Modify: `src/track_robot/track_robot_bringup/launch/semantic_search_phase4b.launch.py`
- Modify: `docs/guides/semantic-search/phase4b-nav2-supervised-test.md`

**Interfaces:**
- Produces `semantic_search_ctl run phase4b`.
- Launches Domain 20 Phase 0-4B plus RViz with IMU disabled.
- On shutdown, calls cancel/disarm before terminating its verified process
  group.

- [ ] **Step 1: Write failing CLI/process tests**

  Require the exact command, fixed Domain 20 environment, Phase 4B launch
  arguments, no IMU argument, and best-effort bounded cancel/disarm before
  process termination.

- [ ] **Step 2: Verify RED**

  ```bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
    src/track_robot/track_robot_bringup/test/test_control_cli.py \
    src/track_robot/track_robot_bringup/test/test_process_control.py
  ```

- [ ] **Step 3: Implement the managed Phase 4B command**

  Keep existing passive `start/test` behavior unchanged. Add the explicit
  supervised `run phase4b` path and render a short fixed operator checklist.

- [ ] **Step 4: Update the guide**

  Make the one-command panel workflow the primary path. Move topic echo
  commands into troubleshooting and document blue/pink costmap meaning.

- [ ] **Step 5: Full build and regression**

  ```bash
  colcon build --packages-up-to \
    track_robot_bringup track_robot_navigation \
    track_robot_semantic_search_rviz_plugins
  colcon test --packages-select \
    track_robot_interfaces track_robot_semantic_search \
    track_robot_semantic_memory track_robot_navigation \
    track_robot_safety track_robot_bringup \
    track_robot_semantic_search_rviz_plugins \
    --event-handlers console_direct+
  colcon test-result --verbose
  ```

  Also run deterministic semantic-memory replay and verify no ROS process is
  left running.

- [ ] **Step 6: Commit**

  ```bash
  git add src/track_robot/track_robot_bringup \
    docs/guides/semantic-search/phase4b-nav2-supervised-test.md
  git commit -m "feat(bringup): add one-command supervised Phase 4B test"
  ```

## Live acceptance after offline gates

Run only after all six tasks pass:

1. Launch `semantic_search_ctl run phase4b`.
2. Enter the agreed English target in RViz.
3. Confirm one canonical best candidate, stable global ID and valid position.
4. Confirm the robot remains stationary before authorization.
5. Walk through the LiDAR field and verify blue/pink costs clear after leaving.
6. Click `Start Approach` and verify the exact target reference is authorized.
7. Exercise RC override and `Cancel & Disarm`.
8. Save the bounded report and stop all launched nodes.
