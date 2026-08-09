# Phase 5 Robot Model and TF Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Display the Bunker Pro 2 model in Phase 4A/4B/5A RViz and produce the single-parent motion tree `odom -> robot_bottom -> base_link`.

**Architecture:** `robot_state_publisher` owns fixed URDF edges. The configurable Bunker driver owns only `odom -> robot_bottom`; existing Nav2 and perception consumers retain `base_link` and resolve it through the fixed URDF edge.

**Tech Stack:** ROS 2 Foxy, URDF, Python launch, RViz2, `robot_state_publisher`, pytest, colcon

## Global Constraints

- Preserve all existing sensor-station, camera, and LiDAR joint transforms exactly.
- Keep `camera_link` and `lidar_link` empty and add no duplicate meshes.
- Define `robot_bottom -> base_link` as `xyz="0 0 0.45"`, `rpy="0 0 0"`.
- Preserve the generic platform default `base_frame=base_link`.
- Phase 4B and Phase 5A must explicitly use `base_frame=robot_bottom`.
- Keep Nav2 `robot_base_frame=base_link` and preserve safety/control behavior.
- Do not modify unrelated files or publish motion during validation.

---

### Task 1: Publish the physical URDF tree

**Files:**
- Modify: `src/bunker_pro2/urdf/bunker_pro2.urdf`
- Create: `src/bunker_pro2/launch/description.launch.py`
- Modify: `src/bunker_pro2/launch/display.launch.py`
- Modify: `src/bunker_pro2/test/test_description_contract.py`
- Modify: `src/bunker_pro2/README.md`

**Interfaces:**
- Produces `/robot_description` plus the fixed tree rooted at `robot_bottom`.

- [ ] Add failing tests for the empty `robot_bottom`, exact 0.45 m joint, pure description launch, and standalone `world -> robot_bottom` viewer.
- [ ] Run `python3 -m pytest -q src/bunker_pro2/test/test_description_contract.py` and confirm failure.
- [ ] Add the root link/joint, pure description launch, and viewer composition without changing calibrated sensor joints.
- [ ] Re-run the package test and confirm it passes.
- [ ] Commit the independent URDF/description change.

### Task 2: Select the correct dynamic odometry child

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_platform.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase4b.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase5a.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py`

**Interfaces:**
- Generic platform consumes `base_frame`, default `base_link`, and forwards it to `bunker_base.launch.py`.
- Phase 4B/5A consume the platform interface with the constant `robot_bottom`.

- [ ] Add failing tests for platform declaration/forwarding and explicit Phase 4B/5A `robot_bottom` selection.
- [ ] Run the three affected bringup tests and confirm failure.
- [ ] Implement only launch-argument forwarding; do not modify the Bunker driver or Nav2 configs.
- [ ] Re-run the tests and confirm they pass.
- [ ] Commit the independent dynamic-TF ownership change.

### Task 3: Compose description and show the model

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase4a.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/package.xml`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/rviz/semantic_search_phase4.rviz`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/rviz/semantic_search_phase4b.rviz`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/rviz/semantic_search_phase5a.rviz`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py`

**Interfaces:**
- Phase 4A includes `bunker_pro2/description.launch.py` exactly once.
- RViz consumes transient-local `/robot_description`.

- [ ] Add failing tests for description composition, package dependency, and RobotModel displays in all three RViz files.
- [ ] Run Phase 4A/4B/5A contract tests and confirm failure.
- [ ] Include the description and add one RobotModel block per RViz configuration without altering existing displays.
- [ ] Re-run all focused tests and confirm they pass.
- [ ] Commit the independent visualization/composition change.

### Task 4: Regression and runtime verification

**Files:**
- Add: this plan and `docs/superpowers/specs/2026-08-09-phase5-robot-model-tf-design.md`.

**Interfaces:**
- Verifies installed package discovery and runtime TF composition.

- [ ] Build `bunker_pro2`, `bunker_base`, and `track_robot_bringup` with colcon.
- [ ] Run their package tests plus the focused Phase 4A/4B/5A contracts; require zero failures.
- [ ] Launch only the description and simulated Bunker base with `base_frame:=robot_bottom`; verify `odom -> robot_bottom -> base_link` and stop both nodes.
- [ ] Verify the feature branch is clean, commit the design/plan, and fast-forward merge it into local `main` without pushing remote.
- [ ] Re-run focused tests on merged `main` and preserve unrelated worktrees.
