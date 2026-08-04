# Phase 4B Zero-Inflation Footprint Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make active Phase 4B navigation use a 0.88 m by 0.80 m physical footprint with no additional inflation.

**Architecture:** Keep Nav2 obstacle layers, the safety supervisor, and the velocity gate intact. Change only the active Phase 4B geometry parameters and protect their consistency with contract tests.

**Tech Stack:** ROS 2 Foxy, Nav2 costmap YAML, track_robot_safety YAML, pytest, colcon.

## Global Constraints

- Physical footprint is exactly `0.88 m x 0.80 m`.
- Nav2 and safety inflation are exactly `0.0 m`.
- Nav2 footprint padding is exactly `0.0 m`, and both inflation layers are disabled.
- Collision checking and the existing command safety chain remain enabled.
- Existing public topics and interfaces do not change.

---

### Task 1: Freeze the Phase 4B geometry contract

**Files:**
- Modify: `src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py`
- Modify: `src/track_robot/track_robot_safety/test/test_nav2_safety_contract.py`

**Interfaces:**
- Consumes: active Phase 4B YAML parameters.
- Produces: regression assertions for the shared physical footprint and zero inflation.

- [x] **Step 1: Change assertions to require the new footprint**

Require Nav2 polygon `x=+/-0.44`, `y=+/-0.40`, zero footprint padding, disabled inflation layers, safety length `0.88`, width `0.80`, and all active inflation values `0.0`.

- [x] **Step 2: Run tests to verify the old configs fail**

Run: `pytest -q src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py src/track_robot/track_robot_safety/test/test_nav2_safety_contract.py`

Expected: geometry assertions fail against the old `1.20 x 1.00 m` and non-zero inflation values.

### Task 2: Apply the unified geometry

**Files:**
- Modify: `src/track_robot/track_robot_navigation/config/nav2_phase4b.yaml`
- Modify: `src/track_robot/track_robot_safety/config/local_obstacle_map.yaml`
- Modify: `src/track_robot/track_robot_safety/config/motion_safety_supervisor_nav2.yaml`

**Interfaces:**
- Consumes: `footprint`, `footprint_length`, `footprint_width`, and `safety_inflation` ROS parameters.
- Produces: consistent collision geometry for Nav2 planning and safety supervision.

- [x] **Step 1: Set both Nav2 costmaps to the rectangular polygon**

Use `[[-0.44,-0.40],[-0.44,0.40],[0.44,0.40],[0.44,-0.40]]`, matching existing YAML string style. Set `footprint_padding: 0.0`, `inflation_layer.enabled: false`, and `inflation_radius: 0.0` in both costmaps.

- [x] **Step 2: Set both active safety configs**

Set `footprint_length: 0.88`, `footprint_width: 0.80`, and `safety_inflation: 0.0`. Keep `self_filter_margin: 0.03` unchanged.

- [x] **Step 3: Run focused contract tests**

Run: `pytest -q src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py src/track_robot/track_robot_safety/test/test_nav2_safety_contract.py`

Expected: all tests pass.

- [x] **Step 4: Run affected package tests and build**

Run: `colcon test --packages-select track_robot_navigation track_robot_safety --event-handlers console_cohesion+` followed by `colcon test-result --verbose`.

Run: `colcon build --symlink-install --packages-select track_robot_safety track_robot_navigation track_robot_bringup --event-handlers console_cohesion+`.

Expected: no test failures and all selected packages build successfully.

- [x] **Step 5: Inspect installed configuration values**

Search the installed YAML files for the new polygon, dimensions, and zero inflation. Expected: source and installed values agree.
