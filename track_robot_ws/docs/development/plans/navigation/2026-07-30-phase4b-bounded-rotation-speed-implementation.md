# Phase 4B Bounded Rotation and Speed Update Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use
> superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use
> checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace unlimited pure-rotation collision checking, raise supervised
Phase 4B speeds, reduce inflation to 65%, and preserve active navigation across
short dynamic-obstacle stops.

**Architecture:** A small header-only rotation collision model provides
deterministic, ROS-independent unit tests and is called by the existing motion
safety supervisor. Existing Nav2, safety-supervisor and final-gate boundaries
remain intact. A semantic-policy flag distinguishes an armed transient BLOCKED
state from safety failures that must cancel.

**Tech Stack:** ROS 2 Foxy, C++17, ament_cmake_gtest, Python 3, pytest, Nav2
Regulated Pure Pursuit.

## Global Constraints

- Preserve public topics, messages, services, actions and TF frames.
- Keep the physical footprint at 1.20 m by 1.00 m.
- Route every executable velocity through the motion safety supervisor and
  `cmd_vel_gate`.
- Use 0.15 m/s linear and 0.50 rad/s angular hard limits.
- Use 0.13 m safety inflation and 0.1625 m Nav2 costmap inflation.
- Allow automatic resume only for an armed `STATE_BLOCKED`.
- RC override, E-stop, stale input, base fault and invalid semantic references
  remain cancel/disarm conditions.
- Use one logically independent implementation commit per task.

---

### Task 1: Bounded pure-rotation collision model

**Files:**
- Create: `src/track_robot/track_robot_safety/include/track_robot_safety/rotation_collision.hpp`
- Create: `src/track_robot/track_robot_safety/test/test_rotation_collision.cpp`
- Modify: `src/track_robot/track_robot_safety/src/motion_safety_supervisor_node.cpp`
- Modify: `src/track_robot/track_robot_safety/CMakeLists.txt`
- Modify: `src/track_robot/track_robot_safety/package.xml`
- Modify: `src/track_robot/track_robot_safety/config/motion_safety_supervisor_nav2.yaml`

**Interfaces:**
- Produces:
  `track_robot_safety::rotationStopAngle(double, double, double, double)`
  returning radians.
- Produces:
  `track_robot_safety::evaluateRotationCollision(...)` returning collision,
  collision angle and time-to-collision without ROS dependencies.
- Consumes the existing obstacle point sequence with `x` and `y` members.

- [ ] **Step 1: Add failing deterministic unit tests**

Test zero speed, stop-angle growth, the measured `-0.026 rad/s` clear case,
and a point inside the finite swept footprint. Use:

```cpp
EXPECT_NEAR(rotationStopAngle(-0.026, 0.80, 0.25, 0.05), 0.0569225, 1e-6);
EXPECT_FALSE(evaluateRotationCollision(
  clear_points, -0.026, 0.73, 0.63, 0.80, 0.25, 0.05, 0.005).collision);
EXPECT_TRUE(evaluateRotationCollision(
  collision_points, -0.40, 0.73, 0.63, 0.80, 0.25, 0.05, 0.005).collision);
```

- [ ] **Step 2: Verify RED**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon test --packages-select track_robot_safety \
  --ctest-args -R test_rotation_collision --output-on-failure
```

Expected: build/test failure because the header and functions do not exist.

- [ ] **Step 3: Implement the minimal pure model and node integration**

Implement:

```cpp
stop_angle =
  omega * omega / (2.0 * angular_braking_deceleration) +
  std::abs(omega) * response_latency +
  fixed_rotation_margin;
```

Sample signed rotation angles at no more than 0.005 rad. Transform each static
obstacle into the rotating robot frame and test the inflated rectangle. Replace
only the existing full-circumscribed-circle pure-rotation branch. Add
`angular_braking_deceleration` and `fixed_rotation_margin` parameters.

- [ ] **Step 4: Verify GREEN**

Run the targeted gtest, the safety pytest contract and build
`track_robot_safety`. Expected: all pass, zero compiler warnings introduced.

- [ ] **Step 5: Commit**

Commit only the Task 1 files with:

```text
fix(safety): bound pure rotation collision sweep
```

---

### Task 2: Consistent speed and inflation envelope

**Files:**
- Modify: `src/track_robot/track_robot_navigation/config/nav2_phase4b.yaml`
- Modify: `src/track_robot/track_robot_navigation/config/cmd_vel_gate_nav2.yaml`
- Modify: `src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py`
- Modify: `src/track_robot/track_robot_safety/config/motion_safety_supervisor_nav2.yaml`
- Modify: `src/track_robot/track_robot_safety/config/local_obstacle_map.yaml`
- Modify: `src/track_robot/track_robot_safety/test/test_nav2_safety_contract.py`

**Interfaces:**
- Nav2 emits at most 0.15 m/s and 0.40 rad/s nominal rotation.
- Safety and final gate independently cap at 0.15 m/s and 0.50 rad/s.
- Physical footprint remains unchanged.

- [ ] **Step 1: Change contract tests first**

Require exact values:

```python
assert controller['desired_linear_vel'] == 0.15
assert controller['rotate_to_heading_angular_vel'] == 0.40
assert gate['max_linear_x'] == 0.15
assert gate['max_angular_z'] == 0.50
assert safety['safety_inflation'] == 0.13
assert local_inflation['inflation_radius'] == 0.1625
assert global_inflation['inflation_radius'] == 0.1625
```

- [ ] **Step 2: Verify RED**

Run both config test files. Expected: failures showing the old 0.10/0.25 and
0.20/0.25 values.

- [ ] **Step 3: Update the YAML chain**

Set:

```text
desired_linear_vel: 0.15
rotate_to_heading_angular_vel: 0.40
max_linear_accel: 1.50
max_linear_decel: 0.25
max_angular_accel: 0.50
supervisor/gate max_linear_x: 0.15
supervisor/gate max_angular_z: 0.50
safety_inflation: 0.13
costmap inflation_radius: 0.1625
```

- [ ] **Step 4: Verify GREEN and stopping-envelope math**

Run both config suites and assert the configured 0.15 m/s stopping distance is
0.5325 m using the unchanged safety formula.

- [ ] **Step 5: Commit**

Commit only Task 2 configuration and contract files with:

```text
feat(navigation): raise supervised Phase 4B speed envelope
```

---

### Task 3: Dynamic-obstacle hold and bounded resume

**Files:**
- Modify: `src/track_robot/track_robot_navigation/track_robot_navigation/semantic_goal_policy.py`
- Modify: `src/track_robot/track_robot_navigation/track_robot_navigation/semantic_navigation_supervisor_node.py`
- Modify: `src/track_robot/track_robot_navigation/test/test_semantic_goal_policy.py`
- Modify: `src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py`
- Modify: `src/track_robot/track_robot_navigation/config/nav2_phase4b.yaml`

**Interfaces:**
- Add `safety_temporarily_blocked: bool` to `SemanticGoalSnapshot`.
- `STATE_BLOCKED` plus `armed=true` maps to HOLD without clearing dispatched
  state.
- All other non-permitting states retain CANCEL behavior.

- [ ] **Step 1: Add failing policy tests**

Prove:

```python
assert policy.evaluate(clear).action is GoalAction.NAVIGATE
assert policy.evaluate(blocked).action is GoalAction.HOLD
assert policy.evaluate(clear_again).reason == 'goal_already_dispatched'
assert policy.evaluate(rc_disarmed).action is GoalAction.CANCEL
assert policy.evaluate(stale_nonpermitting).action is GoalAction.CANCEL
```

Also require `movement_time_allowance == 30.0`.

- [ ] **Step 2: Verify RED**

Run the policy and Nav2 config tests. Expected: missing snapshot field or
blocked-state cancellation, and old 8-second allowance.

- [ ] **Step 3: Implement HOLD mapping and timeout**

Populate the new flag only for an armed `SafetyState.STATE_BLOCKED`. In active
policy evaluation, check disarmed first, transient block second, then other
non-permitting states. Set Nav2 progress allowance to 30 seconds.

- [ ] **Step 4: Verify GREEN**

Run all navigation tests. Expected: all pass, including existing target-loss,
odometry-stale and safety-loss cancellation cases.

- [ ] **Step 5: Commit**

Commit only Task 3 files with:

```text
feat(navigation): resume after transient obstacle blocks
```

---

### Task 4: Integrated regression and non-motion validation

**Files:**
- Update only if measured behavior requires a correction supported by a new
  failing test.

**Interfaces:**
- Consumes all Task 1-3 behavior.
- Produces build/test evidence and a disarmed runtime result.

- [ ] **Step 1: Build affected packages**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --packages-select track_robot_safety track_robot_navigation \
  track_robot_bringup --symlink-install
```

- [ ] **Step 2: Run affected and aggregate regressions**

Run package CTest/pytest, deterministic semantic replay tests and the existing
Phase 0-4B aggregate suite. Any failure blocks completion.

- [ ] **Step 3: Offline measured-bag comparison**

Use `/tmp/phase4b_manual_motion_0p1_clear_20260730` to verify the former
`-0.026 rad/s` full-circle stop is outside the new bounded sweep while a
constructed nearby collision remains blocked.

- [ ] **Step 4: Disarmed runtime probe**

With ROS Domain 20 and the standard launch, send a short Nav2 goal while the
safety supervisor remains disarmed. Require:

```text
first raw linear.x = 0.15 m/s
max safe command = 0
max final /cmd_vel = 0
```

Stop all nodes after the probe.

- [ ] **Step 5: Final diff and requirement review**

Confirm no public interface changes, no direct Nav2-to-base path and no
unrelated files staged. Report the live walking-person and RC-override motion
tests as pending until explicitly run.
