# Phase 4B Soft Inflation Reduction Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reduce the Nav2 soft obstacle-cost influence by approximately 30% without shrinking the Bunker's physical collision reserve.

**Architecture:** Keep both costmaps' `inflation_radius` at `0.60 m`, which covers the `0.88 x 0.80 m` rectangular footprint corners. Increase the exponential cost decay from `5.0` to `7.0` in both costmaps and keep the regulated pure-pursuit controller's matching inflation parameter synchronized.

**Tech Stack:** ROS 2 Foxy, Nav2 InflationLayer, RegulatedPurePursuitController, YAML, pytest.

## Global Constraints

- Preserve the `0.88 x 0.80 m` footprint and `0.60 m` physical collision reserve.
- Change no ROS interfaces, topics, safety gates, velocity limits, or recovery behavior.
- Keep all runtime nodes stopped while modifying and testing configuration.

---

### Task 1: Narrow the soft cost gradient

**Files:**
- Modify: `src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py`
- Modify: `src/track_robot/track_robot_navigation/config/nav2_phase4b.yaml`

**Interfaces:**
- Consumes: Nav2 costmap and controller YAML parameters.
- Produces: synchronized `cost_scaling_factor = 7.0` configuration with unchanged collision radius.

- [x] **Step 1: Write the failing configuration-contract test**

Assert that each costmap uses `cost_scaling_factor == 7.0` and that `FollowPath.inflation_cost_scaling_factor == 7.0`.

- [x] **Step 2: Run the focused test and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py -q
```

Expected: failure showing the existing value `5.0`.

- [x] **Step 3: Apply the minimal configuration change**

Set the two costmap factors and the matching controller factor to `7.0`; leave both inflation radii at `0.60`.

- [x] **Step 4: Run focused and full regression tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/track_robot/track_robot_navigation/test src/track_robot/track_robot_safety/test -q
```

Expected: all tests pass.

- [x] **Step 5: Build the affected package**

Run:

```bash
colcon build --packages-select track_robot_navigation --symlink-install
```

Expected: `track_robot_navigation` builds successfully.
