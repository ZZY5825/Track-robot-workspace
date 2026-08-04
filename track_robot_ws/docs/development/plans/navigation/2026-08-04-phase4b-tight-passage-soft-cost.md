# Phase 4B Tight-Passage Soft-Cost Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Lower traversable Nav2 inflation costs so the Bunker can plan through a measured `1.0 m` passage while retaining its physical collision boundary.

**Architecture:** Keep the `0.88 m x 0.80 m` footprint, `0.60 m` inflation radii, and safety chain unchanged. Synchronize the local costmap, global costmap, and regulated pure-pursuit inflation cost scaling at `12.0` so soft costs decay faster outside the inscribed footprint.

**Tech Stack:** ROS 2 Foxy, Nav2 InflationLayer, RegulatedPurePursuitController, YAML, pytest, colcon.

## Global Constraints

- Physical footprint remains `0.88 m x 0.80 m`.
- Both Nav2 inflation radii remain `0.60 m`.
- Footprint padding remains `0.0 m`.
- Do not change velocity limits, topics, safety supervisor, velocity gate, or executable-motion authorization.
- Runtime validation starts in planning-only mode and uses a measured passage no narrower than `1.0 m`.

---

### Task 1: Synchronize the tighter soft-cost decay

**Files:**
- Modify: `src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py`
- Modify: `src/track_robot/track_robot_navigation/config/nav2_phase4b.yaml`

**Interfaces:**
- Consumes: `controller_server.ros__parameters.FollowPath.inflation_cost_scaling_factor` and each costmap's `inflation_layer.cost_scaling_factor`.
- Produces: three synchronized values of `12.0`, with unchanged footprint and inflation radius.

- [x] **Step 1: Write the failing configuration-contract test**

Change the existing assertions to:

```python
assert controller['inflation_cost_scaling_factor'] == 12.0
assert params['inflation_layer']['inflation_radius'] == 0.60
assert params['inflation_layer']['cost_scaling_factor'] == 12.0
```

- [x] **Step 2: Run the focused test and verify RED**

Run:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py -q
```

Expected: two assertions fail because the runtime configuration still contains `7.0`.

- [x] **Step 3: Implement the minimal YAML change**

Set only these values in `nav2_phase4b.yaml`:

```yaml
inflation_cost_scaling_factor: 12.0
cost_scaling_factor: 12.0  # local costmap
cost_scaling_factor: 12.0  # global costmap
```

- [x] **Step 4: Run focused and full regression tests**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_navigation/test \
  src/track_robot/track_robot_safety/test -q
```

Expected: all tests pass.

- [x] **Step 5: Build and inspect the effective configuration**

Run:

```bash
colcon build --packages-select track_robot_navigation --symlink-install
```

Expected: the package builds successfully and all three scaling values are `12.0`; both radii remain `0.60`.
