# Task 7 Report

## Scope

Implemented the dedicated human-following live launch, fail-closed compatibility
wrapper, RViz configuration, expansion-based topology contract, and CMake test
registration.

## TDD Evidence

Baseline:

```bash
source /opt/ros/foxy/setup.bash && source track_robot_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py track_robot_ws/src/track_robot/track_robot_bringup/test/test_shared_hardware_launch_contract.py
```

Result: `14 passed, 6 warnings in 0.60s`.

RED, after creating the topology contract and before implementation:

```bash
source /opt/ros/foxy/setup.bash && source track_robot_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_launch_contract.py
```

Result: `12 failed, 6 warnings in 0.72s`. Failures were caused by the missing
live launch and RViz files plus the legacy wrapper's missing fail-closed
contract.

GREEN, after implementation and source-tree config-path isolation for the
selective test environment:

```bash
source /opt/ros/foxy/setup.bash && source track_robot_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_launch_contract.py
```

Result: `12 passed, 6 warnings in 0.52s`.

The contract expands launch actions and verifies five feature includes, zero
gate nodes in shadow, one gate node in confirmed active mode, supervisor and
RViz action counts, profile propagation, command topics, wrapper delegation,
and semantic/navigation feature absence.

## Review Findings Closure

Adapted the deterministic recursive expansion pattern from
`test_shared_hardware_launch_contract.py` so the Task 7 contract now walks
repository-local includes across package boundaries, evaluates action
conditions and scoped groups, executes opaque launch setup, and resolves the
effective YAML-plus-inline parameters for every leaf node.

The recursive contract covers both `human_following_live.launch.py` and the
legacy `safe_human_following.launch.py` entry point and proves:

- the complete 12-node human-following feature graph in shadow mode;
- zero `cmd_vel_gate` nodes in shadow and exactly one in confirmed active mode;
- active-without-confirmation rejection through the legacy wrapper;
- `enable_cmd_vel == false` at the effective controller node;
- the final `/follow/cmd_vel_planned` to `/follow/cmd_vel_avoiding` to
  `/follow/cmd_vel_safe` to `/cmd_vel` command chain;
- effective supervised-profile limits at decision, controller, planner,
  safety, gate, and supervisor leaf nodes; and
- no active semantic-search or navigation include/node identity anywhere in
  either recursively expanded graph.

RED, before the recursive expander existed:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_launch_contract.py::test_shadow_expansion_has_full_feature_topology_and_zero_gate_actions
```

Result: `1 failed, 6 warnings in 0.47s`; the expected failure was
`NameError: name '_recursive_launch_graph' is not defined`.

Focused GREEN:

```bash
source /opt/ros/foxy/setup.bash && source track_robot_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_launch_contract.py
```

Result: `20 passed, 6 warnings in 0.83s`.

All affected bringup launch/profile contracts:

```bash
source /opt/ros/foxy/setup.bash && source track_robot_ws/install/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py track_robot_ws/src/track_robot/track_robot_bringup/test/test_shared_hardware_launch_contract.py track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_no_motion_contract.py track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_launch_contract.py
```

Result: `74 passed, 6 warnings in 1.30s`.

## Verification Boundary

Registered CTests could not be run with the new Task 7 registrations. The
existing `CTestTestfile.cmake` predates `c7365cf` and lists 11 tests, omitting
the shared-hardware, profile, and human-following registrations. Reconfiguring
through `colcon build --packages-select track_robot_bringup` is blocked before
CMake because this partial worktree lacks installed `package.sh` files for 15
declared dependencies, including `track_robot_core`, `track_robot_control`,
`track_robot_lidar_tracking`, and `track_robot_perception`. No production file
was changed because recursive expansion exposed no production launch defect.
