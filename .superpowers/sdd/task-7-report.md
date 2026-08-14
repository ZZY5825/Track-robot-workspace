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

## Verification Boundary

The user requested an immediate commit after the focused GREEN run. The full
affected bringup CTest/profile matrix was therefore not rerun in this turn.
The pre-change focused profile/shared-hardware baseline and post-change Task 7
contract results above are the available test evidence.
