# Task 6 Report

## RED Evidence

Command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_shared_hardware_launch_contract.py
```

Result: `6 failed in 0.11s`.

- `track_robot_hardware.launch.py` was missing.
- `semantic_search_sensors.launch.py` still had three direct includes.
- `semantic_search_platform.launch.py` still had two direct includes.
- Neither compatibility wrapper included the neutral hardware launch.

## GREEN Evidence

Focused command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_shared_hardware_launch_contract.py
```

Result: `6 passed in 0.04s`.

All bringup launch-contract tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_shared_hardware_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py
```

Result: `44 passed, 6 warnings in 0.70s`. The warnings are existing ROS
Foxy and `pkg_resources` deprecations.

Registered-test commands:

```bash
cmake -S track_robot_ws/src/track_robot/track_robot_bringup \
  -B /tmp/task6-track-robot-bringup-build-20260814 -DBUILD_TESTING=ON
ctest --test-dir /tmp/task6-track-robot-bringup-build-20260814 \
  -R launch_contract --output-on-failure
```

Result: CMake configured successfully and CTest passed `5/5` registered
launch-contract targets.

Syntax command:

```bash
python3 -m py_compile \
  track_robot_ws/src/track_robot/track_robot_bringup/launch/track_robot_hardware.launch.py \
  track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_sensors.launch.py \
  track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_platform.launch.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_shared_hardware_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py
```

Result: exit `0`.

Whitespace command:

```bash
git diff --check
```

Result: exit `0` with no output.

## Changed Files

- `.superpowers/sdd/task-6-report.md`
- `track_robot_ws/src/track_robot/track_robot_bringup/CMakeLists.txt`
- `track_robot_ws/src/track_robot/track_robot_bringup/launch/track_robot_hardware.launch.py`
- `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_sensors.launch.py`
- `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_platform.launch.py`
- `track_robot_ws/src/track_robot/track_robot_bringup/test/test_shared_hardware_launch_contract.py`
- `track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py`

## Commit

Message: `refactor: share a neutral robot hardware launch`

The exact hash of the commit containing this report is recorded in the final
Task 6 handoff; a Git commit cannot contain its own content-derived hash.
