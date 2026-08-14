# Task 5 Report

## RED Evidence

Command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py
```

Result: `3 failed in 0.14s`.

- `human_following_supervised_test.yaml` was missing (`FileNotFoundError`).
- `human_following_shadow.yaml` was missing (`FileNotFoundError`).
- `outdoor_follow_decision.launch.py` did not declare `profile_config`.

The initial `pytest` executable was a Python 2 launcher and failed to import
`pathlib`; all recorded Python test commands use `python3 -m pytest`.

## GREEN Evidence

Command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py
```

Result: `3 passed in 0.06s`.

Command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py
```

Result: `24 passed, 6 warnings in 0.56s`. The warnings are existing ROS Foxy
and `pkg_resources` deprecations.

Command:

```bash
cmake -S track_robot_ws/src/track_robot/track_robot_bringup \
  -B /tmp/task5-track_robot_bringup-build -DBUILD_TESTING=ON
ctest --test-dir /tmp/task5-track_robot_bringup-build \
  -R 'test_human_following_profiles|test_launch_contract' --output-on-failure
```

Result: CMake configured with ROS Foxy and CTest passed `2/2` registered
pytest targets.

The worktree-local colcon command was also attempted:

```bash
colcon test --base-paths track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --test-result-base track_robot_ws/test_results \
  --packages-select track_robot_bringup \
  --ctest-args -R 'test_human_following_profiles|test_launch_contract' \
  --output-on-failure --event-handlers console_direct+
```

It exited before CMake/test execution because the linked worktree install
overlay references missing dependency hooks, including `cob_srvs`, `fast_lio`,
`track_robot_core`, and `track_robot_control`. The isolated CMake/CTest run
above verifies the newly registered targets without that stale overlay.

## Important Review Fix

The review found that the controller launch always placed default-filled
launch arguments after `profile_config`. Because later ROS parameter sources
win, standalone defaults replaced the profile's required follow distance,
speed, and acceleration limits.

### Baseline

Command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py
```

Result before adding the review regression tests: `24 passed, 6 warnings in
0.57s`.

### RED Evidence

Command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py
```

Result: `3 failed, 4 passed, 6 warnings in 0.52s`.

- Profile-only controller parameters resolved to standalone defaults: `1.8`,
  `0.15`, `0.35`, `0.10`, and `0.25`, rather than the profile values.
- Supplying only `max_linear_x:=0.08` still allowed default
  `follow_distance:=1.8` to replace the profile's `2.0`.
- Each launch had only the node-building `OpaqueFunction`; none captured
  explicit arguments before declarations populated defaults.

### GREEN Evidence

Focused command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py
```

Result: `7 passed, 6 warnings in 0.47s`.

Focused regression command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py
```

Result: `28 passed, 6 warnings in 0.63s`. The warnings are the existing ROS
Foxy and `pkg_resources` deprecations.

Registered-test command:

```bash
cmake -S track_robot_ws/src/track_robot/track_robot_bringup \
  -B /tmp/task5-review-track_robot_bringup-build -DBUILD_TESTING=ON
ctest --test-dir /tmp/task5-review-track_robot_bringup-build \
  -R 'test_human_following_profiles|test_launch_contract' --output-on-failure
```

Result: CMake configured successfully and CTest passed `2/2` registered
pytest targets. An AST comparison against `9909795` also confirmed unchanged
public argument names and defaults in all three launch files.

### Review-Fix Changed Files

- `track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py`
- `track_robot_ws/src/track_robot/track_robot_control/launch/target_follow_controller.launch.py`
- `track_robot_ws/src/track_robot/track_robot_decision/launch/outdoor_follow_decision.launch.py`
- `track_robot_ws/src/track_robot/track_robot_safety/launch/motion_safety.launch.py`
- `.superpowers/sdd/task-5-report.md`

## Changed Files

- `track_robot_ws/src/track_robot/track_robot_bringup/CMakeLists.txt`
- `track_robot_ws/src/track_robot/track_robot_bringup/config/human_following_shadow.yaml`
- `track_robot_ws/src/track_robot/track_robot_bringup/config/human_following_supervised_test.yaml`
- `track_robot_ws/src/track_robot/track_robot_bringup/test/test_human_following_profiles.py`
- `track_robot_ws/src/track_robot/track_robot_control/launch/target_follow_controller.launch.py`
- `track_robot_ws/src/track_robot/track_robot_decision/launch/outdoor_follow_decision.launch.py`
- `track_robot_ws/src/track_robot/track_robot_safety/launch/motion_safety.launch.py`
- `.superpowers/sdd/task-5-report.md`
