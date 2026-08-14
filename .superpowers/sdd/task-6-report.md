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

## Review Follow-up: Composed Hardware Topology

The original composed-AST assertion only checked launch-file strings in the
wrappers. It could not prove which includes become active after
`OpaqueFunction` expansion, argument forwarding, and `IfCondition`
evaluation.

`test_shared_hardware_launch_contract.py` now loads the local launch entry
points, executes their `OpaqueFunction` actions in a `LaunchContext`, resolves
forwarded include arguments recursively, and evaluates every physical-driver
condition. It verifies the active driver topology for:

- `semantic_search_live.launch.py` at its Phase 1 default and `sensors` stage;
- `semantic_search_phase4a.launch.py` defaults;
- `semantic_search_phase4b.launch.py` and `semantic_search_phase5a.launch.py`
  at defaults and with their forwarded `start_base:=true` override.

The test requires one active instance for each enabled physical module and
zero for modules explicitly disabled by the entry point. It also evaluates the
neutral hardware launch's `PythonExpression` values directly: description
ownership forces camera `extrinsic_mode=robot_description` and
`publish_base_lidar_tf=false`; with description disabled, both values forward
their supplied standalone settings.

### Review RED Evidence

After adding the recursive contract, a temporary mutation changed the LiDAR
include condition to `start_camera`. The focused topology command failed as
expected because the default Phase 1 live launch incorrectly activated LiDAR:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_shared_hardware_launch_contract.py \
  -k composed_entrypoints
```

Result: `1 failed, 6 deselected in 0.47s`. The production launch condition was
then restored to `start_lidar`.

### Review GREEN Evidence

Affected bringup launch-contract tests:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_shared_hardware_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py
```

Result: `45 passed, 6 warnings in 0.76s`. The warnings are the existing ROS
Foxy and `pkg_resources` deprecations.

Review follow-up files:

- `.superpowers/sdd/task-6-report.md`
- `track_robot_ws/src/track_robot/track_robot_bringup/test/test_shared_hardware_launch_contract.py`

## Commit

Original Task 6 implementation: `1dc94ede111debab780cd934d93903cf4dad211d`
(`refactor: share a neutral robot hardware launch`).

Review follow-up message: `test: verify composed hardware topology`.

The exact follow-up hash is recorded in the final Task 6 handoff; a Git commit
cannot contain its own content-derived hash.
