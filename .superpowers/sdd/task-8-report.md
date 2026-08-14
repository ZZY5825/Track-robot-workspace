# Task 8 Report

## Scope

Implemented independent one-command human-following operation, bounded
readiness, verified process ownership, fail-closed ordered shutdown, and a
zero-then-shutdown service for `cmd_vel_gate`.

The human-following controller uses ROS Domain 20 and state path
`~/.ros/track_robot_human_following/managed_process.json`. It reuses the
semantic-search hardware selection, managed environment, bounded subprocess,
and process identity helpers without sharing feature state or changing
`semantic_search_ctl` behavior.

## TDD Evidence

Baseline semantic-search CLI, readiness, and process tests:

```bash
env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_control_cli.py test/test_readiness.py test/test_process_control.py
```

Result: `122 passed in 0.77s`.

Initial RED for the new Python boundaries:

```bash
env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_human_following_cli.py test/test_human_following_readiness.py
```

Result: collection stopped with two expected import errors because
`human_following_cli` and `human_following_readiness` did not exist.

Install registration RED:

```bash
env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_human_following_cli.py test/test_human_following_readiness.py
```

Result: `1 failed, 36 passed`; the failure was the missing
`scripts/human_following_ctl` executable.

Gate RED, run outside the filesystem/network sandbox so DDS could inspect
local interfaces:

```bash
colcon test --packages-select track_robot_core --event-handlers console_direct+ --return-code-on-test-failure
```

Result: `1 failed`; the test delivered a nonzero command and then timed out
waiting for the missing `/cmd_vel_gate/shutdown` service.

Publisher-ownership parser RED:

```bash
env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_human_following_readiness.py::test_active_does_not_mistake_cmd_vel_gate_subscriber_for_publisher
```

Result: `1 failed`; active readiness incorrectly accepted a subscriber entry
named `cmd_vel_gate` after an independently owned publisher. The parser now
limits ownership matching to the verbose publisher section.

Latest-zero shutdown observation RED:

```bash
env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_human_following_cli.py::test_zero_observation_uses_latest_complete_twist
```

Result: `1 failed`; zero observation accepted an earlier zero command followed
by a newer nonzero command. Shutdown now evaluates the latest complete Twist.

## Final Verification

Focused gate/CLI/readiness/process and existing semantic-search regressions:

```bash
env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q test/test_human_following_cli.py test/test_human_following_readiness.py test/test_process_control.py test/test_control_cli.py test/test_readiness.py
```

Result: `161 passed in 1.13s`.

Core build:

```bash
colcon build --packages-select track_robot_core --event-handlers console_direct+
```

Result: `1 package finished`; `cmd_vel_gate` and `lidar_sub` built.

Gate shutdown launch test, isolated on ROS Domain 229:

```bash
colcon test --packages-select track_robot_core --event-handlers console_direct+ --return-code-on-test-failure
```

Result: `1/1 passed`, `0 tests failed`. The test launches only
`cmd_vel_gate`; it does not launch Bunker, CAN, camera, LiDAR, or IMU hardware.
It proves nonzero input is followed by a zero `/cmd_vel` message and then zero
`/cmd_vel` publishers.

Additional source checks:

- `python3 -m flake8` on all new Python production/test files: clean.
- `python3 -m py_compile` on all new Python production/test files: clean.
- `xmllint --noout` on both modified package manifests: clean.
- Standalone bringup CMake configure/build/install under `/tmp`: successful;
  both Python modules and `human_following_ctl` were installed.
- `git diff --check`: clean before staging; the staged check is recorded at
  commit time.

## Environment Boundary

`colcon build --packages-select track_robot_bringup track_robot_core` built
`track_robot_core` but could not start the bringup build because this partial
worktree lacks existing installed `package.sh` files for dependencies including
`bunker_base`, `zed_wrapper`, `track_robot_control`,
`track_robot_lidar_tracking`, and `track_robot_perception`. A clean standalone
bringup CMake configure/build/install was run against `/opt/ros/foxy` to verify
the Task 8 CMake and install rules without those unrelated workspace artifacts.

## Safety Contract

- Shadow is the default and expects zero `/cmd_vel` publishers.
- Active mode requires `--confirm-motion` and exactly one `/cmd_vel` publisher
  owned by `/cmd_vel_gate`.
- Auto hardware starts only missing modules; external hardware starts none.
- Readiness requires exact publishers, fresh messages, healthy CAN/base status,
  required TF and Trigger services, and correct command ownership.
- Stop orders disarm, zero observation, emergency-stop escalation, gate
  shutdown, zero publisher proof, and only then verified process-group cleanup.
- Stale or unverified ownership state never receives a signal.
- Externally owned hardware is never placed in the managed process group.
