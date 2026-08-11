# ZED-only semantic 3D stationary live validation — 2026-08-11

## Result

- Software build/regression: **PASS**, with the harness caveats recorded below.
- Stationary `green bottle` live gate: **NOT EVALUATED**.
- Tested source commit: `3f2e4e581d5bb22834f0de7b84d9308b78ed8b0b`.
- ROS domain: `20`.
- Motion authorization: none. No base, approach/finding service, Nav2 execution,
  or velocity publisher was started by this validation.

The live gate was not started because the read-only hardware preflight found no
ZED USB device and found the LiDAR interface `eth0` in `DOWN` state. The test
contract forbids using `sudo`, changing network/CAN state, or starting the base
to work around those blockers. No live values below are inferred from offline
tests.

## Configuration under test

The intended stationary stack and its checked-in configuration are:

- launch: `src/track_robot/track_robot_bringup/launch/semantic_search_phase4a.launch.py`;
- semantic depth, selector, and planning:
  `src/track_robot_semantic_search/config/semantic_search_phase4a.yaml`;
- ZED-only semantic memory profile:
  `src/track_robot/track_robot_semantic_memory/config/phase4a_test.yaml`;
- obstacle grid:
  `src/track_robot/track_robot_safety/config/local_obstacle_map.yaml`;
- LiDAR sensor profile:
  `src/track_robot/track_robot_sensor_bringup/config/rslidar_track_robot.yaml`;
- YOLO-World checkpoint:
  `/home/track-robot/track_robot_ws/models/r0c/yolov8s-worldv2.pt`;
- YOLO runtime:
  `/home/track-robot/track_robot_ws/models/r0c_runtime/python`;
- CLIP checkpoint:
  `/home/track-robot/track_robot_ws/models/phase1/ViT-B-32.pt`.

The three model/runtime paths above were present during preflight. Phase 4A's
launch contract fixes `start_base=false`, `start_imu=false`, and planning-only
behavior. The profile fixes the semantic 3D path to ZED registered depth ->
`semantic_depth_enricher` -> `/semantic_memory/spatial_observations` ->
semantic memory. Direct LiDAR memory updates and semantic LiDAR attachment are
disabled; LiDAR is reserved for the obstacle grid and later motion safety.

## Software verification

All commands ran from:

```text
/home/track-robot/track_robot_ws/.worktrees/main-integration/track_robot_ws
```

### Affected Python regression

The brief's exact command was run first:

```bash
source /opt/ros/foxy/setup.bash
python3 -m pytest \
  src/track_robot_semantic_search/test \
  src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py -q
```

Initial result: exit `2`, before collection, because the fresh worktree had not
yet installed `track_robot_semantic_search` (`ModuleNotFoundError`). After the
three-package build and `source install/setup.bash`, the same combined file set
again exited `2`, before running assertions, because pytest 4.6 imported both
different `test_launch_contract.py` files as top-level module
`test_launch_contract` (`ImportMismatchError`). This is a test invocation/harness
collision, not a failed test assertion.

The identical affected file set was therefore executed as three isolated pytest
invocations. Third-party plugin autoload was disabled for the two static launch
contract invocations because the user-local vendored `typeguard` plugin is
incompatible with system pytest 4.6 and otherwise aborts during option parsing.

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
python3 -m pytest src/track_robot_semantic_search/test -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  src/track_robot/track_robot_semantic_memory/test/test_launch_contract.py -q
```

Measured results:

- semantic search: `884 passed in 8.31 seconds`;
- Phase 4A bringup launch contract: `6 passed in 0.05 seconds`;
- semantic memory launch contract: `13 passed in 0.12 seconds`;
- total affected assertions: `903 passed`, `0 failed`.

### Three-package build

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select \
  track_robot_semantic_search \
  track_robot_semantic_memory \
  track_robot_bringup
```

Result: exit `0`; `3 packages finished`. `track_robot_semantic_memory` emitted
existing CMake stderr warnings: compatibility with CMake `< 3.10` is deprecated
in project/dependency export files, and developer warning `CMP0148` is not set.
No package failed to compile.

### Package suites and result inspection

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon test --packages-select \
  track_robot_semantic_search \
  track_robot_semantic_memory \
  track_robot_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
```

Result: both commands exited `0`; `Summary: 1574 tests, 0 errors, 0 failures,
4 skipped`.

A fresh pre-commit rerun after the documentation changes also exited `0`:
three-package build `3 packages finished`, affected pytest splits `884 passed in
8.67 seconds`, `6 passed in 0.05 seconds`, and `13 passed in 0.11 seconds`, then
`colcon test-result --verbose` again reported `1574 tests, 0 errors, 0 failures,
4 skipped`.

All four existing skips are in
`track_robot_semantic_memory/test/test_ros_runtime.py` and carry the same reason:
`requires local DDS interface access; set RUN_ROS_RUNTIME_TESTS=1`.

- `test_stage2b_nodes_publish_memory_events_and_bounded_markers`;
- `test_duplicate_lidar_ids_do_not_terminate_semantic_memory`;
- `test_stage2e_leave_and_reentry_preserves_one_global_identity`;
- `test_stage2f_task_services_inspection_and_reset_are_epoch_safe`.

## Live preflight evidence

Preflight ended at `2026-08-11T22:12:43+01:00`. A 30–60 second target capture
did not start, so capture start/end and duration are **NOT MEASURED**.

The first sandboxed preflight could not inspect real interfaces: `ip` returned
`Cannot open netlink socket: Operation not permitted`, `lsusb` returned
`unable to initialize libusb: -99`, and `ros2 topic list` raised
`PermissionError: [Errno 1] Operation not permitted` while opening a socket.
These errors were treated only as sandbox restrictions, not hardware evidence.

The same read-only checks were then run outside the sandbox:

```bash
ps -eo pid,ppid,stat,cmd
ip -brief addr
lsusb
test -f /home/track-robot/track_robot_ws/models/r0c/yolov8s-worldv2.pt
test -d /home/track-robot/track_robot_ws/models/r0c_runtime/python
test -f /home/track-robot/track_robot_ws/models/phase1/ViT-B-32.pt
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
timeout 5s ros2 topic list
```

Measured raw outcomes:

- no ROS/RViz/ZED/LiDAR/semantic/Bunker stack was running;
- `eth0 DOWN 192.168.1.102/24`;
- `can0 UP` was observed but not read, changed, or used;
- USB devices listed only Realtek hubs, an OpenMoko hub, IMC Bluetooth, and
  Linux root hubs; no ZED vendor ID (`2b03`) was present;
- ROS Domain 20 listed only `/parameter_events` and `/rosout`;
- all three required model/runtime path checks succeeded.

Because both required sensors failed preconditions, starting the canonical
launch could not produce ZED depth, spatial observations, LiDAR obstacle data,
or a legitimate green-bottle sample. The launch and query commands below were
therefore **NOT RUN**:

```bash
cd /home/track-robot/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export TRACK_ROBOT_WS=/home/track-robot/track_robot_ws
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
unset FASTRTPS_DEFAULT_PROFILES_FILE
ros2 launch track_robot_bringup semantic_search_phase4a.launch.py \
  configure_network:=false \
  start_rviz:=true

ros2 run track_robot_semantic_search semantic_search_query \
  "green bottle" \
  --query-id 2026081101 \
  --query-version 1 \
  --timeout 20 \
  --subscriber-timeout 10
```

No `start_approach` or `start_finding` request was made. No message was
published to `/cmd_vel` or any Nav2 velocity topic.

## Requested 30–60 second measurements

| Evidence | Measured value |
| --- | --- |
| Bottle measured fixed distance | NOT MEASURED |
| Capture start/end | NOT MEASURED |
| Capture duration | NOT MEASURED |
| `/zed/zed_node/depth/depth_registered` rate | NOT MEASURED |
| `/semantic_memory/spatial_observations` rate | NOT MEASURED |
| Selected `global_object_id` | NOT MEASURED |
| Position samples | NOT MEASURED |
| ZED depth samples corresponding to position changes | NOT MEASURED |
| Dropouts | NOT MEASURED |
| `/rslidar_points` rate | NOT MEASURED |
| `/safety/local_obstacle_grid` rate | NOT MEASURED |
| Obstacle-map liveness | NOT MEASURED |
| `/semantic_memory/diagnostic_ranking` | NOT MEASURED |
| `/semantic_search/phase4a/selected_target` | NOT MEASURED |

Diagnostic counters on
`/semantic_search/spatial_observation_diagnostics` were **NOT MEASURED**:

| Counter/value | Measured value |
| --- | --- |
| `latest_reason` | NOT MEASURED |
| `depth_delta_ms` | NOT MEASURED |
| `valid_depth_samples` | NOT MEASURED |
| `depth_quality` | NOT MEASURED |
| `matched_depth` | NOT MEASURED |
| `no_matching_depth` | NOT MEASURED |
| `depth_delta_exceeded` | NOT MEASURED |
| `insufficient_depth_samples` | NOT MEASURED |
| `depth_out_of_range` | NOT MEASURED |
| `tf_unavailable` | NOT MEASURED |
| `invalid_transformed_position` | NOT MEASURED |

## Acceptance gates

| Gate | Status | Evidence/reason |
| --- | --- | --- |
| No valid position is non-finite or exactly `0 m` | NOT EVALUATED | No live position sample. |
| No unexplained multi-metre one-frame jump | NOT EVALUATED | No paired position/depth samples. |
| Every 3D rejection has a diagnostic reason | NOT EVALUATED | No live diagnostics. |
| One camera track keeps one semantic global ID through short depth-only dropouts | NOT EVALUATED | No live track/dropout interval. |
| `/rslidar_points` remains live without semantic LiDAR tracklets | NOT EVALUATED | `eth0` was down; LiDAR rate NOT MEASURED. |
| `/safety/local_obstacle_grid` remains live at a measured rate | NOT EVALUATED | `eth0` was down; obstacle-grid rate NOT MEASURED. |
| No executable motion is published during live validation | NOT EVALUATED | Live validation did not start; preflight itself started no motion process. |

Overall stationary live gate: **NOT EVALUATED**. Nav2 planner tuning remains
deferred.

## Process cleanup evidence

No canonical launch process was started, so there were no test-owned children
to signal. The preflight process listing contained no ROS, RViz, ZED, LiDAR,
semantic-search, semantic-memory, Bunker, Nav2, `cmd_vel_gate`, or motion safety
process. At `2026-08-11T22:16:30+01:00`, a final host-side `ros2 node list` on
Domain 20 returned no node names. A process-name filter returned no matching ROS
process; a broad substring filter produced only two unrelated VS Code renderer
false positives, which were not stopped. No unrelated process was stopped.

## Rollback points

The tested Tasks 1–4 series is:

1. `44d71981af9cdc8a8a119e917beaa2c14e1eb53f` — depth enrichment callback tests;
2. `14a319852f91c0f58625866e25abf629184c808c` — single semantic 3D owner;
3. `9e40303891d029421e76c1eff1ce9de76f82fe30` — ZED-only semantic memory geometry;
4. `3f2e4e581d5bb22834f0de7b84d9308b78ed8b0b` — direct LiDAR update gate.

The rollback point immediately before the Tasks 1–4 series is
`1675b0691429ea2c7d18423b67b68de16801e289`. Reverting individual commits should
follow normal reverse order (`3f2e4e5`, `9e40303`, `14a3198`, `44d7198`) and is
not part of this validation.

## Minimal next step

Connect and verify the ZED device, and have the operator restore the already
specified LiDAR host interface (`eth0`, `192.168.1.102/24`) to `UP` without
changing CAN. Then rerun the exact Phase 4A no-motion command above, place one
stationary green bottle at a tape-measured distance, submit the fixed query,
capture all listed topics for 30–60 seconds, stop only that launch's process
group, and replace every `NOT MEASURED` / `NOT EVALUATED` entry with raw evidence
or a specific failure. Do not proceed to Nav2 tuning until that gate passes.
