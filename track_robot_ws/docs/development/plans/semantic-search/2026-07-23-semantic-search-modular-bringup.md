# Semantic Search Modular Bringup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a passive, modular, one-command bringup, diagnosis, language-query, and live-test workflow for semantic-search Phase 1 and Phase 2.

**Architecture:** `track_robot_bringup` provides small hardware launch adapters, one aggregate live launch, and a Python `semantic_search_ctl` command. The controller fixes the managed environment at ROS Domain 20, checks readiness, reuses external hardware in auto mode, owns only the processes it starts, delegates queries to the existing portal, and writes bounded live-test reports.

**Tech Stack:** ROS 2 Foxy, Python 3.8, `ament_cmake_python`, ROS launch, `rclpy`, `tf2_ros`, pytest, YAML, JSON, OpenCV through `cv_bridge`.

## Global Constraints

- The default and managed ROS domain is exactly `20`.
- No supported launch may start a controller, planner, velocity gate, follower, or `/cmd_vel` publisher.
- Phase 1 runtime graph starts/depends only on ZED and local CLIP; multi-stage
  bringup manifest may declare Phase2 packages.
- Phase 2 normal readiness requires a measured `base_link -> zed_camera_link` transform.
- Prototype camera extrinsics require both `extrinsic_mode:=prototype` and `allow_degraded:=true`.
- The controller stops only processes that it started and verified.
- Every live wait is bounded; missing dependencies produce actionable output.
- Pipeline health and semantic correctness are reported separately.
- Existing Phase 0, Phase 1, Phase 2, human-tracking, and query interfaces remain backward compatible.

---

## File structure

Create:

```text
src/track_robot/track_robot_bringup/
├── config/
│   ├── camera_extrinsic.example.yaml
│   ├── fastdds_semantic_search.xml
│   └── semantic_search_defaults.yaml
├── launch/
│   ├── semantic_search_camera.launch.py
│   ├── semantic_search_platform.launch.py
│   ├── semantic_search_sensors.launch.py
│   └── semantic_search_live.launch.py
├── resource/
│   └── track_robot_bringup
├── scripts/
│   └── semantic_search_ctl
├── test/
│   ├── test_control_config.py
│   ├── test_launch_contract.py
│   ├── test_live_report.py
│   ├── test_process_control.py
│   └── test_readiness.py
└── track_robot_bringup/
    ├── __init__.py
    ├── control_cli.py
    ├── control_config.py
    ├── live_test.py
    ├── process_control.py
    └── readiness.py
```

Modify:

```text
src/track_robot/track_robot_bringup/CMakeLists.txt
src/track_robot/track_robot_bringup/package.xml
src/track_robot/track_robot_bringup/launch/rslidar_with_tf.launch.py
docs/guides/semantic-search/phase2-recording-and-evaluation.md
```

## Task 1: Package foundation, stage policy, and managed environment

**Files:**

- Create: `src/track_robot/track_robot_bringup/track_robot_bringup/__init__.py`
- Create: `src/track_robot/track_robot_bringup/track_robot_bringup/control_config.py`
- Create: `src/track_robot/track_robot_bringup/scripts/semantic_search_ctl`
- Create: `src/track_robot/track_robot_bringup/resource/track_robot_bringup`
- Create: `src/track_robot/track_robot_bringup/config/semantic_search_defaults.yaml`
- Create: `src/track_robot/track_robot_bringup/config/fastdds_semantic_search.xml`
- Modify: `src/track_robot/track_robot_bringup/CMakeLists.txt`
- Modify: `src/track_robot/track_robot_bringup/package.xml`
- Test: `src/track_robot/track_robot_bringup/test/test_control_config.py`

**Interfaces:**

- Produces: `StageSpec`, `HardwareSelection`, `resolve_stage(name)`,
  `managed_environment(base, domain_id=20, dds_profile=None)`, and
  `default_workspace_paths(workspace_root)`.
- Later tasks consume these types and functions without importing ROS.

- [ ] **Step 1: Write failing configuration tests**

```python
def test_phase1_requires_only_camera():
    spec = resolve_stage('phase1')
    assert spec.camera is True
    assert spec.lidar is False
    assert spec.base is False
    assert spec.imu is False


def test_phase2_requires_all_passive_hardware():
    spec = resolve_stage('phase2')
    assert (spec.camera, spec.lidar, spec.base, spec.imu) == (
        True, True, True, True)


def test_managed_environment_uses_domain_20_and_preserves_parent():
    result = managed_environment({'PATH': '/bin'}, dds_profile='/tmp/dds.xml')
    assert result['PATH'] == '/bin'
    assert result['ROS_DOMAIN_ID'] == '20'
    assert result['FASTRTPS_DEFAULT_PROFILES_FILE'] == '/tmp/dds.xml'
```

- [ ] **Step 2: Run the tests and verify missing-module failures**

Run:

```bash
pytest -q src/track_robot/track_robot_bringup/test/test_control_config.py
```

Expected: collection fails because `track_robot_bringup.control_config` does not
exist.

- [ ] **Step 3: Implement immutable stage policy and environment helpers**

```python
@dataclass(frozen=True)
class StageSpec:
    name: str
    camera: bool
    lidar: bool
    base: bool
    imu: bool
    phase1: bool
    localization: bool
    tracklets: bool
    memory: bool


STAGES = {
    'sensors': StageSpec(
        'sensors', True, True, True, True, False, False, False, False),
    'phase1': StageSpec(
        'phase1', True, False, False, False, True, False, False, False),
    'phase2': StageSpec(
        'phase2', True, True, True, True, True, True, True, True),
}


def managed_environment(base, domain_id=20, dds_profile=None):
    result = dict(base)
    result['ROS_DOMAIN_ID'] = str(domain_id)
    if dds_profile:
        result['FASTRTPS_DEFAULT_PROFILES_FILE'] = str(dds_profile)
    return result
```

The defaults YAML contains topic names, timeouts, model paths relative to the
workspace, the expected checkpoint SHA-256
`40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af`,
and `ros_domain_id: 20`.

The DDS XML sets participant discovery range to 64 while leaving builtin
transports enabled; it must not select UDP-only transport.

- [ ] **Step 4: Convert bringup to a mixed CMake/Python package**

Use `ament_cmake_python`, install the Python package, install
`scripts/semantic_search_ctl` into `lib/${PROJECT_NAME}`, and register pytest
tests with `ament_cmake_pytest`. Add runtime dependencies for all included
launch packages and message types. The script is:

```python
#!/usr/bin/env python3
from track_robot_bringup.control_cli import main

raise SystemExit(main())
```

- [ ] **Step 5: Run foundation tests**

Run:

```bash
pytest -q src/track_robot/track_robot_bringup/test/test_control_config.py
```

Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add src/track_robot/track_robot_bringup
git commit -m "feat: add semantic bringup control foundation"
```

## Task 2: Passive modular hardware and feature launches

**Files:**

- Create: `src/track_robot/track_robot_bringup/launch/semantic_search_camera.launch.py`
- Create: `src/track_robot/track_robot_bringup/launch/semantic_search_platform.launch.py`
- Create: `src/track_robot/track_robot_bringup/launch/semantic_search_sensors.launch.py`
- Create: `src/track_robot/track_robot_bringup/launch/semantic_search_live.launch.py`
- Create: `src/track_robot/track_robot_bringup/config/camera_extrinsic.example.yaml`
- Modify: `src/track_robot/track_robot_bringup/launch/rslidar_with_tf.launch.py`
- Test: `src/track_robot/track_robot_bringup/test/test_launch_contract.py`

**Interfaces:**

- Consumes: stage and path defaults from Task 1.
- Produces: one aggregate launch accepting `stage`, `start_camera`,
  `start_lidar`, `start_base`, `start_imu`, `runtime_path`,
  `checkpoint_path`, `extrinsic_mode`, `extrinsic_file`, and
  `allow_degraded`.

- [ ] **Step 1: Write AST-based launch contract tests**

Tests parse all four launches and assert:

```python
assert required_live_arguments <= declared_arguments(live_launch)
assert 'semantic_search_phase1.launch.py' in live_source
assert 'semantic_search_phase0.launch.py' in live_source
assert 'semantic_memory_lidar_tracklets.launch.py' in live_source
assert 'semantic_memory_phase2.launch.py' in live_source
for forbidden in ('cmd_vel', 'controller', 'planner', 'safe_human_following'):
    assert forbidden not in combined_source
```

Also assert `rslidar_with_tf.launch.py` exposes a
`publish_base_lidar_tf` argument and gates its static publisher, preventing a
duplicate TF when external robot description already owns that edge.

- [ ] **Step 2: Run launch tests and verify failure**

Run:

```bash
pytest -q src/track_robot/track_robot_bringup/test/test_launch_contract.py
```

Expected: failure because modular launches do not exist.

- [ ] **Step 3: Implement camera launch and extrinsic fail-closed policy**

The camera launch includes `zed_wrapper/zed_camera.launch.py` only when
`start_camera` is true, with `camera_model:=zed2i`. It publishes
`base_link -> zed_camera_link` only for `measured` or explicit prototype mode.
An `OpaqueFunction` validates:

```python
if mode == 'prototype' and not allow_degraded:
    raise RuntimeError(
        'prototype camera extrinsic requires allow_degraded:=true')
if mode == 'measured' and not Path(extrinsic_file).is_file():
    raise RuntimeError(
        'measured camera extrinsic file does not exist: {}'.format(
            extrinsic_file))
```

The calibration YAML schema is:

```yaml
calibration_id: replace_with_measured_calibration_id
parent_frame: base_link
child_frame: zed_camera_link
translation: {x: 0.0, y: 0.0, z: 0.0}
rotation_rpy: {roll: 0.0, pitch: 0.0, yaw: 0.0}
```

- [ ] **Step 4: Implement platform and sensors launches**

Platform launch independently gates Bunker and IMU includes. Sensors launch
gates camera, LiDAR, base, and IMU independently and forwards the network and
extrinsic arguments. It never includes `jetson_base.launch.py` because that
launch also starts `cmd_vel_gate`.

- [ ] **Step 5: Implement aggregate live launch**

Use an `OpaqueFunction` to validate `stage in {'sensors','phase1','phase2'}` and
construct:

```text
sensors: requested hardware only
phase1: sensors(camera) + semantic_search_phase1(start_perception=true)
phase2: sensors(all) + Phase 0 localization-health + Phase 1
        + semantic-memory LiDAR tracklets + Phase 2 memory/visualizer
```

Pass the real local defaults:

```text
models/phase1_runtime/python
models/phase1/ViT-B-32.pt
```

- [ ] **Step 6: Run launch-contract and existing launch tests**

Run:

```bash
pytest -q \
  src/track_robot/track_robot_bringup/test/test_launch_contract.py \
  src/track_robot_semantic_search/test/test_launch_contract.py \
  src/track_robot_semantic_search/test/test_phase1_launch_contract.py
```

Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add src/track_robot/track_robot_bringup
git commit -m "feat: add passive modular semantic search launches"
```

## Task 3: Readiness model and bounded ROS probes

**Files:**

- Create: `src/track_robot/track_robot_bringup/track_robot_bringup/readiness.py`
- Test: `src/track_robot/track_robot_bringup/test/test_readiness.py`

**Interfaces:**

- Consumes: `StageSpec`, defaults, and managed environment.
- Produces: `CheckStatus`, `CheckResult`, `ReadinessReport`,
  `StaticProbe`, `RosCliProbe`, and `check_stage(stage, selection, paths,
  probe)`.

- [ ] **Step 1: Write status aggregation and stage-requirement tests**

```python
def test_not_ready_dominates_degraded_and_pass():
    report = ReadinessReport([
        CheckResult.pass_('camera', '15 Hz'),
        CheckResult.degraded('calibration', 'prototype'),
        CheckResult.not_ready('tf', 'base_link -> zed_camera_link missing'),
    ])
    assert report.overall is CheckStatus.NOT_READY


def test_phase1_does_not_check_lidar_or_imu(fake_probe):
    report = check_stage('phase1', paths(), fake_probe)
    assert 'lidar' not in report.names
    assert 'imu' not in report.names
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
pytest -q src/track_robot/track_robot_bringup/test/test_readiness.py
```

Expected: missing readiness module.

- [ ] **Step 3: Implement pure result types and static checks**

```python
class CheckStatus(Enum):
    PASS = 'PASS'
    NOT_READY = 'NOT READY'
    DEGRADED = 'DEGRADED'
    FAIL = 'FAIL'


@dataclass(frozen=True)
class CheckResult:
    status: CheckStatus
    name: str
    detail: str
    action: str = ''
```

Static checks validate runtime directory, checkpoint path and checksum, DDS
profile path, measured extrinsic schema, and prototype/degraded policy.

- [ ] **Step 4: Implement injected bounded ROS CLI probes**

`RosCliProbe` uses argument-vector subprocess calls with timeouts:

```text
ros2 topic list -t
ros2 topic info <topic>
ros2 topic hz <topic>
ros2 run tf2_ros tf2_echo <target> <source>
```

No command is executed through `shell=True`. Topic-rate parsing accepts Foxy
output and returns a timeout result with the exact topic. TF checks require a
successful transform line, not merely process startup.

Required topics:

```text
camera:       /zed/zed_node/left/image_rect_color
camera info:  /zed/zed_node/left/camera_info
lidar:        /rslidar_points
imu:          /imu/data_raw
odometry:     /odom
regions:      /semantic_search/regions
tracklets:    /semantic_memory/lidar_tracklets
localization: /semantic_memory/localization_state
memory:       /semantic_memory/active_objects
```

Check `/cmd_vel` with `ros2 topic info /cmd_vel`; any publisher is `FAIL`.

- [ ] **Step 5: Implement stable text and JSON rendering**

The human output aligns status/name/detail and ends with `Overall`. JSON contains
`stage`, `overall`, `checks`, `timestamp`, and `ros_domain_id`.

- [ ] **Step 6: Run readiness tests**

Run:

```bash
pytest -q src/track_robot/track_robot_bringup/test/test_readiness.py
```

Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add src/track_robot/track_robot_bringup/track_robot_bringup/readiness.py \
  src/track_robot/track_robot_bringup/test/test_readiness.py
git commit -m "feat: add bounded semantic stack readiness checks"
```

## Task 4: Safe process ownership and control CLI

**Files:**

- Create: `src/track_robot/track_robot_bringup/track_robot_bringup/process_control.py`
- Create: `src/track_robot/track_robot_bringup/track_robot_bringup/control_cli.py`
- Test: `src/track_robot/track_robot_bringup/test/test_process_control.py`
- Test: `src/track_robot/track_robot_bringup/test/test_control_cli.py`

**Interfaces:**

- Consumes: Task 1 configuration and Task 3 readiness.
- Produces: `ProcessIdentity`, `OwnedProcessState`, `ProcessManager`,
  `build_parser()`, and `main(argv=None)`.

- [ ] **Step 1: Write process identity and cleanup tests**

```python
def test_stale_pid_is_not_signalled(tmp_path, fake_os):
    manager = ProcessManager(tmp_path / 'state.json', os_api=fake_os)
    manager.write_state(pid=42, start_ticks=100, command=EXPECTED)
    fake_os.start_ticks[42] = 101
    assert manager.stop_owned() is False
    assert fake_os.signals == []


def test_cleanup_uses_process_group_sigint_first(tmp_path, fake_os):
    manager = ProcessManager(tmp_path / 'state.json', os_api=fake_os)
    manager.write_state(pid=42, start_ticks=100, command=EXPECTED)
    fake_os.start_ticks[42] = 100
    fake_os.process_groups[42] = 41
    assert manager.stop_owned() is True
    assert fake_os.signals[0] == (41, signal.SIGINT)
    assert fake_os.signals[0] == (pgid, signal.SIGINT)
```

CLI parser tests assert every command, `--hardware auto|external`,
`--allow-degraded`, `--extrinsic-mode`, `--start-stack`, and default Domain 20.

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
pytest -q \
  src/track_robot/track_robot_bringup/test/test_process_control.py \
  src/track_robot/track_robot_bringup/test/test_control_cli.py
```

Expected: missing modules.

- [ ] **Step 3: Implement verified process state**

State is written atomically under:

```text
${ROS_HOME:-~/.ros}/track_robot_semantic_search/managed_process.json
```

It records PID, process-group ID, `/proc/<pid>/stat` start ticks, exact argv,
stage, owned modules, and start time. `stop_owned()` rereads `/proc` and refuses
to signal when PID, start ticks, or expected launch command differ.

- [ ] **Step 4: Implement CLI and auto hardware selection**

`auto` calls a bounded topic-list probe before launch and maps healthy existing
publishers to:

```text
start_camera:=false
start_lidar:=false
start_base:=false
start_imu:=false
```

for each reusable module. `external` sets all four false. Missing auto modules
are true. The controller launches:

```text
ros2 launch track_robot_bringup semantic_search_live.launch.py \
  stage:=phase1 start_camera:=true start_lidar:=false \
  start_base:=false start_imu:=false
```

with an argument vector, new process group, managed Domain 20 environment, and
no shell.

- [ ] **Step 5: Implement command behavior**

- `doctor`: static and external readiness only; starts nothing.
- `status`: checks current graph and reports managed-process identity.
- `start`: static preflight, auto selection, launch, bounded readiness, then
  foreground wait with signal forwarding.
- `query`: `os.execvpe` the existing `semantic_search_query` command with the
  managed environment.
- `stop`: verified process-group cleanup.
- `test`: delegated to Task 5.

Exit codes are `0=PASS`, `2=NOT READY`, `3=DEGRADED`, `4=FAIL`, and
`130=interrupted`.

- [ ] **Step 6: Run CLI and process tests**

Run:

```bash
pytest -q \
  src/track_robot/track_robot_bringup/test/test_process_control.py \
  src/track_robot/track_robot_bringup/test/test_control_cli.py
```

Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add src/track_robot/track_robot_bringup
git commit -m "feat: add safe semantic stack control CLI"
```

## Task 5: Bounded live test and review artifacts

**Files:**

- Create: `src/track_robot/track_robot_bringup/track_robot_bringup/live_test.py`
- Test: `src/track_robot/track_robot_bringup/test/test_live_report.py`

**Interfaces:**

- Consumes: managed environment, readiness, process manager, and the existing
  `semantic_search_query`.
- Produces: `RegionSample`, `Phase2Sample`, `LiveTestSummary`,
  `build_report(summary)`, `write_report(summary, output_dir)`, and
  `run_live_test(stage, query, duration_sec, output_dir, environment)`.

- [ ] **Step 1: Write pure report and score tests**

```python
def test_report_separates_pipeline_from_semantic_correctness(tmp_path):
    summary = LiveTestSummary(
        stage='phase1', query='blue chair', frames=10,
        nonempty_frames=8, scores=[0.2, 0.4], query_ids={7},
        failures=[])
    report = build_report(summary)
    assert report['pipeline']['status'] == 'PASS'
    assert report['semantic_result']['status'] == 'REVIEW REQUIRED'
    assert report['metrics']['nonempty_frame_ratio'] == 0.8


def test_nonfinite_score_fails_pipeline():
    summary = LiveTestSummary(
        stage='phase1', query='blue chair', frames=1,
        nonempty_frames=1, scores=[float('nan')], query_ids={7},
        failures=[])
    assert build_report(summary)['pipeline']['status'] == 'FAIL'
```

- [ ] **Step 2: Run tests and verify failure**

Run:

```bash
pytest -q src/track_robot/track_robot_bringup/test/test_live_report.py
```

Expected: missing live-test module.

- [ ] **Step 3: Implement bounded subscriptions and metrics**

After the existing query command returns accepted, create a temporary rclpy
collector for the requested duration. Phase 1 subscribes to image and
`SemanticRegionArray`. Phase 2 also subscribes to
`SemanticLidarTrackletArray`, `SemanticLocalizationState`,
`SemanticObjectArray`, and association diagnostics.

Collect only bounded counters, the latest image, and the highest-scoring region;
do not retain an unbounded message history.

- [ ] **Step 4: Implement overlay and report writing**

Draw the best ROI and score on the latest correlated image using `cv_bridge`
and OpenCV. If image conversion is unavailable, still write JSON and record an
overlay warning rather than failing pipeline collection.

Write atomically to:

```text
~/.ros/track_robot_semantic_search/reports/<UTC timestamp>/
├── report.json
└── phase1_overlay.png
```

The JSON includes query IDs/versions, frame counts, score summary, rates,
tracklet/object/association counts, calibration mode, readiness snapshot,
pipeline status, and `semantic_result.status = "REVIEW REQUIRED"`.

- [ ] **Step 5: Implement `--start-stack` ownership**

If requested, start the stack through `ProcessManager`, wait for readiness,
run the query and collector, then clean up in `finally`. If a stack already
passes readiness, default test mode reuses it and owns no process.

- [ ] **Step 6: Run live-report tests**

Run:

```bash
pytest -q src/track_robot/track_robot_bringup/test/test_live_report.py
```

Expected: all pass.

- [ ] **Step 7: Commit**

```bash
git add src/track_robot/track_robot_bringup
git commit -m "feat: add bounded semantic live-test reports"
```

## Task 6: Documentation, build, regression, and passive runtime smoke test

**Files:**

- Modify: `docs/guides/semantic-search/phase2-recording-and-evaluation.md`
- Modify: `src/track_robot/track_robot_bringup/package.xml`
- Modify: `src/track_robot/track_robot_bringup/CMakeLists.txt`

**Interfaces:**

- Consumes: all earlier tasks.
- Produces: a user-ready quick-start and verification evidence.

- [ ] **Step 1: Add a concise quick-start before manual procedures**

Document:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash

ros2 run track_robot_bringup semantic_search_ctl doctor phase1
ros2 run track_robot_bringup semantic_search_ctl start phase1
ros2 run track_robot_bringup semantic_search_ctl query "blue chair"
ros2 run track_robot_bringup semantic_search_ctl test phase1 "blue chair"
```

Add corresponding Phase 2 commands, measured-extrinsic requirement,
`hardware external`, output locations, status meanings, Domain 20, and cleanup.

- [ ] **Step 2: Run focused Python and launch tests**

Run:

```bash
pytest -q \
  src/track_robot/track_robot_bringup/test \
  src/track_robot_semantic_search/test/test_launch_contract.py \
  src/track_robot_semantic_search/test/test_phase1_launch_contract.py \
  src/track_robot_semantic_search/test/test_query_cli.py
```

Expected: all pass.

- [ ] **Step 3: Build affected packages**

Run:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-up-to \
  track_robot_bringup track_robot_semantic_search \
  track_robot_semantic_memory track_robot_lidar_tracking
```

Expected: build completes successfully.

- [ ] **Step 4: Run installed-interface checks**

Run:

```bash
source install/setup.bash
ros2 run track_robot_bringup semantic_search_ctl --help
ros2 launch track_robot_bringup semantic_search_live.launch.py --show-args
```

Expected: all six commands and all launch arguments are listed; help states
Domain 20 and passive behavior.

- [ ] **Step 5: Run a no-hardware passive doctor smoke test**

Run:

```bash
source install/setup.bash
ros2 run track_robot_bringup semantic_search_ctl doctor phase1
```

Expected: bounded `PASS` for the model and environment plus `NOT READY` for an
absent camera if it is not running. The command exits without starting nodes.

- [ ] **Step 6: Verify no managed nodes remain**

Inspect the process state and ROS graph. Any node started by a smoke test must
be stopped. Confirm `/cmd_vel` has zero publishers.

- [ ] **Step 7: Commit**

```bash
git add docs/guides/semantic-search/phase2-recording-and-evaluation.md \
  src/track_robot/track_robot_bringup
git commit -m "docs: add modular semantic search quick start"
```

## Final verification

- [ ] Run `git diff --check`.
- [ ] Run the focused test suite again from a clean shell.
- [ ] Run `colcon test --packages-select track_robot_bringup` and inspect
  `colcon test-result --verbose`.
- [ ] Confirm every spawned test/debug ROS process is stopped.
- [ ] Review `git status --short` and ensure only intended files remain.

## Task 7: Remove the bringup/perception package cycle

**Files:**

- Create: `src/track_robot/track_robot_sensor_bringup/CMakeLists.txt`
- Create: `src/track_robot/track_robot_sensor_bringup/package.xml`
- Create: `src/track_robot/track_robot_sensor_bringup/config/rslidar_track_robot.yaml`
- Create: `src/track_robot/track_robot_sensor_bringup/launch/rslidar_with_tf.launch.py`
- Create: `src/track_robot/track_robot_sensor_bringup/test/test_sensor_bringup_contract.py`
- Modify: `src/track_robot/track_robot_bringup/launch/rslidar_with_tf.launch.py`
- Modify: `src/track_robot/track_robot_bringup/package.xml`
- Modify: `src/track_robot_perception/launch/point_lio_rshelios.launch.py`
- Modify: `src/track_robot_perception/launch/fast_lio_rshelios.launch.py`
- Modify: `src/track_robot_perception/package.xml`

**Interfaces:**

- Produces: a motion-free `track_robot_sensor_bringup` package that owns the
  existing RS-LiDAR network, driver, runtime-config, and
  `base_link -> rslidar` TF behavior.
- Preserves: `ros2 launch track_robot_bringup rslidar_with_tf.launch.py` and
  all of its public launch arguments through a compatibility wrapper.

- [ ] **Step 1: Write failing dependency and compatibility tests**

The test parses package manifests and launch sources:

```python
assert 'track_robot_bringup' not in perception_exec_dependencies
assert 'track_robot_sensor_bringup' in perception_exec_dependencies
assert 'track_robot_sensor_bringup' in bringup_exec_dependencies
assert 'rslidar_with_tf.launch.py' in bringup_wrapper_source
assert 'track_robot_sensor_bringup' in bringup_wrapper_source
assert 'track_robot_sensor_bringup' in point_lio_source
assert 'track_robot_sensor_bringup' in fast_lio_source
```

It also asserts the sensor launch retains `configure_network`,
`network_interface`, `host_ip`, `host_cidr`, `driver_start_delay`,
`publish_base_lidar_tf`, and `config_path`, and contains neither controller
nor `/cmd_vel`.

- [ ] **Step 2: Run RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot/track_robot_sensor_bringup/test/test_sensor_bringup_contract.py
```

Expected: failure because the sensor package does not exist.

- [ ] **Step 3: Move the low-level LiDAR implementation without changing behavior**

The sensor package installs its `launch` and `config` directories. Move the
implementation and YAML byte-for-byte except for package-share lookup. The
new package depends only on `launch`, `launch_ros`, `rslidar_sdk`, and
`tf2_ros`; it must not depend on bringup or perception.

- [ ] **Step 4: Preserve the public bringup launch**

Replace the old bringup implementation with a thin
`IncludeLaunchDescription` wrapper. Declare and forward every existing public
argument:

```text
configure_network
network_interface
host_ip
host_cidr
driver_start_delay
publish_base_lidar_tf
config_path
```

The wrapper default `config_path` resolves the new sensor package config.

- [ ] **Step 5: Redirect perception and remove the reverse edge**

Point-LIO includes the sensor package launch. Fast-LIO resolves the sensor
package config. Replace perception's manifest dependency on
`track_robot_bringup` with `track_robot_sensor_bringup`. Add
`track_robot_sensor_bringup` to top-level bringup.

- [ ] **Step 6: Run GREEN and topology/build checks**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q \
  src/track_robot/track_robot_sensor_bringup/test/test_sensor_bringup_contract.py
python3 -m pytest -q \
  src/track_robot/track_robot_bringup/test/test_launch_contract.py
colcon list --topological-order
colcon build --packages-up-to track_robot_bringup --event-handlers console_direct+
```

Expected: tests pass, topological ordering contains no cycle, and the affected
packages build.

- [ ] **Step 7: Commit**

```bash
git add docs/superpowers src/track_robot/track_robot_sensor_bringup \
  src/track_robot/track_robot_bringup \
  src/track_robot_perception
git commit -m "refactor: split shared lidar sensor bringup"
```
