# Canonical Sensor TF and RViz Data Chain Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the Bunker Pro 2 URDF the sole owner of Phase 1–5 mechanical sensor transforms, publish RoboSense points in `lidar_link`, fail fast when the LiDAR NIC is not configured, and make RViz reliably display the robot, point cloud, paths, and costmaps.

**Architecture:** The Bunker Pro 2 `robot_state_publisher` owns `robot_bottom -> base_link -> sensor_station_link -> camera_mount_link/camera_link/lidar_link`, while the ZED publisher owns only its vendor-internal subtree below `zed_camera_link`. The existing control command starts the same Phase 4B/5A stack but now verifies the NIC before spawning drivers and waits for camera, LiDAR, odometry, semantic outputs, and canonical TFs before presenting the stack as ready.

**Tech Stack:** Ubuntu/Jetson, ROS 2 Foxy Python launch, URDF, `robot_state_publisher`, RoboSense ROS 2 SDK, ZED wrapper, Nav2 costmaps, RViz2, Python 3.8, pytest/ament_cmake_pytest, colcon.

## Global Constraints

- Preserve `robot_bottom -> base_link` as `xyz="0 0 0.45" rpy="0 0 0"`.
- Preserve the calibrated `base_link -> sensor_station_link` and sensor-station-to-original-mount transform values exactly.
- Express the new camera correction as `camera_mount_link -> camera_link`, `xyz="-0.185 0 0"`, along the camera mount's local X axis.
- Connect `camera_link -> zed_camera_link` with `xyz="0.01 0 -0.015" rpy="0 0 0"` so left/right camera frames resolve to `(0, +0.06, 0)` and `(0, -0.06, 0)` from `camera_link`.
- Keep ZED vendor-internal frames; do not duplicate their joints in `bunker_pro2.urdf`.
- Use `lidar_link` as the only RoboSense frame; do not retain a second `rslidar` alias in the Phase 1–5 stack.
- Keep public semantic topics, messages, services, global IDs, Nav2 interfaces, safety supervisor, and velocity gate unchanged.
- Keep Nav2 `robot_base_frame=base_link`, Bunker odometry child `robot_bottom`, ROS domain 20, and IMU disabled in Phase 4B/5A.
- Never run `sudo` inside launch or control code; print the three explicit `ip` recovery commands when the NIC check fails.
- Do not command robot motion during TF/RViz validation.
- Do not overwrite unrelated workspace changes.

---

## File Structure

**Mechanical description**

- Modify `src/bunker_pro2/urdf/bunker_pro2.urdf`: own the canonical camera mount, camera reference, ZED root, and LiDAR frames.
- Modify `src/bunker_pro2/test/test_description_contract.py`: verify exact link/joint ownership and camera geometry.

**LiDAR driver and network boundary**

- Modify `track_robot_ws/src/track_robot/track_robot_sensor_bringup/config/rslidar_track_robot.yaml`: publish PointCloud2 with `header.frame_id=lidar_link`.
- Modify `track_robot_ws/src/track_robot/track_robot_sensor_bringup/launch/rslidar_with_tf.launch.py`: replace hidden network mutation and legacy static TF with a bounded read-only NIC preflight.
- Modify `track_robot_ws/src/track_robot/track_robot_sensor_bringup/test/test_sensor_bringup_contract.py`: verify frame ID, lack of `sudo`, absence of legacy TF, and preflight error text.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/launch/rslidar_with_tf.launch.py`: preserve the compatibility wrapper's argument surface while forwarding verification behavior.

**Canonical TF selection and managed readiness**

- Modify `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_camera.launch.py`: add `robot_description` extrinsic mode and suppress the legacy static publisher in that mode.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_sensors.launch.py`: default the integrated robot path to URDF-owned TF.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_live.launch.py`: forward the canonical TF mode.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase4a.launch.py`: use `robot_description`, disable legacy LiDAR TF, and continue including Bunker description.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase4b.launch.py`: forward the canonical mode.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase5a.launch.py`: forward the canonical mode.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/control_config.py`: define no-IMU Phase 4B/5A readiness requirements.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/readiness.py`: accept URDF-owned calibration and check `base_link -> lidar_link`.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/control_cli.py`: make managed Phase 4B/5A runs use NIC verification and bounded post-launch readiness.
- Modify `track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py`, `test/test_readiness.py`, `test/test_control_cli.py`, `test/test_phase4a_launch_contract.py`, `test/test_phase4b_launch_contract.py`, and `test/test_phase5a_launch_contract.py`: lock the canonical ownership and fail-closed startup contract.

**RViz and operator documentation**

- Modify `track_robot_ws/src/track_robot/track_robot_bringup/rviz/semantic_search_phase4.rviz`, `semantic_search_phase4b.rviz`, and `semantic_search_phase5a.rviz`: explicit PointCloud2 and Map QoS plus existing path/robot displays.
- Modify the matching RViz contract tests listed above.
- Modify `track_robot_ws/docs/guides/semantic-search/phase2-recording-and-evaluation.md`: update the standard managed test sequence already enforced by `test_quick_start_doc.py`.
- Create `docs/validation/2026-08-09-canonical-sensor-tf-rviz-validation.md`: exact build/test/runtime evidence without motion.

---

### Task 1: Canonical Camera and LiDAR URDF Geometry

**Files:**
- Modify: `src/bunker_pro2/urdf/bunker_pro2.urdf`
- Modify: `src/bunker_pro2/test/test_description_contract.py`

**Interfaces:**
- Consumes: existing calibrated sensor-station origins in `bunker_pro2.urdf`.
- Produces: fixed joints `sensor_station_camera_mount_joint`, `camera_mount_to_camera_joint`, `camera_to_zed_camera_joint`, and existing `sensor_station_lidar_joint`; canonical links `camera_mount_link`, `camera_link`, `zed_camera_link`, and `lidar_link`.

- [ ] **Step 1: Replace the old camera-link test with failing exact-geometry tests**

Add assertions equivalent to:

```python
def _joint(robot, name, parent, child, xyz, rpy):
    joint = robot.find("./joint[@name='{}']".format(name))
    assert joint is not None
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == parent
    assert joint.find('child').attrib['link'] == child
    assert joint.find('origin').attrib == {'xyz': xyz, 'rpy': rpy}


def test_camera_reference_uses_local_x_offset_and_vendor_root_connector():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    links = {link.attrib['name'] for link in robot.findall('link')}
    assert {'camera_mount_link', 'camera_link', 'zed_camera_link'} <= links
    _joint(
        robot, 'sensor_station_camera_mount_joint',
        'sensor_station_link', 'camera_mount_link',
        '-0.2212 0.318 0', '1.57079632679 0 3.14159265359')
    _joint(
        robot, 'camera_mount_to_camera_joint',
        'camera_mount_link', 'camera_link', '-0.185 0 0', '0 0 0')
    _joint(
        robot, 'camera_to_zed_camera_joint',
        'camera_link', 'zed_camera_link', '0.01 0 -0.015', '0 0 0')
```

Update the expected ordered link list to include `camera_mount_link` and `zed_camera_link` once each.

- [ ] **Step 2: Run the focused description test and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q src/bunker_pro2/test/test_description_contract.py
```

Expected: FAIL because `camera_mount_link` and the two new joints do not exist.

- [ ] **Step 3: Implement the minimum canonical camera chain**

Replace the existing `sensor_station_camera_joint` section with:

```xml
  <link name="camera_mount_link" />
  <joint name="sensor_station_camera_mount_joint" type="fixed">
    <origin xyz="-0.2212 0.318 0"
            rpy="1.57079632679 0 3.14159265359" />
    <parent link="sensor_station_link" />
    <child link="camera_mount_link" />
  </joint>
  <link name="camera_link" />
  <joint name="camera_mount_to_camera_joint" type="fixed">
    <origin xyz="-0.185 0 0" rpy="0 0 0" />
    <parent link="camera_mount_link" />
    <child link="camera_link" />
  </joint>
  <link name="zed_camera_link" />
  <joint name="camera_to_zed_camera_joint" type="fixed">
    <origin xyz="0.01 0 -0.015" rpy="0 0 0" />
    <parent link="camera_link" />
    <child link="zed_camera_link" />
  </joint>
```

Leave `sensor_station_joint` and `sensor_station_lidar_joint` byte-for-byte numerically unchanged.

- [ ] **Step 4: Run description tests and URDF parser checks**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q src/bunker_pro2/test/test_description_contract.py
check_urdf src/bunker_pro2/urdf/bunker_pro2.urdf
```

Expected: all pytest cases PASS; `check_urdf` reports a valid tree rooted at `robot_bottom`.

- [ ] **Step 5: Commit the independently reviewable URDF change**

```bash
git add src/bunker_pro2/urdf/bunker_pro2.urdf src/bunker_pro2/test/test_description_contract.py
git commit -m "fix(tf): define canonical camera reference chain"
```

---

### Task 2: RoboSense Frame and Read-Only NIC Preflight

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_sensor_bringup/config/rslidar_track_robot.yaml`
- Modify: `track_robot_ws/src/track_robot/track_robot_sensor_bringup/launch/rslidar_with_tf.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_sensor_bringup/test/test_sensor_bringup_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/rslidar_with_tf.launch.py`

**Interfaces:**
- Consumes: launch arguments `configure_network`, `network_interface`, `host_ip`, `host_cidr`, `driver_start_delay`, `config_path`; legacy `publish_base_lidar_tf` remains accepted as a deprecated no-op so existing command lines still parse, while integrated launchers pass `false`.
- Produces: `/rslidar_points` with `header.frame_id=lidar_link`; `_verify_network(interface: str, expected_cidr: str, runner=subprocess.run) -> None`.

- [ ] **Step 1: Write failing driver and preflight contract tests**

Update `test_sensor_bringup_contract.py` to assert:

```python
def test_robot_lidar_uses_urdf_owned_frame_without_legacy_static_tf():
    source = _source(SENSOR_LAUNCH)
    config = _source(SENSOR_PACKAGE / 'config' / 'rslidar_track_robot.yaml')
    assert 'ros_frame_id: lidar_link' in config
    assert "name='base_to_rslidar_tf'" not in source
    assert 'static_transform_publisher' not in source


def test_network_preflight_is_read_only_and_actionable():
    source = _source(SENSOR_LAUNCH)
    assert "['ip', '-o', '-4', 'addr', 'show', 'dev', interface]" in source
    assert "['ip', '-o', 'link', 'show', 'dev', interface]" in source
    assert 'ExecuteProcess' not in source
    assert 'sudo -n' not in source
    assert 'sudo ip addr flush dev' in source
    assert 'sudo ip addr add' in source
    assert 'sudo ip link set' in source
```

The implementation may store the recovery text as one multiline string; the test should match the exact user-facing command fragments rather than an AST layout.

- [ ] **Step 2: Run the sensor contract and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  track_robot_ws/src/track_robot/track_robot_sensor_bringup/test/test_sensor_bringup_contract.py
```

Expected: FAIL because the config still uses `rslidar`, launch still creates a static TF, and launch still invokes `sudo -n`.

- [ ] **Step 3: Implement a bounded read-only preflight**

In `rslidar_with_tf.launch.py`, replace `network_setup`, its exit handler, and `base_to_rslidar_tf` with helpers shaped as:

```python
def _verify_network(interface, expected_cidr, runner=subprocess.run):
    addr = runner(
        ['ip', '-o', '-4', 'addr', 'show', 'dev', interface],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, timeout=2.0, check=False)
    link = runner(
        ['ip', '-o', 'link', 'show', 'dev', interface],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, timeout=2.0, check=False)
    has_address = addr.returncode == 0 and expected_cidr in addr.stdout.split()
    is_up = link.returncode == 0 and re.search(r'<[^>]*\bUP\b[^>]*>', link.stdout)
    if has_address and is_up:
        return
    raise RuntimeError(
        'LiDAR NIC is NOT READY: {} must be UP with {}. Run:\n'
        'sudo ip addr flush dev {}\n'
        'sudo ip addr add {} dev {}\n'
        'sudo ip link set {} up'.format(
            interface, expected_cidr, interface,
            expected_cidr, interface, interface))
```

Call `_verify_network()` from `_launch_setup` only when `configure_network` evaluates true. Keep a delayed `rslidar_sdk_node`, but do not create any process that mutates network state and do not create a static TF node. When the retained `publish_base_lidar_tf` value is true, emit one `LogInfo` deprecation message explaining that `bunker_pro2/robot_description` owns the transform; never silently create a second edge. Change YAML to:

```yaml
ros:
  ros_frame_id: lidar_link
```

- [ ] **Step 4: Add executable helper tests for pass/fail preflight behavior**

Load the launch module using `importlib.util.spec_from_file_location` and inject a runner returning `subprocess.CompletedProcess`. Verify:

```python
def test_preflight_accepts_exact_up_interface():
    outputs = iter((
        subprocess.CompletedProcess([], 0, '2: eth0    inet 192.168.1.102/24', ''),
        subprocess.CompletedProcess([], 0, '2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP>', ''),
    ))
    module._verify_network(
        'eth0', '192.168.1.102/24', runner=lambda *a, **k: next(outputs))


def test_preflight_rejects_missing_ipv4_with_copyable_recovery_commands():
    outputs = iter((
        subprocess.CompletedProcess([], 0, '', ''),
        subprocess.CompletedProcess([], 0, '2: eth0: <BROADCAST,UP,LOWER_UP>', ''),
    ))
    with pytest.raises(RuntimeError, match='192.168.1.102/24') as error:
        module._verify_network(
            'eth0', '192.168.1.102/24', runner=lambda *a, **k: next(outputs))
    assert 'sudo ip addr add 192.168.1.102/24 dev eth0' in str(error.value)
```

- [ ] **Step 5: Run both sensor and wrapper contracts**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  track_robot_ws/src/track_robot/track_robot_sensor_bringup/test/test_sensor_bringup_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py
```

Expected: PASS. The wrapper still accepts/forwards its published argument set, while no integrated runtime creates `base_link -> rslidar`.

- [ ] **Step 6: Commit the driver-boundary change**

```bash
git add \
  track_robot_ws/src/track_robot/track_robot_sensor_bringup/config/rslidar_track_robot.yaml \
  track_robot_ws/src/track_robot/track_robot_sensor_bringup/launch/rslidar_with_tf.launch.py \
  track_robot_ws/src/track_robot/track_robot_sensor_bringup/test/test_sensor_bringup_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/launch/rslidar_with_tf.launch.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py
git commit -m "fix(lidar): use canonical frame and verify NIC"
```

---

### Task 3: URDF-Owned Camera TF Mode and Integrated Launch Ownership

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_camera.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_sensors.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_live.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase4a.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase4b.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase5a.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py`

**Interfaces:**
- Consumes: launch argument `extrinsic_mode`.
- Produces: accepted value `robot_description`; in that mode no `base_to_zed_camera_tf` is created, Bunker description supplies `camera_link -> zed_camera_link`, and Phase 4A/4B/5A pass `publish_base_lidar_tf=false`.

- [ ] **Step 1: Add failing launch ownership tests**

Add contract assertions:

```python
def test_integrated_phases_use_robot_description_as_only_sensor_tf_owner():
    phase4a = _source(PHASE4A_LAUNCH)
    phase4b = _source(PHASE4B_LAUNCH)
    phase5a = _source(PHASE5A_LAUNCH)
    assert "'extrinsic_mode': 'robot_description'" in phase4a
    assert "'publish_base_lidar_tf': 'false'" in phase4a
    assert 'robot_description' in phase4b
    assert 'robot_description' in phase5a


def test_camera_robot_description_mode_emits_no_legacy_static_transform():
    source = _source(CAMERA_LAUNCH)
    assert "('none', 'prototype', 'measured', 'robot_description')" in source
    assert "if mode in ('none', 'robot_description'):" in source
```

Also assert top-level Phase 4B/5A defaults are `robot_description` and Phase 4A still includes `bunker_pro2/description.launch.py` once.

- [ ] **Step 2: Run launch tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py
```

Expected: FAIL because integrated launches still request prototype/static sensor TF.

- [ ] **Step 3: Add the non-publishing `robot_description` camera mode**

In `_launch_extrinsic`, use:

```python
if mode not in ('none', 'prototype', 'measured', 'robot_description'):
    raise RuntimeError(
        'unknown camera extrinsic mode {!r}; expected none, prototype, '
        'measured, or robot_description'.format(mode))
if mode in ('none', 'robot_description'):
    return []
```

Keep the legacy `prototype` and `measured` code paths for standalone compatibility. Extend each declaration/parser choice that owns this public value; do not delete an existing value.

- [ ] **Step 4: Pin the integrated stack to one owner**

Apply these launch rules:

```python
# semantic_search_phase4a.launch.py -> semantic_search_sensors.launch.py
'publish_base_lidar_tf': 'false',
'extrinsic_mode': 'robot_description',
```

Phase 4B and Phase 5A defaults become `robot_description` and forward it to Phase 4A. `semantic_search_live.launch.py` includes `bunker_pro2/description.launch.py` exactly once for non-Phase-0 stages, defaults to `robot_description`, and defaults the legacy LiDAR publisher false. `semantic_search_sensors.launch.py` retains its standalone-compatible `extrinsic_mode=none` default, but forwards the explicit mode selected by its integrated parent. Do not remove the existing Bunker description include from Phase 4A.

- [ ] **Step 5: Run focused launch and description regression**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  src/bunker_pro2/test/test_description_contract.py \
  track_robot_ws/src/track_robot/track_robot_sensor_bringup/test/test_sensor_bringup_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py
```

Expected: PASS; source scan finds no integrated Phase 1–5 legacy sensor TF owner.

- [ ] **Step 6: Commit launch ownership separately**

```bash
git add track_robot_ws/src/track_robot/track_robot_bringup/launch \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py
git commit -m "fix(bringup): make robot description own sensor TF"
```

---

### Task 4: Fail-Closed Phase 4B/5A Managed Readiness

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/control_config.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/readiness.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/control_cli.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_control_config.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_readiness.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_control_cli.py`

**Interfaces:**
- Consumes: `resolve_stage('phase4b'|'phase5a')`, canonical TFs and sensor topics.
- Produces: `run phase4b/phase5a --readiness-timeout 30 --probe-timeout 3`; motion-capable launch remains stopped/not presented as ready if LiDAR, odometry, semantic output, canonical TF, or the enforced velocity chain is absent. Internal readiness policies are named `phase4b`, `phase5a_active`, and `phase5a_passive`.

- [ ] **Step 1: Add failing no-IMU stage and canonical readiness tests**

Add exact assertions:

```python
def test_phase4b_and_phase5a_require_navigation_inputs_but_not_imu():
    for name in ('phase4b', 'phase5a_active'):
        spec = resolve_stage(name)
        assert spec.camera and spec.lidar and spec.base
        assert spec.phase1 and spec.memory and spec.diagnostic_ranking
        assert not spec.imu
    passive = resolve_stage('phase5a_passive')
    assert passive.camera and passive.lidar and passive.phase1
    assert not passive.base and not passive.imu


def test_phase4b_checks_canonical_sensor_frames(paths, fake_probe):
    paths.update(extrinsic_mode='robot_description')
    report = check_stage('phase4b', selection(), paths, fake_probe)
    assert 'imu' not in report.names
    assert ('tf_lidar', 'base_link', 'lidar_link') in fake_probe.tf_calls
```

Extend `FakeProbe` with publisher recording and add assertions that `phase4b` and `phase5a_active` require exactly one publisher on `/nav2/cmd_vel_raw`, `/nav2/cmd_vel_safe`, and `/cmd_vel`, while `phase5a_passive` retains the zero-`/cmd_vel` check. Add a control test whose fake child remains alive but readiness returns `NOT READY(lidar)`; assert `stop_owned()` is called, exit code is 2, and the “started” operator instructions are not printed.

- [ ] **Step 2: Run config/readiness/control tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_control_config.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_readiness.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_control_cli.py
```

Expected: FAIL because Phase 4B/5A are not readiness stages, TF still checks `rslidar`, and `run` does not wait for readiness.

- [ ] **Step 3: Define the runtime stage policies and URDF calibration result**

Add a `motion_active: bool = False` field to `StageSpec`. Define `phase4b` and `phase5a_active` with camera/lidar/base true, IMU false, Phase 1/localization/tracklets/memory/ranking true, and `motion_active=true`. Define `phase5a_passive` with the same semantic requirements but base, IMU, and `motion_active` false. In `StaticProbe._calibration()`:

```python
if mode == 'robot_description':
    return CheckResult.pass_(
        'calibration', 'sensor extrinsics owned by bunker_pro2 robot_description')
```

Change the canonical TF readiness check to:

```python
checks.append(probe.transform('tf_lidar', 'base_link', 'lidar_link'))
```

At the end of `check_stage`, replace the unconditional zero-publisher check with:

```python
if spec.motion_active:
    checks.extend((
        probe.publisher('nav2_cmd_raw', '/nav2/cmd_vel_raw'),
        probe.publisher('nav2_cmd_safe', '/nav2/cmd_vel_safe'),
        probe.publisher('cmd_vel_gate', '/cmd_vel'),
    ))
else:
    checks.append(probe.cmd_vel())
```

Static navigation contract tests remain responsible for proving that controller output is remapped to raw, the supervisor owns safe output, and the gate is the only final publisher. Runtime readiness proves those three links are actually present.

- [ ] **Step 4: Make managed Phase 4B/5A wait for readiness**

Add `--readiness-timeout` and `--probe-timeout` to the `run` parser. Build both commands with:

```python
'configure_network:=true',
'extrinsic_mode:=robot_description',
```

Extend `_wait_for_readiness()` with an optional `readiness_stage` argument that defaults to `args.stage`. After `spawn_verified`, call it with `phase4b`, `phase5a_active` for `ROTATION_SUPERVISED`, or `phase5a_passive` for passive/shadow mode, plus `HardwareSelection(camera=True, lidar=True, base=active, imu=False)`. Only print the operator workflow and enter `_foreground_wait` when status is PASS. For DEGRADED, NOT READY, or FAIL, call the existing stop proxy/manager, render the report, and return `exit_code(report.overall)`. The public command remains `run phase5a`.

- [ ] **Step 5: Run focused tests and verify all readiness gates**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_control_config.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_readiness.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_control_cli.py
```

Expected: PASS; tests prove no IMU requirement, `lidar_link` TF checking, missing LiDAR shutdown, and no readiness bypass before operator instructions.

- [ ] **Step 6: Commit readiness behavior independently**

```bash
git add \
  track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/control_config.py \
  track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/readiness.py \
  track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/control_cli.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_control_config.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_readiness.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_control_cli.py
git commit -m "fix(bringup): gate supervised runs on canonical sensors"
```

---

### Task 5: Explicit RViz QoS and Display Contracts

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/rviz/semantic_search_phase4.rviz`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/rviz/semantic_search_phase4b.rviz`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/rviz/semantic_search_phase5a.rviz`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py`

**Interfaces:**
- Consumes: `/rslidar_points`, `/global_costmap/costmap`, `/local_costmap/costmap`, `/safety/local_obstacle_grid`, `/semantic_search/phase4/planned_path`, `/plan`, and `/robot_description`.
- Produces: saved RViz views whose sensor/costmap subscriptions match publisher QoS and whose path displays remain enabled.

- [ ] **Step 1: Write failing RViz serialization tests**

For every active PointCloud2 block, assert the saved file contains:

```yaml
Topic:
  Depth: 5
  Durability Policy: Volatile
  History Policy: Keep Last
  Reliability Policy: Best Effort
  Value: /rslidar_points
```

For every Nav2/safety Map block, assert:

```yaml
Topic:
  Depth: 5
  Durability Policy: Transient Local
  History Policy: Keep Last
  Reliability Policy: Reliable
```

Retain assertions for RobotModel and the Phase 4A/Nav2 path topics.

- [ ] **Step 2: Run RViz contract tests and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py
```

Expected: FAIL because topic QoS is not explicit in the current saved views.

- [ ] **Step 3: Add explicit bounded QoS to the three RViz views**

Update only the Topic mapping of the relevant displays. Keep queue/depth finite, leave all semantic panel settings intact, and do not remove `/semantic_search/phase4/planned_path` or `/plan`.

- [ ] **Step 4: Verify RViz YAML contracts**

Run the same three pytest files. Expected: PASS, with each view containing RobotModel, point cloud, costmaps, and planning paths.

- [ ] **Step 5: Commit visualization changes**

```bash
git add \
  track_robot_ws/src/track_robot/track_robot_bringup/rviz/semantic_search_phase4.rviz \
  track_robot_ws/src/track_robot/track_robot_bringup/rviz/semantic_search_phase4b.rviz \
  track_robot_ws/src/track_robot/track_robot_bringup/rviz/semantic_search_phase5a.rviz \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4a_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py
git commit -m "fix(rviz): pin sensor and costmap QoS"
```

---

### Task 6: Build, Regression, Runtime Evidence, and Operator Guide

**Files:**
- Modify: `track_robot_ws/docs/guides/semantic-search/phase2-recording-and-evaluation.md`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_quick_start_doc.py`
- Create: `docs/validation/2026-08-09-canonical-sensor-tf-rviz-validation.md`

**Interfaces:**
- Consumes: the five implementation commits above and physically connected ZED/RoboSense hardware.
- Produces: one repeatable Phase 1–5 command sequence and a no-motion evidence record with commit SHA, environment, topic rates, frames, TF checks, and RViz observations.

- [ ] **Step 1: Add failing documentation contract assertions**

Require the guide to contain the exact network preparation, domain, overlay source, and managed command:

```text
sudo ip addr flush dev eth0
sudo ip addr add 192.168.1.102/24 dev eth0
sudo ip link set eth0 up
export ROS_DOMAIN_ID=20
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run track_robot_bringup semantic_search_ctl run phase5a --rotation-supervised
```

Also require a prominent statement that TF/RViz validation does not authorize motion.

- [ ] **Step 2: Run documentation contract and verify RED**

Run:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_quick_start_doc.py
```

Expected: FAIL until the standardized sequence and canonical frame checks are documented.

- [ ] **Step 3: Update the standard operator guide**

Document one sequence only: stop old managed nodes, configure `eth0`, source Foxy and the worktree overlay, export domain 20, run the managed command, enter the target, and perform observation-only TF/RViz checks before enabling any operator action.

- [ ] **Step 4: Build the three affected packages into the existing worktree overlay**

From `/home/track-robot/track_robot_ws/.worktrees/main-integration` run:

```bash
source /opt/ros/foxy/setup.bash
colcon build \
  --base-paths src track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --packages-select bunker_pro2 track_robot_sensor_bringup track_robot_bringup \
  --symlink-install
```

Expected: all three packages finish successfully.

- [ ] **Step 5: Run package and cross-phase regression tests**

Run:

```bash
source /opt/ros/foxy/setup.bash
source track_robot_ws/install/setup.bash
colcon test \
  --base-paths src track_robot_ws/src \
  --build-base track_robot_ws/build \
  --install-base track_robot_ws/install \
  --packages-select bunker_pro2 track_robot_sensor_bringup track_robot_bringup \
  --event-handlers console_direct+
colcon test-result --test-result-base track_robot_ws/build --verbose
```

Expected: zero failed tests. Then run `colcon test` for `track_robot_interfaces`, `track_robot_lidar_tracking`, `track_robot_navigation`, `track_robot_safety`, `track_robot_semantic_memory`, `track_robot_semantic_search`, and `track_robot_semantic_search_rviz_plugins`; record exact pass/fail totals rather than copying an earlier count.

- [ ] **Step 6: Prove fail-fast behavior without changing the NIC**

With `eth0` missing `192.168.1.102/24`, run the launch in a no-motion configuration and capture the error. Expected: it exits before RoboSense/ZED/Nav2 nodes remain running and prints all three recovery commands. Restore the NIC only with the explicit operator commands.

- [ ] **Step 7: Run hardware validation without sending motion commands**

After sourcing the overlay and setting domain 20, collect:

```bash
ros2 topic info /rslidar_points --verbose
ros2 topic echo /rslidar_points --qos-reliability best_effort --qos-durability volatile | head -n 20
timeout 15 ros2 topic hz /rslidar_points
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 run tf2_ros tf2_echo base_link zed_left_camera_optical_frame
ros2 run tf2_ros tf2_echo camera_link zed_left_camera_frame
ros2 run tf2_ros tf2_echo camera_link zed_right_camera_frame
ros2 topic info /global_costmap/costmap --verbose
ros2 topic info /local_costmap/costmap --verbose
```

Expected evidence:

- `/rslidar_points`: one publisher, `header.frame_id=lidar_link`, sustained rate above 10 Hz;
- left/right transforms `(0,+0.06,0)` and `(0,-0.06,0)` within floating-point tolerance;
- `base_link -> lidar_link` and `base_link -> zed_left_camera_optical_frame` resolve;
- no `base_to_rslidar_tf` or `base_to_zed_camera_tf` node;
- RViz displays robot, point cloud, path, and valid costmaps;
- no executable goal or non-zero velocity is sent.

- [ ] **Step 8: Write the validation record with only observed facts**

Record:

```markdown
- Commit SHA:
- Ubuntu / ROS distro / ROS_DOMAIN_ID:
- Build command and result:
- Test commands and exact totals:
- /rslidar_points publisher count, frame, and observed min/mean/max rate:
- TF results:
- Costmap publishers and RViz subscription/display result:
- Remaining blocker or `none observed`:
- Motion commands sent: none
```

Do not claim the RViz issue fixed if either costmap has no messages, RViz has no subscription, or LiDAR TF is unavailable.

- [ ] **Step 9: Run final cleanliness and regression gates**

```bash
git diff --check
git status --short
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  src/bunker_pro2/test/test_description_contract.py \
  track_robot_ws/src/track_robot/track_robot_sensor_bringup/test/test_sensor_bringup_contract.py \
  track_robot_ws/src/track_robot/track_robot_bringup/test
```

Expected: no whitespace errors, only intended documentation changes uncommitted, and all focused tests PASS.

- [ ] **Step 10: Commit documentation and evidence**

```bash
git add \
  docs/validation/2026-08-09-canonical-sensor-tf-rviz-validation.md \
  track_robot_ws/docs/guides/semantic-search/phase2-recording-and-evaluation.md \
  track_robot_ws/src/track_robot/track_robot_bringup/test/test_quick_start_doc.py \
git commit -m "docs: standardize canonical sensor validation"
```

---

## Final Regression Gates

Reject or revert the responsible independent commit if any of these occur:

- duplicate or disconnected camera/LiDAR TF branches;
- changed calibrated transform values outside the two explicitly approved camera connector transforms;
- `/rslidar_points` frame differs from `lidar_link` or has more/less than one publisher in the managed stack;
- Phase 4B/5A reports ready without LiDAR, required odometry, semantic outputs, or canonical TFs;
- an IMU becomes required by Phase 4B/5A;
- RViz loses RobotModel, semantic overlay, Phase 4A path, Nav2 path, or costmap displays;
- any public semantic topic/message/service/ID or the safety/velocity chain changes;
- any test regression or unexplained topic-rate regression;
- validation sends a motion command.
