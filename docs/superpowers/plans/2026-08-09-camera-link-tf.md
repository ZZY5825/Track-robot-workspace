# Sensor Station Camera TF Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Publish an empty `camera_link` frame 221.2 mm forward and 318 mm above `sensor_station_link`.

**Architecture:** Add one empty URDF link and one fixed joint. `robot_state_publisher` will publish the relationship without any camera mesh or additional launch node.

**Tech Stack:** ROS 2 Foxy, URDF XML, pytest, check_urdf, colcon

## Global Constraints

- Parent: `sensor_station_link`.
- Child: `camera_link`.
- Fixed-joint name: `sensor_station_camera_joint`.
- Translation: `xyz="0.2212 0 0.318"` metres.
- Rotation: `rpy="0 0 0"`.
- `camera_link` has no visual, collision, inertial, STL, or optical child frame.
- Verification is limited to one contract test, URDF parsing, and package build/test.

---

### Task 1: Add the camera TF

**Files:**
- Modify: `src/bunker_pro2/test/test_description_contract.py`
- Modify: `src/bunker_pro2/urdf/bunker_pro2.urdf`

**Interfaces:**
- Consumes: existing `sensor_station_link`
- Produces: fixed TF `sensor_station_link -> camera_link`

- [ ] **Step 1: Write the failing contract test**

```python
def test_camera_link_is_fixed_to_sensor_station():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    camera_link = robot.find("./link[@name='camera_link']")
    assert camera_link is not None
    assert len(camera_link) == 0

    joint = robot.find("./joint[@name='sensor_station_camera_joint']")
    assert joint is not None
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == 'sensor_station_link'
    assert joint.find('child').attrib['link'] == 'camera_link'
    assert joint.find('origin').attrib == {
        'xyz': '0.2212 0 0.318',
        'rpy': '0 0 0',
    }
```

- [ ] **Step 2: Run the test and verify RED**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/bunker_pro2/test/test_description_contract.py::test_camera_link_is_fixed_to_sensor_station
```

Expected: FAIL because `camera_link` is absent.

- [ ] **Step 3: Add the minimal URDF elements**

Insert before `</robot>`:

```xml
  <link name="camera_link" />
  <joint name="sensor_station_camera_joint" type="fixed">
    <origin xyz="0.2212 0 0.318" rpy="0 0 0" />
    <parent link="sensor_station_link" />
    <child link="camera_link" />
  </joint>
```

- [ ] **Step 4: Verify GREEN, parse, build, and test**

```bash
set -e
source /opt/ros/foxy/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/bunker_pro2/test/test_description_contract.py
check_urdf src/bunker_pro2/urdf/bunker_pro2.urdf
colcon build --symlink-install --packages-select bunker_pro2
source install/setup.bash
colcon test --packages-select bunker_pro2
colcon test-result --test-result-base build/bunker_pro2 --verbose
```

Expected: ten pytest checks pass, `check_urdf` shows `camera_link` beneath `sensor_station_link`, and package tests report 0 failures.

- [ ] **Step 5: Commit**

```bash
git add src/bunker_pro2/test/test_description_contract.py src/bunker_pro2/urdf/bunker_pro2.urdf
git diff --cached --check
git commit -m "feat: add sensor station camera TF"
```
