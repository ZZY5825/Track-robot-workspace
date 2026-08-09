# Bunker Pro 2 Sensor Station Mount Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `FullCase.STL` as a sensor-station link rigidly centred on the Bunker Pro 2 top rail with roll +90 degrees and yaw 180 degrees, then verify the result in RViz2.

**Architecture:** Keep the sensor station separate from the upstream `base_link` mesh. A fixed URDF joint places a semantic `sensor_station_link` frame at the rail midpoint, while the link's visual origin removes the STL's x/y corner-origin offset and its mesh scale converts millimetres to metres.

**Tech Stack:** ROS 2 Foxy, URDF XML, `ament_cmake`, `pytest`, `check_urdf`, `robot_state_publisher`, RViz2

## Global Constraints

- After rotation, 537.5 mm runs along robot x, 467 mm runs along robot y, and 447.55 mm points upward along robot z.
- The fixed joint origin is exactly `xyz="-0.0075 0 0.016" rpy="1.57079632679 0 3.14159265359"`.
- The visual origin is exactly `xyz="-0.26875 0 -0.2335" rpy="0 0 0"`.
- The STL scale is exactly `0.001 0.001 0.001`.
- This version adds visual geometry only; it does not add collision or inertial properties for the sensor station.
- Generated `build/`, `install/`, and `log/` directories must remain untracked.

## File map

- Add `src/bunker_pro2/meshes/FullCase.STL`: user-supplied visual mesh.
- Modify `src/bunker_pro2/urdf/bunker_pro2.urdf`: sensor link, transform, and fixed joint.
- Modify `src/bunker_pro2/test/test_description_contract.py`: mount contract tests.
- Modify `src/bunker_pro2/README.md`: mount documentation.
- Add `artifacts/bunker_pro2/rviz-bunker-pro2-sensor-station.png`: RViz evidence.

---

### Task 1: Add centred sensor-station visual link

**Files:**
- Add: `src/bunker_pro2/meshes/FullCase.STL`
- Modify: `src/bunker_pro2/urdf/bunker_pro2.urdf`
- Test: `src/bunker_pro2/test/test_description_contract.py`

**Interfaces:**
- Consumes: `/home/track-robot/track_robot_ws/src/bunker_pro2/meshes/FullCase.STL`
- Produces: `sensor_station_link` with a scaled and centred visual mesh

- [ ] **Step 1: Write the failing visual test**

Append:

```python
def test_sensor_station_visual_is_scaled_and_centered():
    mesh_path = PACKAGE_ROOT / 'meshes' / 'FullCase.STL'
    assert mesh_path.is_file()
    assert mesh_path.stat().st_size > 1024

    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    link = robot.find("./link[@name='sensor_station_link']")
    assert link is not None
    origin = link.find('./visual/origin')
    assert origin.attrib == {
        'xyz': '-0.26875 0 -0.2335',
        'rpy': '0 0 0',
    }
    mesh = link.find('./visual/geometry/mesh')
    assert mesh.attrib['filename'] == (
        'package://bunker_pro2/meshes/FullCase.STL'
    )
    assert mesh.attrib['scale'] == '0.001 0.001 0.001'
```

- [ ] **Step 2: Run it and verify RED**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/bunker_pro2/test/test_description_contract.py::test_sensor_station_visual_is_scaled_and_centered
```

Expected: FAIL because the asset and link are absent from the isolated worktree.

- [ ] **Step 3: Add the asset and minimal visual link**

Copy the binary STL:

```bash
cp /home/track-robot/track_robot_ws/src/bunker_pro2/meshes/FullCase.STL src/bunker_pro2/meshes/FullCase.STL
```

Insert before `</robot>`:

```xml
  <link name="sensor_station_link">
    <visual>
      <origin xyz="-0.26875 0 -0.2335" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://bunker_pro2/meshes/FullCase.STL"
              scale="0.001 0.001 0.001" />
      </geometry>
      <material name="sensor_station_gray">
        <color rgba="0.35 0.35 0.35 1" />
      </material>
    </visual>
  </link>
```

- [ ] **Step 4: Re-run the visual test and verify GREEN**

Run the Step 2 command. Expected: `1 passed`.

- [ ] **Step 5: Write the failing fixed-joint test**

Append:

```python
def test_sensor_station_is_fixed_to_top_rail_midpoint():
    robot = ET.parse(str(PACKAGE_ROOT / 'urdf' / 'bunker_pro2.urdf')).getroot()
    joint = robot.find("./joint[@name='sensor_station_joint']")
    assert joint is not None
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == 'base_link'
    assert joint.find('child').attrib['link'] == 'sensor_station_link'
    assert joint.find('origin').attrib == {
        'xyz': '-0.0075 0 0.016',
        'rpy': '1.57079632679 0 3.14159265359',
    }
```

- [ ] **Step 6: Run it and verify RED**

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/bunker_pro2/test/test_description_contract.py::test_sensor_station_is_fixed_to_top_rail_midpoint
```

Expected: FAIL because `sensor_station_joint` is absent.

- [ ] **Step 7: Add the minimal fixed joint**

Insert after `sensor_station_link`:

```xml
  <joint name="sensor_station_joint" type="fixed">
    <origin xyz="-0.0075 0 0.016"
            rpy="1.57079632679 0 3.14159265359" />
    <parent link="base_link" />
    <child link="sensor_station_link" />
  </joint>
```

- [ ] **Step 8: Verify all tests and URDF parsing**

```bash
source /opt/ros/foxy/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/bunker_pro2/test/test_description_contract.py
check_urdf src/bunker_pro2/urdf/bunker_pro2.urdf
```

Expected: `9 passed`; root `base_link` has child `sensor_station_link`.

- [ ] **Step 9: Commit the feature**

```bash
git add src/bunker_pro2/meshes/FullCase.STL src/bunker_pro2/urdf/bunker_pro2.urdf src/bunker_pro2/test/test_description_contract.py
git diff --cached --check
git commit -m "feat: mount sensor station on Bunker Pro 2"
```

### Task 2: Build and verify the package

**Files:**
- Verify: `src/bunker_pro2/CMakeLists.txt`
- Verify: `src/bunker_pro2/launch/display.launch.py`

**Interfaces:**
- Consumes: the link and joint from Task 1
- Produces: installed TF chain `world -> base_link -> sensor_station_link`

- [ ] **Step 1: Build**

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select bunker_pro2
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 2: Run package tests**

```bash
source install/setup.bash
colcon test --packages-select bunker_pro2 --event-handlers console_cohesion+
colcon test-result --test-result-base build/bunker_pro2 --verbose
```

Expected: 0 errors and 0 failures.

- [ ] **Step 3: Launch RViz2 and verify TF**

Launch in one terminal:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch bunker_pro2 display.launch.py
```

Verify in another:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
timeout 8 ros2 run tf2_ros tf2_echo base_link sensor_station_link
```

Expected: translation approximately `[-0.0075, 0.0, 0.016]`; rotation corresponds to roll +90 degrees and yaw 180 degrees.

### Task 3: Capture evidence and document the mount

**Files:**
- Add: `artifacts/bunker_pro2/rviz-bunker-pro2-sensor-station.png`
- Modify: `src/bunker_pro2/README.md`

**Interfaces:**
- Consumes: running RViz2 model from Task 2
- Produces: screenshot evidence and user documentation

- [ ] **Step 1: Inspect and capture RViz2**

Frame the complete robot, confirm `Global Status: Ok`, and run:

```bash
gnome-screenshot -f artifacts/bunker_pro2/rviz-bunker-pro2-sensor-station.png
```

Expected: the station has roll +90 degrees and yaw 180 degrees, is centred in x/y, rests on the rail, and is fully visible.

- [ ] **Step 2: Document the mount**

Append to `src/bunker_pro2/README.md`:

```markdown
## Sensor station mount

`FullCase.STL` is mounted as `sensor_station_link` with a fixed joint at the
centre of the built-in top rail. The mesh is converted from millimetres to
metres in URDF and centred without modifying the supplied STL asset.

Visual verification: `artifacts/bunker_pro2/rviz-bunker-pro2-sensor-station.png`.
```

- [ ] **Step 3: Run decisive verification**

```bash
set -e
source /opt/ros/foxy/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/bunker_pro2/test/test_description_contract.py
check_urdf src/bunker_pro2/urdf/bunker_pro2.urdf
colcon build --symlink-install --packages-select bunker_pro2
source install/setup.bash
colcon test --packages-select bunker_pro2 --event-handlers console_cohesion+
colcon test-result --test-result-base build/bunker_pro2 --verbose
git diff --check
```

Expected: nine pytest checks pass, URDF parsing and build succeed, and test results report 0 failures.

- [ ] **Step 4: Commit documentation and evidence**

```bash
git add src/bunker_pro2/README.md artifacts/bunker_pro2/rviz-bunker-pro2-sensor-station.png
git diff --cached --check
git commit -m "docs: verify Bunker sensor station in RViz2"
```
