# Bunker Pro 2 RViz2 Viewer

This ROS 2 Foxy package keeps a local copy of the AgileX Bunker Pro 2 URDF and
mesh and provides a repeatable RViz2 viewer.

## Build and launch

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select bunker_pro2
source install/setup.bash
ros2 launch bunker_pro2 display.launch.py
```

The launch file starts:

- A static `world -> base_link` transform, required because the upstream model
  contains one root link and no joints.
- `robot_state_publisher` with the imported URDF.
- RViz2 with a transient-local `/robot_description` subscription and `world`
  as its fixed frame.

The robot is a rigid visualization model; Gazebo physics, driving, sensors,
and track articulation are outside this package.

## Verified output

The checked RViz2 screenshot is stored at
[`artifacts/bunker_pro2/rviz-bunker-pro2.png`](../../artifacts/bunker_pro2/rviz-bunker-pro2.png).
It shows the Bunker Pro 2 mesh with RViz reporting `Global Status: Ok`.

## Sensor station mount

`FullCase.STL` is mounted as `sensor_station_link` with a fixed joint at the
centre of the built-in top rail. The mesh is converted from millimetres to
metres in URDF and centred without modifying the supplied STL asset.

The sensor-station result is shown in
[`artifacts/bunker_pro2/rviz-bunker-pro2-sensor-station.png`](../../artifacts/bunker_pro2/rviz-bunker-pro2-sensor-station.png).

See [UPSTREAM.md](UPSTREAM.md) for the pinned source revision and license.
