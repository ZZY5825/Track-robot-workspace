# Bunker Pro 2 RViz2 Viewer

This ROS 2 Foxy package keeps the authoritative combined robot description for
the AgileX Bunker Pro 2, its sensor station, and the front-mounted AgileX PiPER
arm. It also provides a repeatable, motion-free RViz2 viewer.

## Build and launch

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select piper_description bunker_pro2
source install/setup.bash
ros2 launch bunker_pro2 display.launch.py
```

The display launch starts:

- A static `world -> robot_bottom` transform for the standalone viewer.
- One `joint_state_publisher` that consumes `/joint_states_single` and supplies
  zero positions until real PiPER feedback arrives.
- One `robot_state_publisher` with the combined URDF.
- RViz2 with a transient-local `/robot_description` subscription and `world`
  as its fixed frame.

These nodes publish model state only. They do not start the Bunker or PiPER
drivers and do not publish motion commands. Gazebo physics and track
articulation are outside this package.

The semantic-search stack includes `description.launch.py` directly, without
the standalone `world` transform or a second RViz instance. Its rigid TF chain
is:

```text
robot_bottom
`-- base_link
    |-- sensor_station_link
    |   |-- camera_mount_link
    |   |   `-- zed_camera_link
    |   `-- lidar_link
    `-- arm_base_link
        `-- link1 ... link6
            `-- gripper_base
                |-- camera_holder -> l515_visual
                |-- link7
                `-- link8
```

`robot_bottom` has the same x/y axes as `base_link`; `base_link` is exactly
0.45 m above it. The calibrated sensor-station, camera and LiDAR transforms are
preserved unchanged.

## PiPER arm mount and source

The PiPER root is renamed from its standalone `base_link` to
`arm_base_link`. Its only connection to the mobile robot is:

```text
base_link -> arm_base_link
xyz = 0.39 0 0.016 m
rpy = 0 0 0 rad
```

This places the arm at the longitudinal front of the top rails, centred at
`y=0`, and at the same rail height as the sensor-station root. The PiPER
internal joints remain `joint1` through `joint8`, so the model accepts the
existing `/joint_states_single` feedback without changing driver contracts.

The model and meshes come from the imported `piper_description` package. The
hash-locked source remains:

```text
piper_description.xacro
e32d340b72389d237fb367ad700af9de34970fe987aa7eb8bb795c6b2e2f35e1
camera_holder.STL
a68851c67c3c631b3176d1038478a37acd5b2ebc6aa6189793df4b6ee68478b2
Intel_RealSense_L515_CAD_external.STL
8da72869225af4826ed8b059361109b9e18df5295624d629109e32a906f02d6f
```

The downloaded handoff contained one stale L515 visual-position expectation
in its test file. The imported test expectation is synchronized to the
hash-locked Xacro; the Xacro and all mesh geometry are unchanged.

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
