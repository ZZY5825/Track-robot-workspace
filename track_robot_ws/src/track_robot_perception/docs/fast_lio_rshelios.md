# FAST-LIO with RS-Helios and Phidget IMU

The official `hku-mars/FAST_LIO` ROS 2 branch is vendored at:

`src/third_party_ros/fast_lio`

Robot-specific configuration stays in `track_robot_perception`. The selected
configuration sets `feature_extract_enable: true`, which runs the original
feature-based FAST-LIO behavior rather than FAST-LIO2's direct raw-point mode.

## Prerequisites

- `/rslidar_points` must contain `x`, `y`, `z`, `intensity`, `ring`, and
  absolute `FLOAT64 timestamp` fields.
- The RoboSense driver must use `ts_first_point: true`.
- LiDAR and IMU message stamps must share the Jetson/PTP clock domain.
- Measure the LiDAR pose in the IMU frame and replace `mapping.extrinsic_T`
  and `mapping.extrinsic_R`. The identity values are placeholders.

For a PTP-synchronized LiDAR, set `use_lidar_clock: true` in the RoboSense
driver only after `ptp4l` and `phc2sys` are confirmed locked.

## Run

Start the RoboSense driver, then:

```bash
source /opt/ros/foxy/setup.bash
source /home/track-robot/track_robot_ws/install/setup.bash
ros2 launch track_robot_perception fast_lio_rshelios.launch.py
```

The launch starts the Phidget IMU by default. Use `start_imu:=false` if the IMU
is already running from another bringup launch.

Useful checks:

```bash
ros2 topic hz /rslidar_points
ros2 topic hz /imu/data_raw
ros2 topic echo /Odometry --once
```

Do not enable FAST-LIO's software `time_sync_en` when PTP/system-clock
synchronization is active.
