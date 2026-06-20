# Point-LIO ROS 2 Foxy Port Assessment

## Result

No maintained ROS 2 Foxy Point-LIO port was found locally. The official
`hku-mars/Point-LIO` repository is ROS 1/catkin, so a local ROS 2 migration was
made under:

```text
src/third_party_ros/point_lio
```

This local port builds on ROS 2 Foxy:

```bash
colcon build --symlink-install --packages-select point_lio
```

## What Was Ported

The package now uses `ament_cmake` and `rclcpp`. A small compatibility layer was
added under `point_lio/include` so most algorithm code can keep the upstream
ROS 1-style names while compiling against ROS 2 messages.

Main changes:

```text
catkin -> ament_cmake
ros::NodeHandle -> thin rclcpp wrapper
ros::Publisher / ros::Subscriber -> rclcpp publisher/subscription wrapper
ros::Time().fromSec and stamp.toSec -> ROS2-compatible helpers
tf::TransformBroadcaster -> tf2_ros bridge wrapper
sensor_msgs/nav_msgs/geometry_msgs ROS1 include names -> ROS2 message aliases
```

Livox `CustomMsg` is stubbed only so the upstream source compiles. This robot's
integration path is the standard `sensor_msgs/msg/PointCloud2` Helios-32 input,
not Livox.

## Helios-32 Compatibility

The recorded RoboSense bags show:

```text
height: 900
width: 32
point_step: 26
fields:
  x FLOAT32 offset 0
  y FLOAT32 offset 4
  z FLOAT32 offset 8
  intensity FLOAT32 offset 12
  ring UINT16 offset 16
  timestamp FLOAT64 offset 18
frame_id: rslidar
ring range: 0..31
per-scan timestamp span: about 0.05 s
```

This matches Point-LIO's existing `HESAIxt32` point struct:

```text
x, y, z, intensity, timestamp, ring
```

So the active config uses:

```text
common/lid_topic: /rslidar_points
preprocess/lidar_type: 4
preprocess/scan_line: 32
preprocess/scan_rate: 20
mapping/lidar_time_inte: 0.05
```

## Remaining Risks

The port is compile-validated, not yet field-validated. Before trusting
odometry, verify:

```text
LiDAR-IMU extrinsic_T/R
LiDAR-IMU time offset
Phidget IMU axis convention and frame_id
IMU noise/covariance parameters
startup with robot stationary for initialization
```

The bag files found under `~/track_robot_bags` contain LiDAR-only or
LiDAR+odom/tf data, but no `/imu/data_raw`, so they can verify point cloud
field compatibility but cannot fully validate Point-LIO odometry.
