# Point-LIO RS-Helios Integration Notes

## Current Status

`hku-mars/Point-LIO` has been copied into:

```text
src/third_party_ros/point_lio
```

and ported locally enough to build on ROS 2 Foxy with `rclcpp`. The integration
launch/config live in `track_robot_perception`:

```text
config/point_lio_rshelios.yaml
launch/point_lio_rshelios.launch.py
```

The default configuration now uses Point-LIO's `HESAIxt32` PointCloud2 path
directly for RoboSense Helios-32.

## Required Topics

```text
/rslidar_points   sensor_msgs/msg/PointCloud2, frame rslidar
/imu/data_raw     sensor_msgs/msg/Imu, frame imu_link or current Phidget IMU frame
/imu/data_lio     sensor_msgs/msg/Imu, frame rslidar, adapter output for Point-LIO
```

Recorded bags in `~/track_robot_bags` show the Helios cloud layout as:

```text
x FLOAT32
y FLOAT32
z FLOAT32
intensity FLOAT32
ring UINT16
timestamp FLOAT64
```

That matches the Point-LIO `HESAIxt32` branch, so the adapter is disabled by
default.

## Launch

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20

ros2 launch track_robot_perception point_lio_rshelios.launch.py
```

To start LiDAR and IMU from the same launch file:

```bash
sudo -v
ros2 launch track_robot_perception point_lio_rshelios.launch.py \
  start_lidar:=true \
  start_imu:=true
```

With `start_lidar:=true`, this launch includes
`track_robot_bringup/rslidar_with_tf.launch.py`. Network setup defaults to true
and performs the proven sequence `ip addr flush`, `ip addr add
192.168.1.102/24`, then `ip link set eth0 up` before starting the SDK. Pass
`configure_network:=false` only when the interface has already been prepared.

The launch starts `imu_lio_adapter_node` by default. It rotates `/imu/data_raw`
into the LiDAR/body frame, publishes `/imu/data_lio`, and applies the convention:

```text
corrected IMU stamp = raw IMU stamp - imu_time_offset_sec
```

For a quick time-offset test:

```bash
ros2 launch track_robot_perception point_lio_rshelios.launch.py \
  imu_time_offset_sec:=0.020
```

The old adapter remains available for ports that expect a Velodyne-style
`time` field:

```bash
ros2 launch track_robot_perception point_lio_rshelios.launch.py \
  start_adapter:=true \
  config_file:=/path/to/adapter_based_point_lio.yaml
```

## Parameters Still Needing Calibration

`config/point_lio_rshelios.yaml` contains placeholders for:

```text
mapping/extrinsic_T
mapping/extrinsic_R
mapping/extrinsic_est_en
mapping/acc_cov_input
mapping/gyr_cov_input
mapping/imu_meas_acc_cov
mapping/imu_meas_omg_cov
mapping/satu_acc
mapping/satu_gyro
mapping/acc_norm
common/time_diff_lidar_to_imu
```

Point-LIO defines the extrinsic as the LiDAR pose expressed in the IMU body
frame. The current value is identity and must be replaced with the measured
`rslidar` to IMU transform before trusting odometry.

## Drift Debug Workflow

Record a repeatable straight-out-and-back test:

```bash
ros2 bag record -o ~/track_robot_bags/point_lio_drift_debug_$(date +%Y%m%d_%H%M%S) \
  /rslidar_points \
  /imu/data_raw \
  /imu/data_lio \
  /imu/time_sync_status \
  /cloud_registered \
  /cloud_registered_body \
  /Laser_map \
  /aft_mapped_to_init \
  /path \
  /tf \
  /tf_static
```

Analyze the bag:

```bash
ros2 run track_robot_perception analyze_lio_bag.py \
  --bag ~/track_robot_bags/<bag_dir>
```

Generate first-pass offset sweep commands:

```bash
ros2 run track_robot_perception point_lio_offset_sweep.py --mode coarse
```

After finding the best coarse offset, refine around it:

```bash
ros2 run track_robot_perception point_lio_offset_sweep.py \
  --mode fine \
  --center 0.020
```

Run a LiDAR-only control by switching the config:

```bash
ros2 launch track_robot_perception point_lio_rshelios.launch.py \
  config_file:=install/track_robot_perception/share/track_robot_perception/config/point_lio_rshelios_lidar_only.yaml
```

## Expected Output Topics

Upstream-compatible output names are preserved:

```text
/cloud_registered
/cloud_registered_body
/Laser_map
/aft_mapped_to_init
/path
```

Useful checks:

```bash
ros2 topic hz /rslidar_points
ros2 topic hz /imu/data_raw
ros2 topic list | grep -E 'cloud|Laser_map|aft_mapped|path'
```

## Build

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select point_lio track_robot_perception
```
