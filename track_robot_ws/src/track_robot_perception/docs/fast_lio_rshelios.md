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

## Recommended run order

Run the sensors independently first, then start FAST-LIO as a subscriber-only
mapping node. This avoids two practical startup problems on the robot:

- The Phidget IMU is more reliable when started alone.
- The RS-Helios driver sometimes reports `MSOPTIMEOUT` until the Ethernet
  interface has been reset and the driver has been restarted.

Terminal 1: start the Phidget IMU and leave it running:

```bash
source /opt/ros/foxy/setup.bash
source /home/track-robot/track_robot_ws/install/setup.bash
ROS_DOMAIN_ID=20 PYTHONWARNINGS=ignore::DeprecationWarning \
  ros2 run track_robot_perception phidget_spatial_imu_node
```

Terminal 2: reset the LiDAR Ethernet interface and start the RoboSense driver:

```bash
cd /home/track-robot/track_robot_ws
sudo ip addr flush dev eth0
sudo ip addr add 192.168.1.102/24 dev eth0
sudo ip link set eth0 up
source /opt/ros/foxy/setup.bash
source install/setup.bash
sudo -v
ROS_DOMAIN_ID=20 PYTHONWARNINGS=ignore::DeprecationWarning \
  ros2 launch track_robot_bringup rslidar_with_tf.launch.py configure_network:=false
```

Terminal 3: after `/imu/data_raw` and `/rslidar_points` are publishing, start
FAST-LIO:

```bash
source /opt/ros/foxy/setup.bash
source /home/track-robot/track_robot_ws/install/setup.bash
ROS_DOMAIN_ID=20 PYTHONWARNINGS=ignore::DeprecationWarning \
  ros2 launch track_robot_perception fast_lio_rshelios_mapping.launch.py rviz:=true
```

The older `fast_lio_rshelios.launch.py` is now safe by default too: it starts
FAST-LIO only, unless `start_imu:=true`, `start_lidar:=true`, or
`configure_network:=true` are explicitly passed.

Useful checks:

```bash
ROS_DOMAIN_ID=20 PYTHONWARNINGS=ignore::DeprecationWarning ros2 topic hz /rslidar_points
ROS_DOMAIN_ID=20 PYTHONWARNINGS=ignore::DeprecationWarning ros2 topic hz /imu/data_raw
ROS_DOMAIN_ID=20 PYTHONWARNINGS=ignore::DeprecationWarning ros2 topic echo /Odometry
```

Do not enable FAST-LIO's software `time_sync_en` when PTP/system-clock
synchronization is active.

## LiDAR-IMU extrinsic

FAST-LIO does not read the ROS `/tf` tree to learn the transform between the
LiDAR and IMU. The fusion transform is configured directly in
`config/fast_lio_rshelios.yaml`:

- `mapping.extrinsic_T`: LiDAR translation expressed in the IMU frame.
- `mapping.extrinsic_R`: LiDAR rotation expressed in the IMU frame.

The current values are identity placeholders because the physical LiDAR-to-IMU
mount transform has not been measured yet. `extrinsic_est_en: true` lets
FAST-LIO estimate the extrinsic online for early testing, but the proper fix is
to measure the mount transform, put it into the YAML, and then set
`extrinsic_est_en: false`.
