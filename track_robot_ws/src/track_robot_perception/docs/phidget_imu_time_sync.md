# PhidgetSpatial IMU time synchronization

The PhidgetSpatial 1056 cannot join PTP directly. It has no Ethernet, PPS, or
external trigger input. Synchronization is therefore split across layers:

1. `ptp4l` synchronizes the Jetson NIC PHC with the PTP grandmaster.
2. `phc2sys` disciplines Jetson `CLOCK_REALTIME` from that PHC.
3. The RoboSense driver uses the LiDAR clock (`use_lidar_clock: true`).
4. `phidget_spatial_imu_node` maps the Phidget's free-running sample clock onto
   Jetson `CLOCK_REALTIME`.

PTP should run as host systemd services. It should not be started by a ROS
node: sensor processes may restart, while clock discipline must remain stable.

The IMU node publishes:

- `/imu/data_raw` (`sensor_msgs/msg/Imu`), SI units, no orientation estimate
- `/imu/mag` (`sensor_msgs/msg/MagneticField`), tesla
- `/imu/time_sync_status` (`diagnostic_msgs/msg/DiagnosticArray`)

For LiDAR-inertial odometry, `imu_lio_adapter_node` republishes:

- `/imu/data_lio` (`sensor_msgs/msg/Imu`), rotated into the LiDAR/body frame

The adapter applies:

```text
corrected IMU stamp = raw IMU stamp - time_offset_sec
```

Point-LIO subscribes to `/imu/data_lio` by default.

Start it with:

```bash
ros2 launch track_robot_perception phidget_imu.launch.py
```

Check the data and synchronization:

```bash
ros2 topic hz /imu/data_raw
ros2 topic echo /imu/time_sync_status
```

The driver receives multiple 4 ms samples in each USB packet. It uses the
newest sample in each packet as a clock observation, rejects delayed USB
observations through a lower-envelope fit, and estimates both clock offset and
clock-rate error. `calibrated_time_offset_sec` is reserved for the remaining
constant sensor/LiDAR delay measured by LI-Init or offline motion correlation.

Do not publish a static LiDAR-to-IMU transform until it has been measured.
FAST-LIO expects the LiDAR pose expressed in the IMU body frame.
