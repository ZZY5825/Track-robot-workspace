# IMU and Point-LIO live debug

Run this node alongside Point-LIO:

```bash
ros2 launch track_robot_perception imu_lio_debug.launch.py
```

It compares `/imu/data_raw`, `/imu/data_lio`, and `/aft_mapped_to_init` and
prints one status line per second. `RUNAWAY` means the raw IMU has been static
for at least one second while Point-LIO body speed remains above `0.2 m/s`.

Useful output topics:

```text
/imu_lio_debug/raw_acceleration
/imu_lio_debug/raw_angular_velocity
/imu_lio_debug/raw_accel_tilt_rpy
/imu_lio_debug/lio_acceleration
/imu_lio_debug/lio_angular_velocity
/imu_lio_debug/lio_accel_tilt_rpy
/imu_lio_debug/body_rpy
/imu_lio_debug/body_velocity
/imu_lio_debug/body_speed
/imu_lio_debug/stationary
/imu_lio_debug/runaway
/imu_lio_debug/status
```

The RPY vector topics use radians. Plot the key failure signals with:

```bash
rqt_plot \
  /imu_lio_debug/raw_accel_tilt_rpy/vector/x \
  /imu_lio_debug/raw_accel_tilt_rpy/vector/y \
  /imu_lio_debug/body_rpy/vector/x \
  /imu_lio_debug/body_rpy/vector/y \
  /imu_lio_debug/body_speed/data
```

For a stationary calibration estimate, keep the assembly still and run:

```bash
ros2 launch track_robot_perception imu_lio_debug.launch.py \
  calibrate_on_start:=true
```

After ten stationary seconds, suggestions are written to:

```text
/tmp/imu_lio_stationary_calibration.json
```

One pose can estimate residual gyro bias and a uniform accelerometer gain only.
For per-axis acceleration calibration, collect all six static faces:

```bash
ros2 run track_robot_perception imu_collect_static_sample --label x_plus
ros2 run track_robot_perception imu_collect_static_sample --label x_minus
ros2 run track_robot_perception imu_collect_static_sample --label y_plus
ros2 run track_robot_perception imu_collect_static_sample --label y_minus
ros2 run track_robot_perception imu_collect_static_sample --label z_plus
ros2 run track_robot_perception imu_collect_static_sample --label z_minus
ros2 run track_robot_perception imu_six_face_calibrate
```
