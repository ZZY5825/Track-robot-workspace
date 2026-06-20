# Releases

## V1.2.0 - 2026-06-20

This release adds a ROS 2 Foxy Point-LIO path for the RoboSense RS-Helios-32
and expands the LiDAR/IMU calibration and diagnosis workflow. Point-LIO is
compile-validated, but its extrinsics, time offset, and IMU noise parameters
must still be calibrated and field-validated before its odometry is trusted.

### Point-LIO And LIO Workflows

- Added a local ROS 2 Foxy port of Point-LIO under
  `track_robot_ws/src/third_party_ros/point_lio` using `ament_cmake`, `rclcpp`,
  ROS 2 message aliases, and a `tf2_ros` broadcaster bridge.
- Added direct RS-Helios-32 `PointCloud2` support using the recorded
  `x/y/z/intensity/ring/timestamp` layout.
- Added Point-LIO launch, calibrated and LiDAR-only configs, RViz setup, IMU
  frame/time adapter, and optional RoboSense field adapter.
- Added LIO bag analysis and coarse/fine IMU time-offset sweep utilities for
  repeatable drift investigations.
- Added focused documentation covering topics, launch commands, output topics,
  calibration placeholders, build steps, and remaining field-validation risk.

### FAST-LIO And IMU Updates

- Updated FAST-LIO RoboSense preprocessing to read the native Helios-32 point
  fields with ROS 2 `PointCloud2` iterators and preserve per-point timestamps.
- Made FAST-LIO IMU initialization frame count configurable and expanded its
  initialization diagnostics.
- Added separate FAST-LIO mapping launch support and updated RS-Helios tuning
  and operational documentation.
- Added Phidget IMU attachment timeout, acceleration/gyro bias and scale
  correction, acceleration deadband, static sampling, six-face calibration,
  and static-data validation tools.

### Validation Status

- Point-LIO and `track_robot_perception` are intended to build together with:

  ```bash
  colcon build --symlink-install --packages-select point_lio track_robot_perception
  ```

- The Point-LIO port is compile-validated, not yet field-validated. Measure the
  LiDAR-to-IMU transform and validate timing/noise settings on recorded and
  live data before relying on mapping or odometry output.

## 2026-06-16 Local Workspace Sync

This release updates the GitHub repository to match the current local
`track_robot_ws` source workspace. The previous GitHub-only GUI teleop package
was removed because it is not part of the active local robot workspace.

### Added Or Updated Packages

- `track_robot_perception`: ROS 2 Python perception package for LiDAR, ZED2i,
  fusion, IMU, and pretrained-model experiments.
- `lidar_mos_filter`: C++ range-image moving-object segmentation style filter
  for separating likely static and dynamic LiDAR points.
- `track_robot_core`: shared C++ utilities, ZED2i configs, YOLO/DINO-related
  local support files, and command velocity gating.
- `track_robot_bringup`: Jetson base and RoboSense launch/config updates.
- `track_robot_interfaces`: local interface changes matching the active
  workspace.
- `third_party_ros`: local third-party ROS dependencies for RoboSense,
  FAST-LIO, Bunker, UGV SDK, and ZED2i support.

### Algorithms And Perception Features

- Adaptive ground segmentation with lowest-point seed selection, RANSAC plane
  estimation, SVD plane refinement, tilt limiting, and a fixed-height fallback.
- LiDAR human-candidate segmentation using voxel sampling, DBSCAN clustering,
  height/footprint filters, PCA verticality, and ground-contact checks.
- LiDAR-only object clustering with finite/range/ROI filtering, voxel
  downsampling, DBSCAN or Euclidean clustering, markers, debug JSON, and
  optional colored cluster clouds.
- ZED2i Detectron2 Mask R-CNN instance segmentation with annotated image and
  compact JSON detection output.
- ZED2i Keypoint R-CNN human pose estimation with COCO keypoint metadata.
- RF-DETR Small detector wrapper with runtime-only import so ROS Foxy builds
  remain usable on Python 3.8 systems.
- DINOv3 ViT-S+/16 feature extraction prototype with feature heatmaps, debug
  metadata, optional token export, and a Python 3.8-compatible local DINOv3
  checkout.
- LiDAR mask projection that transforms RoboSense points into the ZED2i camera,
  samples Detectron2 instance masks, and publishes semantic `PointCloud2`
  fields for class, instance, confidence, and RViz color.
- Phidget Spatial IMU ROS node plus documentation for LiDAR/IMU time sync.
- FAST-LIO RS-Helios launch/config workflow and supporting documentation.

### Repository Hygiene

- Removed generated `build/`, `install/`, and `log/` directories from the
  GitHub source tree.
- Added ignore rules for ROS build products, Python caches, local datasets,
  model outputs, bags, and large ML artifacts.
- The local DINOv3 checkpoint
  `models/dinov3_vits16plus_pretrain_lvd1689m.pth` is not committed because it
  exceeds GitHub's normal per-file Git push limit.
