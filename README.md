# Track Robot Workspace

ROS 2 Foxy workspace for the tracked robot platform. The current workspace
focuses on Jetson-side robot bringup, RoboSense LiDAR support, Bunker base
interfaces, ZED2i perception, LiDAR-camera fusion, and experimental pretrained
vision/LiDAR perception pipelines.

## Workspace Layout

```text
track_robot_ws/
  src/
    track_robot/                 Core bringup, control, drivers, interfaces
    track_robot_core/            Shared C++ nodes, ZED configs, model configs
    track_robot_perception/      LiDAR, camera, fusion, IMU, and ML perception
    lidar_mos_filter/            Range-image moving-object segmentation filter
    third_party_ros/             FAST-LIO, RoboSense, Bunker, ZED dependencies
  tools/                         Utility scripts
```

Generated ROS output directories such as `build/`, `install/`, and `log/` are
not part of the source release.

## Main Features

- RoboSense RS-Helios LiDAR launch and configuration, including RS-LiDAR plus
  TF bringup.
- Bunker base integration and Jetson-side robot bringup.
- Velocity gate support for safe command filtering.
- FAST-LIO integration tuned for the RS-Helios workflow.
- ROS 2 Foxy Point-LIO port and RS-Helios launch/configuration workflow.
- Phidget Spatial IMU calibration, bias/scale correction, and LiDAR/IMU time
  synchronization tooling.
- LIO bag analysis and coarse/fine IMU time-offset sweep utilities.
- ZED2i RGB perception with Detectron2 Mask R-CNN instance segmentation.
- ZED2i Keypoint R-CNN human pose and skeleton visualization.
- RF-DETR Small detection wrapper, kept runtime-optional for ROS Foxy/Python 3.8.
- DINOv3 ViT-S+/16 feature extraction prototype using a Python 3.8-compatible
  local DINOv3 checkout.
- LiDAR-only geometric clustering baseline using voxel sampling and
  DBSCAN/Euclidean clustering.
- Adaptive LiDAR ground highlighting with RANSAC plane fitting and a fixed
  height fallback.
- LiDAR human-candidate segmentation based on 3D cluster geometry, verticality,
  and local ground contact.
- LiDAR mask projection that projects RoboSense points into ZED2i Mask R-CNN
  masks and publishes semantic point clouds.
- Learning-free range-image MOS-style filter for static/dynamic LiDAR point
  separation.
- Gesture-triggered single-person tracking using YOLO pose for identity,
  camera-guided LiDAR association, persistent C++ LiDAR tracklets, and
  Kalman-filtered camera/LiDAR target state.
- Language-conditioned semantic search with passive OpenAI CLIP perception,
  generalized multi-object 3D semantic memory, deterministic replay, bounded
  task services, and a ROS CLI text-query portal.
- Phase 1 multiscale semantic windows using one bounded six-view GPU batch,
  deterministic duplicate suppression, and whole-frame fallback when no local
  candidate passes.

## Build

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For focused development, build selected packages:

```bash
colcon build --symlink-install --packages-select track_robot_perception
colcon build --symlink-install --packages-select lidar_mos_filter
```

## Common Launch Commands

```bash
ros2 launch track_robot_bringup jetson_base.launch.py
ros2 launch track_robot_bringup rslidar_with_tf.launch.py
ros2 launch track_robot_perception fast_lio_rshelios.launch.py
ros2 launch track_robot_perception point_lio_rshelios.launch.py
ros2 launch track_robot_perception phidget_imu.launch.py
ros2 launch track_robot_perception lidar_ground_segment.launch.py
ros2 launch track_robot_perception lidar_human_segment.launch.py
ros2 launch track_robot_perception lidar_cluster_baseline.launch.py
ros2 launch track_robot_perception lidar_mask_projector.launch.py
ros2 launch track_robot_perception zed_mask_rcnn.launch.py
ros2 launch track_robot_perception zed_pose_rcnn.launch.py
ros2 launch track_robot_perception zed_dinov3_feature.launch.py
ros2 launch lidar_mos_filter range_image_mos_filter.launch.py
ros2 launch track_robot_perception human_tracking_simplified.launch.py
ros2 launch track_robot_semantic_search semantic_search_phase1.launch.py \
  start_perception:=true
ros2 run track_robot_semantic_search semantic_search_query "a red backpack"
```

## Model And Data Notes

Large local checkpoints and datasets are intentionally not committed to Git.
The DINOv3 ViT-S+/16 checkpoint used locally is expected at:

```text
~/track_robot_ws/models/dinov3_vits16plus_pretrain_lvd1689m.pth
```

That checkpoint is larger than GitHub's normal per-file push limit, so keep it
as a local artifact or publish it through a release asset or Git LFS if it needs
to be shared.

## Documentation

- `track_robot_ws/src/track_robot_perception/README.md`
- `track_robot_ws/src/track_robot_perception/docs/fast_lio_rshelios.md`
- `track_robot_ws/src/track_robot_perception/docs/phidget_imu_time_sync.md`
- `track_robot_ws/src/track_robot_perception/docs/point_lio_rshelios.md`
- `track_robot_ws/src/track_robot_perception/docs/point_lio_ros2_port_assessment.md`
- `track_robot_ws/src/track_robot_perception/docs/pretrained_lidar_feasibility.md`
- `track_robot_ws/src/track_robot_perception/docs/human_tracking_fusion_refactor_log_2026-07-09.md`
- `track_robot_ws/src/track_robot_semantic_search/README.md`
- `track_robot_ws/rosbags/semantic_search/phase2_recording_guide.md`
- `RELEASES.md`
