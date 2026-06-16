# Releases

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
