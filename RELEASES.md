# Releases

## V1.4.0 - 2026-07-21

V1.4.0 promotes the Phase 2 semantic-search software preview into an official
source release and retains the alpha architecture, contracts, replay tools,
evaluation gates, and operator documentation. This is a stable source
checkpoint; it does not claim that the outstanding synchronized physical-robot
evaluation has passed.

### Semantic Search And Memory

- Adds language-conditioned passive perception with aligned OpenAI CLIP image
  and text embeddings.
- Adds generalized multi-object 3D semantic memory with camera/LiDAR
  association, appearance history, re-identification, lifecycle management,
  task relevance, and bounded query/inspection/reset services.
- Adds deterministic replay, threshold calibration, quality/resource gates,
  and the Phase 2 recording and evaluation workflow.
- Keeps perception and memory paths fail-closed; semantic search does not
  publish `cmd_vel` or silently enable robot motion.

### Text Query Portal

- Adds the `semantic_search_query` ROS CLI so operators can submit plain text
  without constructing JSON manually.
- Supports one-shot and interactive queries, explicit query IDs and versions,
  acknowledgement diagnostics, bounded subscriber/timeout waits, and clear
  exit codes.
- Preserves the future RViz panel contract without coupling the CLI to RViz.

### Phase 1 Multiscale Improvement

- Replaces grid-only default inference with `multiscale_v1`: the unchanged
  four 2x2 crops, one letterboxed whole frame, and one centered 60% crop are
  encoded in a single bounded GPU batch.
- Treats the center window as a local candidate and the whole frame only as a
  fallback when no local candidate passes.
- Adds deterministic IoU/containment duplicate suppression and one shared
  absolute or quantile cutoff across grid and extra windows.
- Fixes ROS 2 Foxy launch parameter precedence by merging YAML parameters and
  CLI overrides before creating the perception node.
- Keeps ROS messages/topics and the Phase 2 external-proposal pooling contract
  unchanged.

### Verification

- `track_robot_semantic_search`: 482 Python tests passed.
- ROS package result: 484 tests, 0 failures, 0 errors, 0 skipped.
- Real OpenAI CLIP ViT-B/32 checkpoint load passed with the pinned SHA-256
  `40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af`.
- On the Jetson CUDA core path, `multiscale_v1` measured 129.54 ms P95 and
  7.72 Hz capacity across 30 measured iterations, satisfying the 150 ms and
  5 Hz runtime gates.
- The alpha Phase 2 clean-archive checkpoint remains recorded as 599 tests,
  0 failures, 0 errors, with 3 expected DDS skips; explicit DDS validation was
  3/3.

### Remaining Physical Gate

The included
`track_robot_ws/rosbags/semantic_search/reports/phase2_evaluation_2026-07-17.json`
remains `unavailable` because a complete synchronized physical recording has
not yet been collected. Production selection remains fail-closed until the
documented field quality, latency, resource, and regression gates pass. Model
weights, rosbag payloads, build/install products, and generated test caches are
not included in the source release.

## V1.3.0 - 2026-07-09

This release adds the first persistent camera-initialized LiDAR tracking
architecture for gesture-selected human following. The camera and gesture FSM
own target identity. LiDAR supplies 3D geometry and short camera-out-of-view
continuation. Robot motion control remains disabled.

### Camera And Target Identity

- Retains YOLO pose tracking, ByteTrack IDs, and the gesture FSM for selecting
  one person.
- Adds camera target-lock diagnostics for bbox prediction, active tracker ID,
  visibility, confidence, and dropout timing.
- Preserves the logical camera target ID separately from the current LiDAR
  tracklet ID.

### Persistent LiDAR Tracklets

- Adds the C++ `track_robot_lidar_tracking` package.
- Crops, downsamples, and clusters RS-Helios point clouds into weak candidate
  clusters and filtered persistent tracklets.
- Uses range-adaptive point and height limits for close, mid-range, and sparse
  far-range observations.
- Tracks generic clusters with a four-state XY constant-velocity Kalman filter.
- Uses NIS and distance gating plus Hungarian global assignment to reduce ID
  switches.
- Adds maneuver process noise, stationary detection, velocity decay, confidence
  lifecycle management, and retained predicted tracklets.

### Camera-LiDAR Target Fusion

- Projects tracklet boxes and raw LiDAR points into the ZED2i image.
- Selects torso, upper-body, or central-bbox ROIs from YOLO pose observations.
- Uses foreground depth percentiles and robust medians to reduce desk and wall
  contamination.
- Maintains the selected target with a six-state XYZ constant-velocity Kalman
  filter and NIS measurement rejection.
- Continues the exact selected LiDAR tracklet when the camera loses visibility.
- Allows replacement IDs only after three compatible local matches inside the
  prediction gate.
- Dampens unsupported velocity and enters `TARGET_LOST` after the configured
  prediction timeout instead of globally switching to clutter.

### Performance And Diagnostics

- Moves full-cloud camera projection to fresh cloud callbacks and caps it at
  10 Hz.
- Keeps tracklet callbacks lightweight and uses the timer only for prediction
  and publication.
- Adds selected-tracklet, selected-target, prediction-gate, fused-target, and
  camera-guided point-cloud visualization topics.
- Adds structured timing and state diagnostics through
  `/human_tracking/target_tracker_debug`.

### Validation Status

- `track_robot_lidar_tracking` and `track_robot_perception` build successfully
  on ROS2 Foxy.
- Both new C++ nodes pass standalone startup tests with their active YAML files.
- The generic LiDAR tracklet markers and camera-visible selected tracklet were
  observed working in recorded tests.
- Stop/turn relinking, circular camera-out-of-view tracking, and the 10 Hz
  camera-visible acceptance target still require complete four-bag validation.
- The system remains perception-only and does not publish robot control
  commands.

## V1.2.1 - 2026-06-24

This release records the current Point-LIO calibration/debugging state for the
RoboSense Helios-32 and Phidget IMU setup. It improves the IMU frame handling
and adds instrumentation for the remaining near-90-degree fly-away issue; it
does not add reset, clipping, or fallback logic.

### Point-LIO IMU Frame Handling

- Updated `imu_lio_adapter.yaml` to use the selected right-handed Candidate B
  mapping from raw IMU into the LiDAR frame:
  `lidar_x = -imu_y`, `lidar_y = imu_x`, `lidar_z = imu_z`.
- Documented the Plan A convention that `/imu/data_lio` is already expressed in
  the `rslidar` frame, so Point-LIO keeps identity extrinsics with
  `extrinsic_est_en: false`.
- Initialized Point-LIO EKF gyro bias from the stationary IMU mean collected
  during startup.

### Fly-Away Diagnosis

- Added optional `Log/lio_update_debug.csv` output with LiDAR update success,
  effective point counts, residuals, nearest-neighbor distances, IMU norms,
  before/after attitude, position, and velocity.
- Added live IMU/LIO debug tooling for comparing `/imu/data_raw`,
  `/imu/data_lio`, and `/aft_mapped_to_init` while reproducing runaway motion.
- Kept the remaining suspected root cause visible: near 90 degrees, LiDAR update
  degeneracy or failed updates may allow output-state gravity compensation error
  to integrate into velocity and position.

### Launch And IMU Robustness

- Reused the shared RoboSense bringup launch from the Point-LIO launch path and
  added the body-to-base static transform bridge used for RViz.
- Improved Phidget IMU reconnect diagnostics and optional USB reset recovery.

### Validation Status

- The current workspace build succeeded with:

  ```bash
  colcon build --symlink-install --packages-select point_lio track_robot_perception
  ```

- The known remaining issue is still open: with the LiDAR+IMU assembly tilted
  close to 90 degrees, Point-LIO can still fly away. The next step is to inspect
  `Log/lio_update_debug.csv` from a failing run and decide whether prediction,
  LiDAR correction, or failed-update propagation is the first divergence.

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
