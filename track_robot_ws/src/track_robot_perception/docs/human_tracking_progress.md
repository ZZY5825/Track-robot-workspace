# Human Tracking Progress

Last updated: 2026-07-09

> Historical status: the LiDAR association sections below describe the earlier
> Python implementation. The active C++ fusion architecture, exact algorithms,
> parameters, and current limitations are recorded in
> [`human_tracking_fusion_refactor_log_2026-07-09.md`](human_tracking_fusion_refactor_log_2026-07-09.md).

## Current Goal

Build a gesture-triggered single-person tracking pipeline for the Bunker Pro
robot. The camera selects identity. LiDAR estimates 3D target position and
continues the target when the person leaves the camera field of view. Robot base
control is not active in the current validation flow.

## Working Pieces

### Camera Person Tracking

Implemented in `human_image_tracker_node`.

- Subscribes to the ZED2i left RGB image.
- Runs YOLOv8 pose tracking with ByteTrack by default.
- Tracks only people.
- Publishes detections with bbox, track ID, confidence, and pose keypoints.
- Publishes annotated image output for visual checking.

Main topics:

```text
/zed/zed_node/left/image_rect_color
/human_tracking/detections
/human_tracking/annotated_image
```

### Gesture Trigger and Camera Target Lock

Implemented in `gesture_trigger_node` and `camera_target_lock_node`.

- Start tracking gesture: two hands above jaw line with crossing/waving motion.
- Stop tracking gesture: one hand above jaw line held long enough.
- The target lock behaves as a finite state machine:
  - no locked target
  - first valid waving person becomes the target
  - only that target can stop tracking
  - after stop, the system can accept a new start gesture
- The camera target lock keeps a logical target ID separate from the YOLO track
  ID, so short camera tracker ID changes can be reacquired by bbox prediction.

Main topics:

```text
/human_tracking/gesture_state
/human_tracking/gesture_overlay
/human_tracking/camera_target
/human_tracking/target_overlay
```

### Camera-Initialized LiDAR Target Tracking

The active path is now the C++ tracklet architecture:

- `lidar_tracklet_manager_node`
  - builds generic LiDAR candidate clusters and persistent tracklets
  - does not decide human identity
- `selected_human_target_tracker_node`
  - binds the camera-locked target to one LiDAR tracklet
  - uses camera-guided LiDAR anchor points for 3D initialization
  - maintains selected target state with a constant-velocity Kalman filter
  - continues the selected LiDAR tracklet after the camera loses sight

The older Python `lidar_human_cluster_node`, `target_fusion_node`, and Python
camera-LiDAR association node were removed from the human-tracking source path
after the C++ tracklet pipeline became stable on rosbag `145900`.

Main topics:

```text
/rslidar_points
/zed/zed_node/left/camera_info
/human_tracking/lidar_candidate_clusters
/human_tracking/lidar_tracklets
/human_tracking/selected_lidar_tracklet
/human_tracking/camera_guided_target_points
/human_tracking/fused_target_state
/human_tracking/target_state
/human_tracking/fused_target_marker
/human_tracking/selected_tracklet_marker
/human_tracking/selected_target_marker
/human_tracking/target_prediction_gate_marker
/human_tracking/target_tracker_debug
```

## Important LiDAR Fixes Already Made

- Fixed RoboSense QoS mismatch by using reliable QoS for `/rslidar_points`.
- Removed a conflicting static TF publisher for `rslidar`.
- Added camera-LiDAR extrinsic fallback for projection:

```text
zed_camera_link -> rslidar
x=-0.27, y=0.0, z=0.08
yaw=0.0, pitch=0.0, roll=0.0
```

- Added robot-frame fallback matching current bringup:

```text
base_link -> rslidar
x=0.0, y=0.0, z=0.70
yaw=0.0, pitch=0.0, roll=0.0
```

- Fixed fallback transform direction for static extrinsics.
- Switched visualization/output frame back to `base_link` because
  `track_robot_center` was not present in the live TF tree.
- Added debug fields showing camera lock, projection counts, projected UV
  range, selected LiDAR-only score, and reject reason.
- Legacy cluster topics now publish the selected target cluster after
  association instead of always publishing empty arrays.

## LiDAR-Only Continuation Status

LiDAR-only continuation is implemented in the C++ selected-target tracker.

When the camera loses the target, the tracker:

- keeps the logical camera target ID alive until stop gesture or reset
- continues only the previously selected LiDAR tracklet ID
- uses a linear constant-velocity Kalman filter with state
  `[x, y, z, vx, vy, vz]`
- rejects large inconsistent jumps using NIS gating
- allows strict local relinking only after consecutive compatible observations
- decays to prediction-only and then lost instead of globally picking a new
  cluster

Relevant parameters are in:

```text
src/track_robot/track_robot_lidar_tracking/config/selected_target_tracker.yaml
```

Expected state flow:

```text
CAMERA_LIDAR_TRACKED -> LIDAR_ONLY_TRACKING -> PREDICTION_ONLY -> TARGET_LOST
```

## Current Validation Status

Confirmed by user testing:

- Camera tracking works.
- Gesture-triggered target lock works.
- Camera-LiDAR target points and RViz markers now appear while the target is
  visible to the camera and LiDAR.
- The next active validation item is whether LiDAR-only continuation holds the
  same target cluster after the person exits the camera field of view.

If LiDAR-only continuation fails, inspect:

```bash
ros2 topic echo /human_tracking/target_tracker_debug --full-length
```

Key fields:

```text
state
selected_lidar_tracklet_id
selected_tracklet_present
selected_tracklet_score
switch_failure_count
switch_reject_reason
target_clear_reason
camera_guided_status
camera_guided_points
kalman_nis_xy
```

## Run Command

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch track_robot_perception human_tracking_validation.launch.py
```

## RViz

Use a TF-visible fixed frame:

```text
Fixed Frame: base_link
```

Displays:

```text
PointCloud2: /rslidar_points
PointCloud2: /human_tracking/camera_guided_target_points
MarkerArray: /human_tracking/fused_target_marker
MarkerArray: /human_tracking/selected_tracklet_marker
MarkerArray: /human_tracking/selected_target_marker
MarkerArray: /human_tracking/target_prediction_gate_marker
```

## Not Done Yet

- No robot base following control is enabled.
- No cmd_vel is published by this validation launch.
- No NAV2 integration is part of the current phase.
- LiDAR-only continuation needs more walking-out-of-camera-FOV validation and
  parameter tuning.
- Static background subtraction is available as a config option but not tuned
  for the lab yet.

## Early-Phase Reinforcement Update

The active camera/LiDAR path now includes the following completed changes:

- `CameraTarget` separates logical human identity from the current ByteTrack ID.
- Camera reacquisition uses torso HSV appearance, normalized torso pose, bbox
  motion, and size with a top-two ambiguity margin.
- The exact selected pose keypoints are carried into LiDAR fusion.
- Generic LiDAR tracklets and selected-target prediction use sensor timestamps.
- Live tracking uses `track_robot_center`; fused control-facing output is
  transformed to `base_link` at the measurement timestamp.
- Stationary bag replay uses `human_tracking_rosbag_replay.launch.py` with
  `base_link` tracking.
- Camera and cloud inputs use bounded nearest-timestamp queues with an 80 ms
  correction limit and 200 ms queue-age limit.
- Camera-guided geometry uses torso depth modes and a local 3D component rather
  than a fixed depth percentile over the complete bbox.
- Association retains three hypotheses and rejects low-margin initial binding,
  visible switching, and LiDAR-only relinking.
- Horizontal target motion uses a three-model IMM. Height uses a separate
  robust scalar filter.
- PointCloud2 parse, transform, crop, and voxelization use one traversal.
- Rosbag timestamp rollback clears gesture, camera identity, generic tracklet,
  and selected-target temporal state before the next loop.
- Regression tools report rates, synchronization, identity changes, physical
  tracklet switches, ambiguity, position jumps, and deterministic sensor hashes.

Still requiring robot-side execution:

- Replay all four bags at normal rate and inspect the regression reports.
- Replay one representative bag at 0.5x, 1x, and 2x and compare the reports.
- Confirm the optimized LiDAR manager sustains at least 15 Hz on the Jetson.
- Record and validate the planned 2/4/6/8/10 m long-range sequence.
