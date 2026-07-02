# Human Tracking Progress

Last updated: 2026-07-02

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

Implemented in `lidar_human_cluster_node`.

The old Phase 4 global LiDAR-only human classification approach was dropped
because it was unreliable in the lab. The current node uses camera identity
first, then LiDAR geometry:

1. Wait for `/human_tracking/camera_target` with `lock_state == 2`.
2. Project RoboSense LiDAR points into the ZED image.
3. Select LiDAR points inside the locked target bbox.
4. Use foreground depth filtering and local clustering to estimate the target
   3D position.
5. Publish target point cloud, fused state, search gate marker, and fused marker.
6. When camera visibility is lost, continue tracking locally around the
   predicted target position using LiDAR-only candidate association.

Main topics:

```text
/rslidar_points
/zed/zed_node/left/camera_info
/human_tracking/target_lidar_points
/human_tracking/fused_target_state
/human_tracking/target_state
/human_tracking/fused_target_marker
/human_tracking/target_search_gate_marker
/human_tracking/lidar_target_debug
/human_tracking/lidar_clusters
/human_tracking/lidar_human_candidates
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

LiDAR-only continuation is implemented as a first usable version.

When the camera loses the target, the tracker:

- predicts target position with a constant-velocity alpha-beta filter
- searches LiDAR points near the predicted position
- gates candidates by distance from last LiDAR measurement
- limits max jump and max target speed
- uses last target z range and rough size similarity
- applies softer LiDAR-only filter gains than camera-LiDAR updates
- keeps the last target cloud visible during short prediction-only gaps

Relevant parameters in `config/lidar_human_candidates.yaml`:

```text
lidar_only_alpha_position: 0.35
lidar_only_beta_velocity: 0.08
max_target_speed_mps: 2.0
lidar_only_min_score: 0.32
lidar_only_max_jump_m: 0.85
lidar_only_z_margin: 0.45
lidar_only_keep_last_points: true
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
ros2 topic echo /human_tracking/lidar_target_debug --full-length
```

Key fields:

```text
source
track_state
camera_visible
lidar_only_candidate_count
lidar_only_selected_score
lidar_only_reject_reason
allowed_lidar_only_jump
last_measurement_base
last_target_z_range
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
PointCloud2: /human_tracking/target_lidar_points
MarkerArray: /human_tracking/fused_target_marker
MarkerArray: /human_tracking/target_search_gate_marker
```

## Not Done Yet

- No robot base following control is enabled.
- No cmd_vel is published by this validation launch.
- No NAV2 integration is part of the current phase.
- LiDAR-only continuation needs more walking-out-of-camera-FOV validation and
  parameter tuning.
- Static background subtraction is available as a config option but not tuned
  for the lab yet.
