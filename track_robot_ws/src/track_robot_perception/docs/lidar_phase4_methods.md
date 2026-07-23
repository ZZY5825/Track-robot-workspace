# Phase 4 Camera-Initialized LiDAR Tracklet Tracking

Phase 4 no longer tries to classify humans from LiDAR alone. Camera and gesture
locking provide identity. LiDAR provides 3D geometry, persistent tracklets, and
short camera-out-of-view continuation.

The old Python `lidar_human_cluster_node`, `target_fusion_node`, and Python
camera-LiDAR association node were removed from the active human-tracking
source path. The current implementation is the C++ tracklet pipeline.

## Data Flow

```text
/human_tracking/camera_target
/human_tracking/detections
/zed/zed_node/left/camera_info
/rslidar_points
TF or configured fallback: rslidar -> zed_left_camera_optical_frame
TF or configured fallback: rslidar -> base_link
TF when available: base_link -> map
  -> lidar_tracklet_manager_node
  -> selected_human_target_tracker_node
  -> /human_tracking/fused_target_state
  -> /human_tracking/target_state
```

The tracklet manager publishes generic LiDAR candidates and persistent LiDAR
tracklets. It does not decide which cluster is the human target.

The selected-target tracker binds the gesture-selected camera target to one
LiDAR tracklet using camera-guided 3D anchor points. When the camera target is
not visible, it continues only the selected LiDAR tracklet and uses a
constant-velocity Kalman filter for short prediction gaps.

## Outputs

```text
/human_tracking/lidar_candidate_clusters        track_robot_interfaces/LidarClusterArray
/human_tracking/lidar_tracklets                 track_robot_interfaces/LidarTrackletArray
/human_tracking/selected_lidar_tracklet         track_robot_interfaces/LidarTracklet
/human_tracking/camera_guided_target_points     sensor_msgs/PointCloud2
/human_tracking/fused_target_state              track_robot_interfaces/TargetState
/human_tracking/target_state                    track_robot_interfaces/TargetState
/human_tracking/lidar_tracklet_markers          visualization_msgs/MarkerArray
/human_tracking/lidar_candidate_cluster_markers visualization_msgs/MarkerArray
/human_tracking/selected_tracklet_marker        visualization_msgs/MarkerArray
/human_tracking/selected_target_marker          visualization_msgs/MarkerArray
/human_tracking/target_prediction_gate_marker   visualization_msgs/MarkerArray
/human_tracking/fused_target_marker             visualization_msgs/MarkerArray
/human_tracking/target_tracker_debug            std_msgs/String JSON
```

## Current Hardware Assumptions

The live validation setup uses `base_link` as the RViz/output frame. If TF is
missing, the tracker can fall back to configured static extrinsics.

Current expected local frames:

```text
base_link
rslidar
zed_left_camera_optical_frame
```

## Run

Camera-only lock plus LiDAR tracklet tracking:

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch track_robot_perception human_tracking_validation.launch.py
```

LiDAR tracklet and selected-target tracker only, assuming
`/human_tracking/camera_target` already exists:

```bash
ros2 launch track_robot_perception camera_lidar_tracklet_tracking.launch.py
```

## RViz

Use:

```text
Fixed Frame: base_link
PointCloud2: /rslidar_points
PointCloud2: /human_tracking/camera_guided_target_points
MarkerArray: /human_tracking/lidar_tracklet_markers
MarkerArray: /human_tracking/lidar_candidate_cluster_markers
MarkerArray: /human_tracking/selected_tracklet_marker
MarkerArray: /human_tracking/selected_target_marker
MarkerArray: /human_tracking/target_prediction_gate_marker
MarkerArray: /human_tracking/fused_target_marker
```

For `/human_tracking/camera_guided_target_points`, use the `RGB8` or `RGB`
color transformer.

## Debug

```bash
ros2 topic echo /human_tracking/target_tracker_debug
ros2 topic echo /human_tracking/fused_target_state
ros2 topic echo /human_tracking/selected_lidar_tracklet
ros2 topic hz /human_tracking/camera_guided_target_points
```

Use `--full-length` when inspecting the JSON debug message:

```bash
ros2 topic echo /human_tracking/target_tracker_debug --full-length
```

Important `TargetState.track_state` values:

```text
0 NO_TARGET
1 CAMERA_LOCKED
2 CAMERA_LIDAR_TRACKED
3 LIDAR_ONLY_TRACKING
4 PREDICTION_ONLY
5 TARGET_LOST
```

Important selected-target debug fields:

```text
state
camera_target_id
selected_lidar_tracklet_id
selected_tracklet_present
selected_tracklet_score
challenger_tracklet_id
challenger_score
switch_reject_reason
target_clear_reason
camera_guided_status
camera_guided_points
camera_guided_anchor_quality
kalman_nis_xy
measurement_source
rejection_reason
processing_ms
```

## Tuning Order

1. Validate generic person tracklets first with `/human_tracking/lidar_tracklet_markers`.
2. Validate camera-guided anchor points with `/human_tracking/camera_guided_target_points`.
3. If initial selected ID is wrong, tune anchor gates before projection scores.
4. If selected ID flickers near clutter, tune switch hysteresis and relink gates.
5. If camera-out-of-view tracking drops too soon, inspect selected ID continuity
   before increasing prediction timeout.
6. Prefer `TARGET_LOST` over switching to clutter.

## Validation Scenarios

```text
target visible in camera and LiDAR
target very close to robot with partial body returns
target far from robot with sparse returns
target exits camera FOV but remains visible to 360-degree LiDAR
target walks near desks or lab equipment
multiple clutter objects near predicted target path
target stops and turns near another tracklet
```
