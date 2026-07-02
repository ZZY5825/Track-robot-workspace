# Phase 4 Camera-Initialized LiDAR Target Tracking

Phase 4 no longer tries to classify humans from LiDAR alone. Camera and gesture
locking provide identity; LiDAR estimates and maintains the locked target's 3D
position.

## Data Flow

```text
/human_tracking/camera_target
/zed/zed_node/left/camera_info
/rslidar_points
TF or configured fallback: rslidar -> zed_left_camera_optical_frame
TF or configured fallback: rslidar -> base_link
TF when available: base_link -> map
  -> lidar_human_cluster_node
  -> /human_tracking/fused_target_state
  -> /human_tracking/target_state
```

When the camera target is visible, LiDAR points are projected into the image and
points inside the locked bbox initialize/correct the 3D track. The selected ROI
uses foreground depth filtering and local clustering. When the camera target is
not visible, the tracker predicts target position and searches only inside a
local range-adaptive gate using the last camera-confirmed LiDAR target cluster
as identity memory.

## Outputs

```text
/human_tracking/target_lidar_points       sensor_msgs/PointCloud2
/human_tracking/target_search_gate_marker visualization_msgs/MarkerArray
/human_tracking/fused_target_state        track_robot_interfaces/TargetState
/human_tracking/target_state              track_robot_interfaces/TargetState
/human_tracking/fused_target_marker       visualization_msgs/MarkerArray
/human_tracking/lidar_target_debug        std_msgs/String JSON
```

The legacy `/human_tracking/lidar_clusters` and
`/human_tracking/lidar_human_candidates` topics are not used as the tracking
source. After target association succeeds, they publish one selected target
cluster for compatibility/debug visualization.

## Current Hardware Assumptions

The live validation setup uses `base_link` as the RViz/output frame. If TF is
missing, the tracker can fall back to these configured static extrinsics:

```text
base_link -> rslidar
x=0.0, y=0.0, z=0.70
yaw=0.0, pitch=0.0, roll=0.0

zed_camera_link -> rslidar
x=-0.27, y=0.0, z=0.08
yaw=0.0, pitch=0.0, roll=0.0
```

## Run

Camera-only lock plus LiDAR target tracking:

```bash
cd ~/track_robot_ws
source install/setup.bash
ros2 launch track_robot_perception human_tracking_validation.launch.py
```

LiDAR target tracker only, assuming `/human_tracking/camera_target` already
exists:

```bash
ros2 launch track_robot_perception human_lidar_tracking.launch.py
```

## RViz

Use:

```text
Fixed Frame: base_link
PointCloud2: /rslidar_points
PointCloud2: /human_tracking/target_lidar_points
MarkerArray: /human_tracking/target_search_gate_marker
MarkerArray: /human_tracking/fused_target_marker
```

For `/human_tracking/target_lidar_points`, use the `RGB8` or `RGB` color
transformer.

## Debug

```bash
ros2 topic echo /human_tracking/lidar_target_debug
ros2 topic echo /human_tracking/fused_target_state
ros2 topic hz /human_tracking/target_lidar_points
```

Use `--full-length` when inspecting the JSON debug message:

```bash
ros2 topic echo /human_tracking/lidar_target_debug --full-length
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

Important debug fields for camera-to-LiDAR initialization:

```text
camera_init_status
projection_status
projection_in_front_points
projection_roi_points_raw
projection_roi_points_depth_filtered
camera_bbox
projected_uv_range
```

Important debug fields for LiDAR-only continuation:

```text
lidar_only_candidate_count
lidar_only_selected_score
lidar_only_reject_reason
allowed_lidar_only_jump
last_measurement_base
last_target_z_range
```

## Tuning Order

1. Increase `bbox_padding_px` if too few LiDAR points land inside the camera bbox.
2. Lower `min_projected_points` for sparse/far targets.
3. If camera-visible tracking works but LiDAR-only drops the target, inspect
   `lidar_only_reject_reason`.
4. Increase `near_search_radius`, `mid_search_radius`, or `far_search_radius` if
   LiDAR-only recovery has too few local points.
5. Increase `lidar_only_max_jump_m` or `lidar_only_z_margin` if a real target is
   rejected during normal walking.
6. Lower `lidar_only_min_score` only after checking that wrong clutter is not
   being accepted.
7. Tune `confidence_prediction_decay_per_sec` after the position estimate is stable.

## Validation Scenarios

```text
target visible in camera and LiDAR
target very close to robot with partial body returns
target far from robot with sparse returns
target exits camera FOV but remains visible to 360-degree LiDAR
target walks near desks or lab equipment
multiple clutter objects near predicted target path
```
