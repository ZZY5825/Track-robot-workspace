# Human Tracking Fusion Refactor Log

Date: 2026-07-09
Workspace: `~/track_robot_ws`
ROS distribution: ROS2 Foxy
Platform: Jetson AGX
Tracking frame: `base_link`

## 1. Purpose

This log records the LiDAR and camera fusion work completed during the current
human-tracking development cycle.

The main design rule is:

- The camera and gesture FSM own target identity.
- LiDAR provides the target's 3D position and short-term motion information.
- LiDAR does not independently decide which object is the selected human.
- The system should report a lost target instead of switching to a wall, desk,
  or unrelated cluster.

The initial implementation milestone was limited to reliable target binding
while the selected person was visible to both the camera and LiDAR. The second
implementation pass added measured LiDAR-only continuation, strict local
tracklet relinking, bounded prediction, and event-driven cloud projection.

### Test update before the second implementation pass

Rosbag testing showed:

- Generic LiDAR tracklet markers had good RViz FPS.
- Generic initialization remained weak when the person was close to both the
  ground plane and a desk.
- The selected LiDAR tracklet marker accurately covered the person through
  approximately half of the recording.
- The exact selected tracklet disappeared when the person stopped and turned.
- The selected-target prediction flew away in a straight line when the person
  followed a circular path outside camera view.
- Camera-visible fusion ran noticeably slower than LiDAR-only visualization.
- Marker rate recovered when the target entered the LiDAR-only region.

Code inspection identified three direct causes:

- The camera-invisible branch predicted but never corrected from LiDAR.
- The generic alpha-beta tracklet could overshoot a stop or turn and fail its
  fixed association gate.
- Full-cloud camera projection ran from tracklet callbacks and the timer,
  processing the same cloud more than once.

## 2. Reason for the Refactor

The previous camera-LiDAR association node was implemented in Python. Tests
showed these problems:

- Selected-target markers were often empty or updated at a very low rate.
- Camera-guided target points were empty when TF was missing or stale.
- The selected cluster could jump from the person to static lab objects.
- Large merged clusters, such as a person touching a desk, were rejected before
  camera evidence could be used.
- Scan-to-scan motion filtering was noisy and did not provide a persistent
  target identity.
- Full point-cloud processing in Python increased processing time.
- Several overlapping fallback rules made it difficult to identify which
  measurement was controlling the target state.

The replacement separates the problem into two C++ nodes:

1. `lidar_tracklet_manager_node`
   - Produces generic, persistent LiDAR tracklets.
   - Does not assign human identity.
2. `selected_human_target_tracker_node`
   - Receives the camera-locked target.
   - Associates that target with LiDAR geometry.
   - Maintains one target state with a linear Kalman filter.

The old Python association node remains in the repository for reference, but
the main simplified and validation launch files no longer start it.

## 3. Files Added or Modified

### New C++ target tracker

`src/track_robot/track_robot_lidar_tracking/src/selected_human_target_tracker_node.cpp`

This node replaces the old Python association node in the active launch path.
It implements:

- Camera-to-LiDAR tracklet projection and scoring.
- Multi-frame tracklet-selection confirmation.
- Skeleton-guided or bbox-guided LiDAR point extraction.
- Robust target-anchor estimation.
- A six-state constant-velocity Kalman filter.
- NIS-based measurement rejection.
- Selected-target, selected-tracklet, prediction-gate, and fused markers.
- Structured JSON debug output.

### New target-tracker configuration

`src/track_robot/track_robot_lidar_tracking/config/selected_target_tracker.yaml`

This file contains camera association, camera-guided extraction, Kalman filter,
TF fallback, output-rate, and range parameters.

### LiDAR tracklet manager

`src/track_robot/track_robot_lidar_tracking/src/lidar_tracklet_manager_node.cpp`

The manager was updated to provide:

- Weak/raw candidate clusters and filtered persistent tracklets as separate
  outputs.
- Range-adaptive minimum point and minimum height thresholds.
- Explicit flags for oversized, sparse, and excessively large clusters.
- Suppression of physically impossible markers unless explicitly enabled.
- Persistent generic tracklet IDs using a four-state XY Kalman filter.
- Global minimum-cost cluster assignment.
- Stop detection and maneuver-adaptive process noise.

Configuration:

`src/track_robot/track_robot_lidar_tracking/config/lidar_tracklets.yaml`

### Build system

`src/track_robot/track_robot_lidar_tracking/CMakeLists.txt`

Changes:

- Added `Eigen3`.
- Added the `selected_human_target_tracker_node` executable.
- Installed both C++ executables and their configuration files.

### Launch files

The active fusion launch now starts the C++ target tracker:

`src/track_robot_perception/launch/camera_lidar_tracklet_tracking.launch.py`

The following launch files now use
`selected_target_tracker.yaml` by default:

- `src/track_robot_perception/launch/human_tracking_simplified.launch.py`
- `src/track_robot_perception/launch/human_tracking_validation.launch.py`

### Diagnostic tool

`src/track_robot_perception/track_robot_perception/human_tracking_pipeline_diagnostic.py`

The diagnostic now reads:

- `/human_tracking/target_tracker_debug`
- `/human_tracking/selected_tracklet_marker`
- `/human_tracking/selected_target_marker`
- `/human_tracking/target_prediction_gate_marker`
- `/human_tracking/fused_target_marker`
- `/human_tracking/camera_guided_target_points`

The rosbag test guide was also updated:

`rosbags/human_tracking_rosbag_test_guide.md`

## 4. Current Data Flow

```text
ZED image
  -> human_image_tracker_node
  -> /human_tracking/detections

/human_tracking/detections
  -> gesture_trigger_node
  -> camera_target_lock_node
  -> /human_tracking/camera_target

/rslidar_points
  -> lidar_tracklet_manager_node
  -> /human_tracking/lidar_candidate_clusters
  -> /human_tracking/lidar_tracklets

/human_tracking/camera_target
/human_tracking/detections
/human_tracking/lidar_tracklets
/rslidar_points
/zed/zed_node/left/camera_info
  -> selected_human_target_tracker_node
  -> /human_tracking/selected_lidar_tracklet
  -> /human_tracking/fused_target_state
  -> visualization and debug topics
```

## 5. Generic LiDAR Tracklet Algorithm

### 5.1 Point-cloud preprocessing

The tracklet manager transforms the cloud into `base_link`, then applies:

1. Range crop: `0.5 m <= range <= 10.0 m`.
2. Height crop: `-0.25 m <= z <= 2.2 m`.
3. Ground removal: remove points below `z = -0.15 m`.
4. Voxel downsampling with a `0.10 m` leaf size.
5. Point cap of `12000` before clustering.
6. Process every second input cloud.

Cropping and downsampling happen before clustering to control CPU load.

### 5.2 Candidate clustering

The manager uses a DBSCAN-like region-growing algorithm with a 2D spatial hash.
The XY clustering tolerance is `0.38 m`, and a core point needs at least three
neighbors.

The spatial hash avoids a full all-to-all point-distance calculation. Neighbor
search is limited to nearby grid cells.

Each cluster produces:

- Centroid.
- Minimum and maximum XYZ bounds.
- XYZ dimensions.
- Point count.
- Range and bearing.
- Eligibility flags.

### 5.3 Range-adaptive evidence limits

LiDAR returns become sparse with distance and may only cover part of the body at
close range. The manager therefore uses these minimum point counts:

| Range | Minimum points |
|---|---:|
| Near: below `2.0 m` | 3 |
| Mid: `2.0-6.0 m` | 6 |
| Far: above `6.0 m` | 2 |

Minimum observed cluster height is also range-dependent:

| Range | Minimum observed height |
|---|---:|
| Near | `0.10 m` |
| Mid | `0.25 m` |
| Far | `0.05 m` |

These are observed LiDAR extents, not full anatomical human dimensions. A close
person may only produce waist, chest, or arm returns. A far person may only
produce a few scan-line returns.

### 5.4 Candidate and tracklet separation

All reasonable raw clusters are published on:

```text
/human_tracking/lidar_candidate_clusters
```

Candidate messages include:

- `oversized`
- `too_sparse`
- `too_large`
- `near_static`
- `dynamic_score`
- `point_count`
- centroid and bounding dimensions

Only eligible clusters can create or update generic tracklets. A cluster is not
eligible when it is sparse, too large by point count, oversized, below its
range-adaptive height, or below the generic point threshold.

Current normal tracklet size limits are:

- Height: `0.15-2.2 m`
- Width: `0.05-1.2 m`
- Depth: at most `1.2 m`
- Point count: at most `2500`

An impossible raw cluster is defined by any of:

- Width greater than `3.0 m`
- Depth greater than `3.0 m`
- Height greater than `2.8 m`

Impossible candidate markers are hidden by default. The raw candidate data
model still records the rejection reason where the cluster is retained.

### 5.5 Generic tracklet association

Tracklets use a four-state linear constant-velocity Kalman filter:

```text
state = [x, y, vx, vy]
measurement = [cluster_centroid_x, cluster_centroid_y]
```

The normal white-acceleration process-noise standard deviation is `1.5 m/s^2`.
It increases to `3.0 m/s^2` when measured motion conflicts with predicted
velocity, allowing faster correction during a turn.

Data association uses a global Hungarian assignment. Pair cost is:

```text
cost =
    0.65 * normalized_NIS
  + 0.20 * normalized_bbox_size_change
  + 0.15 * normalized_point_count_change
```

Pairs are invalid when:

- XY distance exceeds `1.2 m`.
- XY NIS exceeds `9.21`.

Stationary handling is activated after three consecutive measurements move less
than `0.10 m` and measured speed is below `0.25 m/s`. Using both conditions
prevents the stop detector from depending only on scan rate. Velocity is
multiplied by `0.5` for each confirmed stationary update and set to zero below
`0.08 m/s`. The maximum speed remains `2.2 m/s`.

Tracklet lifecycle:

- Confirm after 3 hits.
- Increase confidence by `0.15` for each hit.
- Decrease confidence by `0.06` for each miss.
- Delete after 12 missed processed frames, 1.5 seconds without valid
  support, or confidence reaching zero.
- Publish confirmed tracklets with confidence of at least `0.10`.
- Continue publishing retained predicted tracklets with `active=false` and
  faded gray markers.

This four-state XY filter is separate from the selected target's six-state XYZ
filter.

## 6. Camera-to-LiDAR Tracklet Association

### 6.1 Camera target validity

Camera evidence is used only when:

- `target_id >= 0`
- `lock_state == LOCK_TARGET_LOCKED`
- The target message is no older than `1.0 s`
- `camera_visible == true`
- Camera confidence is at least `0.35`

The camera target ID remains the semantic target ID. A LiDAR tracklet ID is a
separate geometric ID and is published with the target state.

### 6.2 Tracklet projection

For each published LiDAR tracklet, the node projects:

- The tracklet centroid.
- All eight corners of its 3D bounding box.

Projection uses the ZED `CameraInfo` matrix. It uses `P` when available and
falls back to `K`.

The node first requests TF into
`zed_left_camera_optical_frame`. If the recorded bag does not contain a complete
camera-to-LiDAR TF chain, it uses the configured direct optical fallback:

```text
translation = (0.06, -0.065, -0.26) m
quaternion  = (0.5, -0.5, 0.5, 0.5)
```

This fallback must be recalibrated if the physical sensor installation changes.

### 6.3 Association score

Each projected tracklet receives this score:

```text
score =
    0.30 * projected_bbox_IoU
  + 0.20 * centroid_inside_camera_bbox
  + 0.20 * Kalman_NIS_score
  + 0.15 * previous_selected_tracklet_bonus
  + 0.10 * tracklet_confidence
  + 0.05 * soft_human_size_score
```

The human-size term is intentionally a soft score. Camera-visible association
does not hard-reject a measurement only because its LiDAR bounding box is not
human-sized.

The soft size references are:

- Width near `0.55 m`
- Depth near `0.35 m`
- Observed height near `1.2 m`

The minimum accepted association score is `0.55`.

A new LiDAR tracklet ID must win for two consecutive association updates before
it becomes `selected_lidar_tracklet_id`. This reduces one-frame ID switching.

## 7. Camera-Guided LiDAR Anchor

The camera-guided anchor is independent of the generic cluster size filter. It
projects raw LiDAR points into a body ROI and can therefore recover useful
target points from a large person-plus-desk cluster.

### 7.1 Body ROI selection

ROI priority:

1. Skeleton torso ROI.
   - Requires confident left/right shoulders and left/right hips.
   - COCO keypoint indices: 5, 6, 11, and 12.
2. Skeleton upper-body ROI.
   - Requires confident left and right shoulders.
3. Central bbox fallback.

Minimum keypoint confidence is `0.35`.

The central bbox fallback keeps:

```text
horizontal: central 50% of bbox width
vertical:   20%-70% of bbox height
```

This avoids bbox edges, feet, much of the ground, and background next to the
person.

### 7.2 Raw-point limits

For each update:

- Read only XYZ and optional intensity from `PointCloud2`.
- Cap projection input to `15000` points using stride sampling.
- Keep points in the `0.5-10.0 m` and `-0.25-2.2 m` crop.
- Require positive optical depth.
- Require at least 3 ROI points.

The debug cloud is capped at 3000 points.

### 7.3 Robust depth and center estimation

The selected ROI points are sorted by optical depth. Only the `20th-55th`
depth-percentile interval is used.

The lower-depth interval is intentional. It suppresses wall or desk returns
behind a visible person.

The 3D anchor is the coordinate-wise median:

```text
anchor = [median(x), median(y), median(z)]
```

A median is less sensitive than a mean to isolated background points.

### 7.4 Anchor quality

The measurement quality is:

```text
quality =
    0.35 * point_count_score
  + 0.25 * depth_spread_score
  + 0.25 * prediction_distance_score
  + 0.15 * camera_confidence
```

Terms:

- Point count reaches full score at five times the minimum point count.
- Depth spread is measured as depth percentile 90 minus percentile 10.
- Maximum expected depth spread is `1.25 m`.
- Prediction consistency reaches zero at `1.5 m` from the predicted target.
- Minimum accepted quality is `0.25`.

Measurement covariance is scaled inversely with quality. Lower-quality anchors
therefore produce weaker Kalman corrections.

## 8. Selected-Target Kalman Filter

### 8.1 Filter type

The selected target uses a linear constant-velocity Kalman filter, not an EKF.
The measurement model is linear, so an EKF would add complexity without adding
information.

State:

```text
x = [px, py, pz, vx, vy, vz]^T
```

Measurement:

```text
z = [px, py, pz]^T
```

The filter runs in `base_link`.

### 8.2 Prediction model

The transition matrix is:

```text
px' = px + vx * dt
py' = py + vy * dt
pz' = pz + vz * dt
v'  = v
```

`dt` is limited to `0.5 s` to prevent a large prediction jump after a delayed
callback.

The process model assumes white acceleration noise with standard deviation:

```text
process_noise_accel_std = 1.2 m/s^2
```

For each axis, process covariance uses:

```text
Qpp = 0.25 * dt^4 * accel_variance
Qpv = 0.50 * dt^3 * accel_variance
Qvv =        dt^2 * accel_variance
```

Initial covariance:

```text
position variance = 0.35 m^2
velocity variance = 1.00 (m/s)^2
```

### 8.3 Measurement covariance

Default measurement variances:

```text
camera-guided anchor XY variance = 0.08 m^2 / quality
selected tracklet XY variance    = 0.18 m^2
Z variance                       = 0.35 m^2
```

Camera-guided anchors are trusted more than generic tracklet centroids when
their quality is high.

### 8.4 Innovation rejection

Measurements are gated using the XY normalized innovation squared:

```text
NIS = residual_xy^T * S_xy^-1 * residual_xy
```

Current limits:

```text
camera-guided anchor NIS limit = 25.0
tracklet centroid NIS limit    = 9.21
```

`9.21` is the 99% chi-square threshold for two dimensions. The camera-guided
anchor uses a looser gate because initial calibration and partial-body geometry
can introduce larger errors.

Rejected measurements do not reset the filter. The target remains in prediction
mode with decaying confidence.

The covariance update uses Joseph form:

```text
P = (I-KH) P (I-KH)^T + K R K^T
```

This form is more numerically stable than the simplified covariance update.

## 9. State and Measurement Priority

While the camera target is visible:

1. Find the best projected LiDAR tracklet.
2. Update the pending selected-tracklet ID.
3. Compute a camera-guided raw-point anchor.
4. Correct the Kalman filter with the anchor when quality and NIS pass.
5. If the anchor fails, try the already confirmed selected tracklet centroid.
6. If both fail, retain prediction rather than select unrelated geometry.

Current output mapping:

| Internal state | `TargetState.track_state` | Source |
|---|---|---|
| `camera_lidar` | `TRACK_CAMERA_LIDAR_TRACKED` | `SOURCE_CAMERA_LIDAR` |
| `lidar_only` | `TRACK_LIDAR_ONLY_TRACKING` | `SOURCE_LIDAR_ONLY` |
| `camera_only` | `TRACK_CAMERA_LOCKED` | `SOURCE_CAMERA_ONLY` |
| `prediction_only` | `TRACK_PREDICTION_ONLY` | `SOURCE_PREDICTION_ONLY` |
| `target_lost` | `TRACK_TARGET_LOST` | `SOURCE_NONE` |
| `none` | `TRACK_NO_TARGET` | `SOURCE_NONE` |

Outside camera view, the exact selected LiDAR ID is corrected first. If it
disappears, replacement is allowed only inside the Kalman gate after three
consecutive compatible matches. Prediction velocity is damped after `0.4 s`.
The system reports `TARGET_LOST` after `2.0 s` without accepted LiDAR evidence.

## 10. Topics

### Inputs

| Topic | Type | Purpose |
|---|---|---|
| `/rslidar_points` | `sensor_msgs/msg/PointCloud2` | Raw LiDAR scan |
| `/zed/zed_node/left/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera projection |
| `/human_tracking/detections` | `HumanDetection2DArray` | Bboxes and pose keypoints |
| `/human_tracking/camera_target` | `TargetState` | Gesture-selected camera identity |
| `/human_tracking/lidar_candidate_clusters` | `LidarClusterArray` | Raw/weak cluster metadata |
| `/human_tracking/lidar_tracklets` | `LidarTrackletArray` | Persistent generic geometry |

### Outputs

| Topic | Type | Purpose |
|---|---|---|
| `/human_tracking/selected_lidar_tracklet` | `SelectedLidarTracklet` | Camera target to LiDAR ID binding |
| `/human_tracking/fused_target_state` | `TargetState` | Main fused target state |
| `/human_tracking/target_state` | `TargetState` | Compatibility output |
| `/human_tracking/camera_guided_target_points` | `PointCloud2` | ROI and depth-filtered target points |
| `/human_tracking/selected_tracklet_marker` | `MarkerArray` | Exact confirmed LiDAR tracklet bbox |
| `/human_tracking/selected_target_marker` | `MarkerArray` | Kalman target volume and state text |
| `/human_tracking/target_prediction_gate_marker` | `MarkerArray` | Covariance-based XY gate |
| `/human_tracking/fused_target_marker` | `MarkerArray` | Fused center and state |
| `/human_tracking/target_tracker_debug` | `std_msgs/msg/String` | JSON diagnostics |

Marker distinction:

- `selected_tracklet_marker` exists only after a LiDAR tracklet ID is confirmed
  and that ID is still present in `/human_tracking/lidar_tracklets`.
- `selected_target_marker` represents the Kalman state. It can remain visible
  during prediction-only operation.
- `target_prediction_gate_marker` is centered on the Kalman prediction. Its
  radius is the larger of `1.2 m` and two standard deviations from XY
  covariance.

## 11. Parameter Summary

### Tracklet manager

| Parameter | Current value | Function |
|---|---:|---|
| `min_range` | `0.5` | Ignore robot/self and unsafe near returns |
| `max_range` | `10.0` | Maximum candidate range |
| `min_z`, `max_z` | `-0.25`, `2.2` | Vertical crop |
| `ground_z_threshold` | `-0.15` | Simple ground rejection |
| `voxel_leaf_size` | `0.10` | Early downsampling |
| `process_every_n_clouds` | `2` | CPU control |
| `cluster_tolerance` | `0.38` | XY neighborhood radius |
| `cluster_core_min_points` | `3` | Region-growing core size |
| `near/mid/far_min_cluster_points` | `3/6/2` | Range-adaptive density |
| `max_cluster_points` | `2500` | Large-cluster flag |
| `max_bbox_width/depth` | `1.2/1.2` | Generic-tracklet size gate |
| `tracklet_gating_distance` | `1.2` | Absolute detection-to-tracklet XY gate |
| `tracklet_confirm_hits` | `3` | Persistent-ID confirmation |
| `max_tracklet_missed_frames` | `12` | Tracklet deletion |
| `max_tracklet_speed_mps` | `2.2` | Velocity limit |
| `tracklet_process_noise_accel_std` | `1.5` | Normal CV process noise |
| `tracklet_maneuver_noise_accel_std` | `3.0` | Stop/turn process noise |
| `tracklet_measurement_variance` | `0.12` | Centroid measurement variance |
| `tracklet_nis_gate` | `9.21` | Statistical association gate |

### Selected target tracker

| Parameter | Current value | Function |
|---|---:|---|
| `camera_visible_min_confidence` | `0.35` | Enable camera-guided mode |
| `min_association_score` | `0.55` | Accept projected tracklet |
| `association_confirm_frames` | `2` | Confirm LiDAR ID |
| `max_projection_center_error_px` | `220` | Normalize 2D center score |
| `camera_target_timeout_sec` | `1.0` | Camera target freshness |
| `input_timeout_sec` | `1.0` | Point-cloud freshness |
| `max_cloud_points_for_projection` | `15000` | Projection CPU cap |
| `camera_guided_min_points` | `3` | Minimum 3D anchor evidence |
| `camera_guided_depth_percentile_low/high` | `20/55` | Foreground depth interval |
| `camera_guided_max_depth_spread` | `1.25` | Anchor-quality scale |
| `camera_guided_prediction_gate_m` | `1.5` | Anchor-quality motion scale |
| `process_noise_accel_std` | `1.2` | Kalman motion uncertainty |
| `camera_anchor_xy_variance` | `0.08` | High-quality anchor variance |
| `tracklet_xy_variance` | `0.18` | Generic centroid variance |
| `max_camera_anchor_nis_xy` | `25.0` | Camera anchor innovation gate |
| `max_tracklet_nis_xy` | `9.21` | Tracklet innovation gate |
| `prediction_gate_radius_m` | `1.2` | Minimum RViz gate radius |
| `max_prediction_gate_radius_m` | `2.0` | Maximum RViz gate radius |
| `selected_relink_confirm_frames` | `3` | Strict replacement-ID confirmation |
| `selected_relink_timeout_sec` | `1.5` | Local replacement search window |
| `prediction_only_timeout_sec` | `2.0` | Lost-state timeout |
| `prediction_velocity_damping_rate` | `0.8` | Prediction velocity decay |
| `camera_projection_max_rate_hz` | `10.0` | Fresh-cloud projection limit |
| `publish_rate` | `10 Hz` | State and marker timer |
| `debug_rate` | `2 Hz` | JSON debug rate |

## 12. Build and Verification Completed

The following build completed successfully:

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build \
  --packages-select track_robot_lidar_tracking track_robot_perception \
  --symlink-install
```

The installed package exposes:

```text
lidar_tracklet_manager_node
selected_human_target_tracker_node
```

The simplified launch resolves the new
`selected_target_tracker.yaml` configuration. A short ROS runtime smoke test
also confirmed that the C++ target tracker starts and loads its parameters.

After the stop/turn and LiDAR-only refactor, both packages rebuilt successfully
and both C++ nodes passed standalone startup smoke tests with their active YAML
files. An automated replay-rate check of bag `145900` was attempted, but Fast
DDS reported deserialization failures in the isolated test domain. No behavioral
or FPS claim is made from that failed replay; the four-bag validation procedure
below remains required on the normal robot ROS domain.

## 13. Test Commands

Start the current pipeline:

```bash
cd ~/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch track_robot_perception human_tracking_simplified.launch.py
```

Run the diagnostic during the part of the bag where the gesture-selected target
is visible:

```bash
ros2 run track_robot_perception human_tracking_pipeline_diagnostic --duration 20
```

Check rates:

```bash
ros2 topic hz /human_tracking/lidar_tracklets
ros2 topic hz /human_tracking/fused_target_state
ros2 topic hz /human_tracking/selected_target_marker
```

Inspect target decisions:

```bash
ros2 topic echo /human_tracking/target_tracker_debug
ros2 topic echo /human_tracking/selected_lidar_tracklet
ros2 topic echo /human_tracking/fused_target_state
```

RViz fixed frame:

```text
base_link
```

Recommended RViz displays:

```text
MarkerArray  /human_tracking/lidar_candidate_cluster_markers
MarkerArray  /human_tracking/lidar_tracklet_markers
MarkerArray  /human_tracking/selected_tracklet_marker
MarkerArray  /human_tracking/selected_target_marker
MarkerArray  /human_tracking/target_prediction_gate_marker
MarkerArray  /human_tracking/fused_target_marker
PointCloud2  /human_tracking/camera_guided_target_points
```

## 14. Known Limitations

These items still require rosbag or live-system validation:

1. Camera-exit continuation
   - Exact-ID LiDAR correction and `LIDAR_ONLY_TRACKING` are implemented.
   - Circular walking, stop/turn behavior, and strict relinking still need
     validation against all four bags.

2. Selected tracklet recovery
   - Three-frame local relinking is implemented.
   - Its NIS, size, point-count, and timeout thresholds require measured tuning.
   - Global replacement outside camera view remains prohibited.

3. Merged person and obstacle
   - Camera-guided raw-point extraction handles visible merged geometry.
   - Splitting a merged cluster during LiDAR-only operation is not implemented.

4. Candidate-cluster use
   - The selected target node subscribes to candidate clusters for diagnostics
     and future camera-guided cluster recovery.
   - Current selected-target correction uses raw projected points and confirmed
     tracklets. Candidate clusters do not directly update the filter.

5. Timestamp synchronization
   - Inputs are cached as latest messages.
   - Exact or approximate message-time synchronization is not implemented.
   - TF lookup uses the latest available transform rather than each cloud's
     original timestamp.

6. Moving robot
   - The Kalman filter currently tracks in `base_link`.
   - This is acceptable for the stationary test milestone.
   - Robot motion will appear as target motion. Before closed-loop following,
     prediction should run in a stable `odom` or `map` frame and then transform
     the output back to `base_link`.

7. Processing schedule
   - Full-cloud projection now runs only from a fresh cloud callback and is
     capped at 10 Hz.
   - Tracklet callbacks perform only association and filter correction.
   - The timer performs only prediction and publication.
   - Jetson testing must confirm the 10 Hz acceptance target.

8. Detection freshness
   - Pose detections are cached, but skeleton ROI selection does not currently
     reject stale detection arrays independently.

9. Calibration
   - The direct optical transform is a fallback, not a substitute for a proper
     calibrated and timestamp-valid TF chain.

## 15. Next Engineering Milestone

The next milestone is controlled validation and tuning:

1. Replay all four bags and capture target-tracker JSON diagnostics.
2. Verify at least 10 Hz during camera-visible projection.
3. Measure generic ID survival through the stop-and-turn section.
4. Verify `CAMERA_LIDAR_TRACKED -> LIDAR_ONLY_TRACKING`.
5. Verify strict relinking requires three compatible observations.
6. Verify prediction ends in `TARGET_LOST` after two seconds.
7. Tune process noise and gates from measured NIS rather than visual guesses.
8. Move tracking to `odom` before enabling a moving robot.

The safety rule remains unchanged: losing the target is preferable to silently
switching identity.
