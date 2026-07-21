# Human Tracking Rosbag Test Guide

This guide replays the recorded camera + LiDAR bags and runs the current
tracklet-based human tracking stack offline.

Do not run the live ZED or RoboSense drivers while replaying a bag on the same
`ROS_DOMAIN_ID`.

## Recorded Bags

Current bags in this folder:

```text
~/track_robot_ws/rosbags/human_tracking_lidar_20260706_145752
~/track_robot_ws/rosbags/human_tracking_lidar_20260706_145900
~/track_robot_ws/rosbags/human_tracking_lidar_20260706_150711
~/track_robot_ws/rosbags/human_tracking_lidar_20260706_150918
```

Each bag should contain:

```text
/rslidar_points
/zed/zed_node/left/image_rect_color
/zed/zed_node/left/camera_info
/tf
/tf_static
```

Check a bag:

```bash
ros2 bag info ~/track_robot_ws/rosbags/human_tracking_lidar_20260706_145752
```

## Terminal 1 - Replay One Bag In Loop

Only play one bag at a time.

Bag 1:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 bag play ~/track_robot_ws/rosbags/human_tracking_lidar_20260706_145752 --loop
```

Bag 2:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 bag play ~/track_robot_ws/rosbags/human_tracking_lidar_20260706_145900 --loop
```

Bag 3:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 bag play ~/track_robot_ws/rosbags/human_tracking_lidar_20260706_150711 --loop
```

Bag 4:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 bag play ~/track_robot_ws/rosbags/human_tracking_lidar_20260706_150918 --loop
```

Stop replay with:

```text
Ctrl+C
```

## Terminal 2 - Publish Missing LiDAR Static TF

The bags may not contain the static transform RViz needs between `base_link`
and `rslidar`. Keep this running while replaying bags:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 run tf2_ros static_transform_publisher 0 0 0.70 0 0 0 base_link rslidar
```

Verify if needed:

```bash
ros2 run tf2_ros tf2_echo base_link rslidar
```

Expected translation is roughly:

```text
x: 0.0
y: 0.0
z: 0.70
```

## Terminal 3 - Run Simplified Human Tracking Stack

This is the preferred current test path. It reruns YOLO, gesture detection,
camera target lock, the C++ LiDAR tracklet manager, and the selected-target
camera-LiDAR fusion node on the recorded bag. The fusion node is now the C++
`selected_human_target_tracker_node`, not the old Python association node:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 launch track_robot_perception human_tracking_rosbag_replay.launch.py
```

The replay launch intentionally tracks in `base_link` because these recordings
were made while the robot was stationary. Live operation uses
`human_tracking_simplified.launch.py` and tracks in `track_robot_center`.

Useful checks:

```bash
ros2 topic list | grep human_tracking
ros2 topic echo /human_tracking/camera_target
ros2 param get /lidar_tracklet_manager_node tracking_frame_override
```

The replay tracking-frame override must report `base_link`. If it is empty,
rebuild and source the workspace again before testing.

Important camera topics:

```text
/human_tracking/detections
/human_tracking/gesture_state
/human_tracking/camera_target
/human_tracking/camera_target_debug
/human_tracking/target_overlay
```

Expected topics:

```text
/human_tracking/lidar_tracklets
/human_tracking/lidar_tracklet_markers
/human_tracking/lidar_candidate_clusters
/human_tracking/lidar_candidate_cluster_markers
/human_tracking/lidar_tracklet_debug
/human_tracking/selected_lidar_tracklet
/human_tracking/fused_target_state
/human_tracking/target_state
/human_tracking/fused_target_marker
/human_tracking/selected_tracklet_marker
/human_tracking/selected_target_marker
/human_tracking/target_prediction_gate_marker
/human_tracking/camera_guided_target_points
/human_tracking/target_tracker_debug
/human_tracking/camera_identity_debug
/human_tracking/fusion_timing_debug
/human_tracking/association_hypothesis_markers
```

## Terminal 4 - Open RViz

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

rviz2 -d ~/track_robot_ws/install/track_robot_perception/share/track_robot_perception/rviz/human_tracking.rviz
```

RViz setup:

```text
Fixed Frame: base_link
Add -> TF
Add -> PointCloud2 -> /rslidar_points
Add -> Image -> /human_tracking/target_overlay
Add -> MarkerArray -> /human_tracking/lidar_tracklet_markers
Add -> MarkerArray -> /human_tracking/lidar_candidate_cluster_markers
Add -> MarkerArray -> /human_tracking/selected_tracklet_marker
Add -> MarkerArray -> /human_tracking/selected_target_marker
Add -> MarkerArray -> /human_tracking/target_prediction_gate_marker
Add -> MarkerArray -> /human_tracking/fused_target_marker
Add -> MarkerArray -> /human_tracking/association_hypothesis_markers
Add -> PointCloud2 -> /human_tracking/camera_guided_target_points
```

If `/human_tracking/target_overlay` is not available, use:

```text
Add -> Image -> /zed/zed_node/left/image_rect_color
```

PointCloud2 display settings:

```text
Topic: /rslidar_points
Style: Points
Size: 0.03 or 0.05
Color Transformer: Intensity or FlatColor
```

If RViz drops `/rslidar_points` messages, confirm Terminal 2 static TF is still
running.

## Terminal 5 - Debug Commands

Tracklet manager debug:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 topic echo /human_tracking/lidar_tracklet_debug
```

Selected target tracker debug:

```bash
ros2 topic echo /human_tracking/target_tracker_debug
```

Camera target-lock debug:

```bash
ros2 topic echo /human_tracking/camera_target_debug
```

Clear the current logical target manually:

```bash
ros2 service call /human_tracking/reset_target std_srvs/srv/Trigger '{}'
```

Camera-guided extraction points (Foxy does not support `ros2 topic echo --once`):

```bash
timeout 2s ros2 topic echo /human_tracking/camera_guided_target_points
```

Confirm that generic tracklet markers contain active markers:

```bash
timeout 2s ros2 topic echo /human_tracking/lidar_tracklet_markers
```

Expected fields include `frame_id: base_link`, `action: 0`, and namespaces
`lidar_tracklets` and `lidar_tracklet_labels`. A leading `action: 3` marker is
normal: it clears markers from the previous update before the new `action: 0`
markers are added.

Selected LiDAR tracklet:

```bash
ros2 topic echo /human_tracking/selected_lidar_tracklet
```

Fused target state:

```bash
ros2 topic echo /human_tracking/fused_target_state
```

## Optional - Perception Decision Test

Existing bags do not contain the IMU and Bunker odometry required by the live
outdoor health gate. Run the decision layer in recorded-data mode:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 launch track_robot_decision outdoor_follow_decision.launch.py \
  require_health:=false \
  require_avoidance_feedback:=false \
  require_safety_feedback:=false
```

Run the command-only controller in another terminal. It does not publish real
`cmd_vel` unless explicitly enabled:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 launch track_robot_control target_follow_controller.launch.py
```

Add these RViz displays:

```text
MarkerArray: /follow/decision_markers
MarkerArray: /follow/controller_markers
```

Inspect the typed decision and planned command:

```bash
ros2 topic echo /follow/decision
ros2 topic echo /follow/cmd_vel_planned
```

Topic rates:

```bash
ros2 topic hz /human_tracking/lidar_tracklets
ros2 topic hz /human_tracking/fused_target_state
ros2 topic hz /human_tracking/selected_target_marker
```

Run the regression summary over a non-looping test interval:

```bash
ros2 run track_robot_perception human_tracking_regression_monitor --duration 30
```

For replay-rate comparison, restart the pipeline before each run and replay the
same bag at each rate:

```bash
ros2 bag play ~/track_robot_ws/rosbags/human_tracking_lidar_20260706_150711 --rate 0.5
ros2 bag play ~/track_robot_ws/rosbags/human_tracking_lidar_20260706_150711 --rate 1.0
ros2 bag play ~/track_robot_ws/rosbags/human_tracking_lidar_20260706_150711 --rate 2.0
```

For a machine-comparable result, run the monitor once for each replay rate and
save the reports outside the bag directories:

```bash
ros2 run track_robot_perception human_tracking_regression_monitor \
  --duration 30 --output /tmp/human_tracking_rate_05.json

ros2 run track_robot_perception human_tracking_regression_monitor \
  --duration 30 --output /tmp/human_tracking_rate_10.json

ros2 run track_robot_perception human_tracking_regression_monitor \
  --duration 30 --output /tmp/human_tracking_rate_20.json

ros2 run track_robot_perception human_tracking_compare_runs \
  /tmp/human_tracking_rate_05.json \
  /tmp/human_tracking_rate_10.json \
  /tmp/human_tracking_rate_20.json
```

Restart the tracking launch before each rate. The comparison fails on logical
identity differences, selected-tracklet sequence differences, unconfirmed
tracklet switches, less than 15 Hz processing, or camera/LiDAR offset above
80 ms. The exact sensor-sequence hash is reported separately because deliberate
latest-frame dropping at high replay rates can change the sampled frames
without changing target identity.

Record one additional long-range bag with the target stopping and moving
laterally at approximately 2, 4, 6, 8, and 10 m. The current four bags cover
startup clutter, stop/U-turn, crossing, and LiDAR-only standing behavior but do
not validate the sparse-return boundary.

## Terminal 7 - Run Pipeline Diagnostic

Use this when RViz topics exist but markers do not visibly update. Keep the bag,
static TF, camera tracking, and camera-LiDAR tracklet launch running, then run:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 run track_robot_perception human_tracking_pipeline_diagnostic --duration 20
```

The report tells whether each marker topic is active or only publishing
`DELETEALL`, whether the camera target is locked/visible, whether
camera-guided extraction selected points, and the latest extraction failure
reason from `/human_tracking/target_tracker_debug`.

## Dry-Run Obstacle Safety Test

Keep the bag, static TF, and simplified human tracking stack running. Start the
follow controller and obstacle safety layer without Bunker status requirements
and without the final `/cmd_vel` gate:

For the first avoidance validation, prefer one bag pass without `--loop`.
Foxy's TF buffer reports `TF_OLD_DATA` when a loop jumps backward to the bag's
first timestamp. If looping is necessary, restart the TF-dependent tracking and
obstacle nodes after a timestamp wrap before judging their behavior.

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 launch track_robot_bringup safe_human_following.launch.py \
  require_bunker_status:=false \
  require_rc_state:=false \
  start_cmd_vel_gate:=false \
  allow_latest_tf_fallback:=true
```

The safety supervisor starts disarmed. First confirm the debug topics:

```bash
ros2 topic echo /safety/obstacle_map_debug
ros2 topic echo /follow/avoidance_state
ros2 topic echo /follow/avoidance_debug
ros2 topic echo /safety/state
ros2 topic hz /safety/filtered_obstacle_points
ros2 topic hz /follow/cmd_vel_avoiding
ros2 topic hz /follow/cmd_vel_safe
```

Arm only the offline safety supervisor after both the planned command and
filtered obstacle cloud are fresh:

```bash
ros2 service call /safety/arm std_srvs/srv/Trigger '{}'
```

Add these RViz displays with fixed frame `base_link`:

```text
PointCloud2  /safety/filtered_obstacle_points
Map          /safety/local_obstacle_grid
MarkerArray  /safety/obstacle_markers
MarkerArray  /follow/avoidance_trajectory_markers
MarkerArray  /safety/collision_envelope_markers
MarkerArray  /follow/controller_markers
```

Expected behavior:

```text
- The blue rectangle is the Bunker footprint.
- The orange rectangle is the inflated safety footprint.
- CLEAR publishes the planned command unchanged.
- DIRECT_CLEAR keeps the target controller's original trajectory.
- AVOIDING shows the selected cyan trajectory and modified angular command.
- NO_SAFE_TRAJECTORY publishes exact zero and becomes BLOCKED in `/safety/state`.
- SLOWDOWN reduces linear and angular command together.
- BLOCKED publishes exact zero.
- Stale LiDAR or planned command publishes exact zero.
```

Emergency-stop service test:

```bash
ros2 service call /safety/emergency_stop std_srvs/srv/Trigger '{}'
ros2 service call /safety/reset_emergency_stop std_srvs/srv/Trigger '{}'
```

Reset leaves the supervisor disarmed. Do not start `bunker_base` during this
rosbag test.

## What To Check

First check LiDAR-only tracklet quality in RViz:

```text
/human_tracking/lidar_tracklet_markers
/human_tracking/lidar_candidate_cluster_markers
```

Look for:

```text
- Does the walking person get a tracklet box?
- Does the tracklet ID stay stable?
- Are there too many boxes on static clutter?
- Does the tracklet disappear too often?
```

Then check camera association:

```text
/human_tracking/selected_lidar_tracklet
/human_tracking/fused_target_state
/human_tracking/camera_guided_target_points
/human_tracking/selected_tracklet_marker
/human_tracking/selected_target_marker
/human_tracking/target_prediction_gate_marker
/human_tracking/target_tracker_debug
```

Expected behavior:

```text
- Before gesture lock: no selected tracklet.
- After waving and camera target lock: selected tracklet ID becomes stable after a few frames.
- After camera dropout: camera target identity should stay persistent until stop gesture or reset.
- While target is visible: fused target should be CAMERA_LIDAR_TRACKED when the camera-guided LiDAR anchor or selected tracklet is valid.
- `/human_tracking/selected_target_marker` should show the filtered target estimate while the visible target is active.
- `/human_tracking/selected_tracklet_marker` should show the associated LiDAR tracklet if confirmed.
- `/human_tracking/target_prediction_gate_marker` should show the Kalman acceptance/search gate.
- `/human_tracking/target_tracker_debug` should explain projection, camera-guided anchor, Kalman NIS, and rejection reason.
- In `/human_tracking/camera_target_debug`, `identity_persistent` should remain true after camera dropout unless stop/reset happened.
- In `/human_tracking/target_tracker_debug`, `target_clear_reason` should stay `none` during normal camera dropout.
- Camera-exit LiDAR-only tracking and desk/wall merge recovery are later checks after visible-mode target binding is reliable.
```

## Common Problems

ROS2 daemon stuck:

```bash
ros2 daemon stop
ros2 daemon start
ros2 daemon status
```

Hard reset ROS CLI cache:

```bash
ros2 daemon stop
rm -rf ~/.ros/ros2cli
ros2 daemon start
```

No LiDAR in RViz:

```bash
timeout 2s ros2 topic echo /rslidar_points
ros2 run tf2_ros tf2_echo base_link rslidar
```

If `tf2_echo` fails, restart Terminal 2 static TF publisher.

No camera target:

```bash
timeout 2s ros2 topic echo /zed/zed_node/left/image_rect_color
ros2 topic echo /human_tracking/detections
ros2 topic echo /human_tracking/camera_target
```

Make sure `human_camera_tracking.launch.py` is running and the bag is playing.
