# Outdoor Follow Decision Layer

The decision layer converts fused target evidence into explicit motion
permissions. It does not bypass the local trajectory planner, motion safety
supervisor, command gate, RC override, or Bunker watchdog.

## Decision States

```text
WAITING_FOR_TARGET
FOLLOW_CONFIRMED
FOLLOW_LIDAR_LIMITED
UNCERTAIN_HOLD
SEARCH_ROTATE
BLOCKED_HOLD
FAULT_HOLD
TARGET_LOST
RC_OVERRIDE
```

The behavior tree is in `config/follow_behavior_tree.xml`; thresholds are in
`config/outdoor_decision.yaml`. Real velocity output remains disabled by
default. The controller is also capped at 0.15 m/s until braking tests justify
raising its independent limit.

## Command-Only Test

Start the human-tracking perception pipeline and its sensor drivers or rosbag,
then run:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 launch track_robot_decision outdoor_follow_decision.launch.py \
  require_health:=false \
  require_avoidance_feedback:=false \
  require_safety_feedback:=false
```

The health requirement is disabled only because the existing perception bags
do not contain IMU and odometry. Keep it enabled for live outdoor operation.

In another terminal:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20

ros2 launch track_robot_control target_follow_controller.launch.py
```

Inspect outputs:

```bash
ros2 topic echo /follow/decision
ros2 topic echo /follow/decision_debug
ros2 topic echo /follow/cmd_vel_planned
```

RViz displays:

```text
MarkerArray: /follow/decision_markers
MarkerArray: /follow/controller_markers
MarkerArray: /follow/avoidance_trajectory_markers
MarkerArray: /safety/collision_envelope_markers
```

## Live Safety Chain

```bash
ros2 launch track_robot_bringup safe_human_following.launch.py
```

This starts the decision layer, controller, obstacle map, local trajectory
planner, safety supervisor, and command gate. The controller remains disabled,
and the safety supervisor remains disarmed. Do not enable either during initial
visualization and command-only testing.

## Manual Takeover and Reauthorization

RC takeover is a hard transition, not a temporary pause. When `/safety/state`
enters `STATE_RC_OVERRIDE`, the decision node publishes `BEHAVIOR_RC_OVERRIDE`
with motion disabled and requests `/human_tracking/reset_target`. This clears
the gesture-authorized logical target.

Switching the Bunker back to CAN mode does not resume the old target or re-arm
the safety supervisor. Resume requires both actions:

1. Perform the start gesture again to authorize a new logical target.
2. Explicitly call `/safety/arm` after confirming CAN mode and healthy inputs.

This matches the autonomous-approach/Nav2 takeover behavior: manual control
cancels the current autonomous authorization and cannot auto-resume.

## Operating Limits

- Structured outdoor paths and mostly level open ground only.
- Daylight and light rain only when perception health remains usable.
- No drop-off, hole, water, mud, steep-slope, or low-obstacle guarantee with
  the current top-mounted sensors.
- LiDAR-only forward motion is limited to 0.15 m/s and three seconds.
- Search uses zero forward speed, a 120-degree sector, and a four-second limit.
- A blocked local path produces a stop; there is no autonomous global detour.
