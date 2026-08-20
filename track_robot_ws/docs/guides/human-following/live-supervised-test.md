# Live Supervised Human-Following Test

This procedure advances human following from software verification to bounded
physical testing. Complete each gate in order. A software build or passing
shadow run does not make the feature hardware-ready; Gates A-D require an
operator-reviewed report.

The test profile limits every command layer to `0.05 m/s` linear and
`0.15 rad/s` angular velocity. Keep the physical E-stop reachable. Use one
operator for the laptop and one safety observer near the robot for motion
gates.

## Terminal Setup

Run commands in ROS Domain 20:

```bash
source /opt/ros/foxy/setup.bash
source ~/track_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=20
```

Check dependencies without starting the feature:

```bash
ros2 run track_robot_bringup human_following_ctl doctor \
  --runtime-mode shadow --hardware auto
```

## One-Command Operation

Start without a `/cmd_vel` publisher:

```bash
ros2 run track_robot_bringup human_following_ctl start \
  --runtime-mode shadow --hardware auto
```

Start the confirmed low-speed motion path only at the appropriate gate:

```bash
ros2 run track_robot_bringup human_following_ctl start \
  --runtime-mode active --hardware auto --confirm-motion
```

Inspect or stop the owned feature process:

```bash
ros2 run track_robot_bringup human_following_ctl status
ros2 run track_robot_bringup human_following_ctl stop
```

The stop command disarms, observes a zero safe command, escalates to emergency
stop if required, shuts down `cmd_vel_gate`, verifies that `/cmd_vel` has no
publisher, and then stops only the process group owned by this feature.

## Emergency Stop

For unexpected motion, use the physical E-stop first. The software emergency
stop is an additional path:

```bash
ros2 service call /safety/emergency_stop std_srvs/srv/Trigger '{}'
```

Resetting the software emergency stop does not arm motion. Stop the feature,
identify the fault, reset only under supervision, and restart from the
appropriate gate.

## Runtime Inspection

```bash
ros2 topic echo /human_following/session_state
ros2 topic echo /follow/decision
ros2 topic echo /safety/state
ros2 topic info /cmd_vel --verbose
```

Expected `/cmd_vel` ownership:

- shadow: publisher count `0`;
- confirmed active: publisher count `1`, node `/cmd_vel_gate`.

The session moves through waiting, validating, following, blocked, RC override,
fault, and disarmed states. A valid wave selects the logical target and requests
authorization. The stop gesture revokes authorization and resets that target.
LiDAR-only evidence cannot create the initial motion session.

Useful RViz topics include:

```text
/human_tracking/selected_tracklet_marker
/human_tracking/selected_target_marker
/follow/decision_markers
/follow/controller_markers
/follow/avoidance_trajectory_markers
/safety/collision_envelope_markers
/human_following/supervisor_markers
```

## RC Takeover

Switching the Bunker to RC mode immediately commands zero, disarms the safety
supervisor, revokes the session, and resets the selected logical target. Stick
movement while nominally in CAN mode is a redundant takeover signal.

Returning to CAN mode does not automatically resume following. Confirm healthy
CAN status and all readiness checks, then perform a new wave to create a new
target session. Never treat centered RC sticks or CAN return as authorization.

## Gate A: Shadow Live Graph

1. Keep the robot on the ground but clear the surrounding area. Do not start
   active mode.
2. Run the shadow start command and open the human-following RViz displays.
3. Confirm camera, LiDAR, fused target, decision, avoidance, and safety topics
   are fresh.
4. Wave, walk through camera and LiDAR coverage, stop, turn, and issue the stop
   gesture. Confirm state transitions without robot motion.
5. Confirm `/cmd_vel` has zero publishers throughout the run.
6. Stop with `human_following_ctl stop` and complete a Gate A report.

## Gate B: Tracks Lifted

1. Securely support the Bunker with its tracks lifted. Keep clear of both
   tracks and have the physical E-stop operator ready.
2. Confirm effective limits `0.05 m/s` and `0.15 rad/s`, healthy CAN mode,
   odometry, RC state, LiDAR, and all safety services.
3. Start confirmed active mode. Before waving, verify the tracks do not move.
4. Wave and verify low-speed direction, stop gesture, target loss, stale input,
   software E-stop, and physical E-stop behavior.
5. Select RC mode and confirm autonomous output becomes zero. Return to CAN and
   confirm there is no motion until a new wave.
6. Stop the feature and complete a Gate B report.

## Gate C: Open Ground at 0.05 m/s

1. Use dry, level, open ground with a wide exclusion zone and no bystanders.
2. Place the target in front of the robot for the initial wave and lock.
3. Start active mode and test straight following at the fixed `0.05 m/s` limit.
4. Test stop gesture, target stopping, slow turns, brief target occlusion, RC
   takeover, and target loss. Abort on unexpected acceleration or steering.
5. Do not increase speed. Stop and complete a Gate C report.

## Gate D: Soft Obstacle Validation

1. Keep the Gate C speed and site controls.
2. Use foam blocks or light cardboard boxes. Always use foam or boxes and never
   use a person as a collision obstacle.
3. Test clear passage, slowdown, a blocked direct path, and no-safe-trajectory
   stop. Confirm the planner never reverses.
4. Do not test drop-offs, holes, water, mud, steep slopes, or rigid collision
   targets.
5. Stop and complete a Gate D report.

## Current Limitations

- The top-mounted sensors do not guarantee low obstacle or drop-off detection.
- Terrain hazards include holes, mud, water, loose ground, and steep slopes.
- Weather performance is not validated beyond controlled dry conditions.
- LiDAR-only continuation is bounded; the supervised profile prohibits
  LiDAR-only forward motion.
- Avoidance is local. There is no global detour or Nav2 recovery in this
  feature.
- Prefer a stopped or lost state over following an uncertain target.

Record every gate with [the report template](gate-report-template.md). Do not
advance after a failure until the cause is fixed and the failed gate is repeated.
