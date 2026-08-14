# Obstacle Safety Layer

This package implements local obstacle-aware human following. It filters the
LiDAR cloud, selects collision-free steering arcs toward the target, predicts
whether the selected Bunker footprint will collide, slows the command when
clearance decreases, and stops inside the braking envelope.

## Command Path

```text
/human_tracking/target_state
  -> target_follow_controller
  -> /follow/cmd_vel_planned
  -> local_trajectory_planner_node
  -> /follow/cmd_vel_avoiding
  -> motion_safety_supervisor_node
  -> /follow/cmd_vel_safe
  -> cmd_vel_gate (50 Hz watchdog)
  -> /cmd_vel
  -> bunker_base (50 Hz SDK refresh and watchdog)
```

The trajectory planner samples differential-drive arcs against the rolling
occupancy grid. The safety supervisor then independently checks the selected
arc against the filtered obstacle points. The supervisor starts disarmed. Real
motion requires fresh LiDAR, fresh Bunker status, fresh RC state, a healthy
base in CAN mode, and an explicit call to `/safety/arm`.

## Local Trajectory Avoidance

The planner runs at 20 Hz. It samples four forward speeds from zero to the
target controller's requested speed and fifteen angular velocities across the
configured range. Every candidate is simulated for up to six seconds.

Each occupancy-grid update creates an 8-connected distance field. A dense
lattice covering the inflated rectangular Bunker footprint is checked along
every candidate arc. Colliding trajectories are rejected. Feasible
trajectories are scored using target heading, progress, obstacle clearance,
desired-command deviation, command continuity, and forward speed.

The direct target command is retained when it is clear. When it is blocked,
the planner commits briefly to one passing side to reduce left/right
oscillation. It never reverses. If no candidate is safe it publishes exact
zero and `STATE_NO_SAFE_TRAJECTORY`; the final supervisor treats that as
`BLOCKED`.

```text
/follow/avoidance_state
/follow/avoidance_debug
/follow/avoidance_trajectory_markers
/follow/cmd_vel_avoiding
```

Avoidance can be disabled for comparison without bypassing the final safety
supervisor:

```bash
ros2 launch track_robot_bringup safe_human_following.launch.py \
  enable_avoidance:=false
```

## Collision Model

The catalog Bunker Pro body is 1.064 m long by 0.845 m wide. Bag `150711`
contains repeatable robot self-returns just outside that rectangle, so the
initial effective software envelope is conservatively 1.20 m by 1.00 m. This
must be checked against the real payload and `base_link`. The software then
adds 0.20 m inflation on every side. It simulates the requested differential
drive arc and checks the inflated rectangle at 0.05 s intervals.

The stop distance is:

```text
v^2 / (2 * braking_deceleration)
  + v * response_latency
  + fixed_stop_margin
```

Current conservative defaults are 0.25 m/s^2 braking deceleration, 0.25 s
response latency, and 0.45 m fixed margin. These values must be replaced by
measurements from the real robot before increasing speed.

## Safety States

```text
DISARMED
WAITING_FOR_DATA
CLEAR
SLOWDOWN
AVOIDING
BLOCKED
SENSOR_STALE
RC_OVERRIDE
BASE_FAULT
EMERGENCY_STOP
```

Any stale input, RC takeover, base fault, or emergency stop produces an
immediate zero output. Bunker `control_mode == 3` is the authoritative RC
takeover signal even with centered sticks; stick movement is retained as a
redundant takeover signal. RC takeover also disarms the supervisor, so motion
does not resume when the sticks return to neutral or the base returns to CAN
mode. A new explicit `/safety/arm` call is required.

`/safety/controller_debug` exposes `bunker_control_mode`,
`rc_control_mode_active`, and `rc_stick_override_active` so the takeover source
can be diagnosed independently.

## Hardware Services

```bash
ros2 service call /safety/arm std_srvs/srv/Trigger '{}'
ros2 service call /safety/disarm std_srvs/srv/Trigger '{}'
ros2 service call /safety/emergency_stop std_srvs/srv/Trigger '{}'
ros2 service call /safety/reset_emergency_stop std_srvs/srv/Trigger '{}'
```

Resetting the software emergency stop does not arm the robot.

## Required Calibration

Before hardware motion, verify:

- `base_link` is at the robot ground center.
- The complete payload stays inside the configured footprint.
- The filtered cloud retains obstacles but removes the floor.
- `ground_z` matches the floor height in `base_link`.
- `/bunker_status` reports vehicle state 0, CAN control mode 1, and error code 0.
- RC stick values remain inside the configured deadband when untouched.
- Measured braking distance is no greater than the configured model.

The top-mounted LiDAR cannot guarantee detection of very low objects or drop
offs. Hardware bumpers and downward-facing sensors remain necessary for those
hazards.
