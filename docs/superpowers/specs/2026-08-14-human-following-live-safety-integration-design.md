# Human-Following Live Safety Integration Design

**Date:** 2026-08-14

**Status:** Approved design

## 1. Objective

Prepare the gesture-triggered human-following feature for supervised Bunker Pro
hardware testing. The implementation must provide one-command startup, explicit
shadow and active modes, gesture-triggered motion authorization, obstacle-aware
command execution, RC takeover, and fail-closed recovery.

Human-following and semantic-search remain separate features. They share the
same physical sensor and base platform, but they do not share perception task
state, target identity, mission state, Nav2 behavior, or feature-specific
supervisors.

## 2. Existing System

The current human-following data path is:

```text
ZED RGB -> YOLO pose -> gesture FSM -> camera target lock
                                      |
RoboSense -> LiDAR tracklets ----------+
                                      v
                         selected target fusion
                                      |
                                      v
                             FollowDecision
                                      |
                                      v
                        target follow controller
                                      |
                                      v
                        local trajectory planner
                                      |
                                      v
                       motion safety supervisor
                                      |
                                      v
                             cmd_vel_gate
                                      |
                                      v
                       /cmd_vel -> Bunker SDK
```

The camera owns semantic identity. The gesture FSM chooses the logical target.
LiDAR provides metric geometry and bounded continuation. The decision layer
decides whether motion is permitted. The safety supervisor and command gate are
the final velocity authorities.

The following capabilities already exist:

- Camera pose tracking and gesture target selection.
- Camera-LiDAR target association and persistent LiDAR tracklets.
- Fused target state and decision output.
- Follow controller command generation.
- Local obstacle map and trajectory sampling.
- Motion safety supervision, command freshness checks, Bunker status checks,
  RC checks, emergency stop services, and a final command watchdog.
- RC takeover based on Bunker control mode or RC stick activity.
- Logical target reset on RC takeover.

## 3. Current Gaps

The remaining gaps are:

1. The latest RC takeover changes exist only in the canonical `main` worktree
   and are not installed in the older root feature worktree.
2. Human-following does not have a feature-owned one-command live startup and
   readiness workflow.
3. Existing semantic-search hardware launch code has feature-specific names
   even though the hardware is shared.
4. The gesture locks a target but does not safely authorize `/safety/arm`.
5. Shadow and active execution are not represented as explicit runtime modes.
6. Initial supervised speed limits are not applied consistently across every
   command-limiting node.
7. Human-following safety does not currently require fresh Bunker odometry.
8. The legacy `/follow/enable_cmd_vel` interface can be confused with the real
   final safety authorization.
9. Hardware acceptance steps and evidence collection are not yet documented as
   mandatory gates.

## 4. Scope

### 4.1 In Scope

- Canonical integration on the local `main` branch.
- Neutral shared hardware launch composition.
- One-command human-following startup and stop commands.
- `SHADOW` and `ACTIVE` runtime modes.
- A dedicated human-following session supervisor.
- Gesture-triggered arm requests after all preconditions pass.
- Immediate disarm for stop gesture, RC takeover, target loss, hard health
  failure, base fault, and emergency stop.
- Consistent supervised-test velocity limits.
- Fresh odometry as an active-mode safety prerequisite.
- Typed session state, debug output, and RViz state markers.
- Hardware-free automated tests and staged hardware acceptance documentation.

### 4.2 Out of Scope

- Nav2 integration for human-following.
- Global path planning or autonomous exploration.
- Automatic reverse motion.
- Drop-off, water, hole, mud, or steep-slope detection.
- Removal of the physical emergency stop requirement.
- Unrestricted LiDAR-only forward following.
- Higher-speed operation before measured braking validation.
- Changes to YOLO models or the camera-LiDAR fusion algorithm.

## 5. Repository and Integration Strategy

The canonical implementation target is:

```text
/home/track-robot/track_robot_ws/.worktrees/main-test
branch: main
```

The older root worktree remains on `feature/semantic-search-phase4a` and is not
used as an implementation target. Human-following changes are not copied into
that branch.

Integration uses focused commits in this order:

1. Existing RC takeover implementation and tests.
2. Neutral shared hardware launch extraction.
3. Human-following session interface and policy.
4. Session supervisor node.
5. Live profiles and launch composition.
6. One-command CLI and readiness checks.
7. Documentation and hardware gate tooling.

Unrelated user files and unrelated semantic-search work are excluded from all
commits.

## 6. Architecture

### 6.1 Shared Hardware Layer

Create a neutral shared launch entry point:

```text
track_robot_bringup/launch/track_robot_hardware.launch.py
```

It composes the existing ZED2i, RoboSense, Bunker base, robot description, and
canonical TF launch adapters. It exposes explicit start flags for each hardware
module so the process manager can start only missing components.

Existing semantic-search hardware launch files remain supported as compatibility
wrappers around the neutral launch. Their public arguments remain unchanged.
Human-following includes the neutral launch and does not include semantic task
nodes.

### 6.2 Human-Following Feature Layer

Create:

```text
track_robot_bringup/launch/human_following_live.launch.py
```

This launch composes:

- Human camera tracking and gesture FSM.
- LiDAR tracklet manager and selected-target fusion.
- Perception health monitor.
- Follow behavior decision node.
- Target follow controller.
- Local obstacle map.
- Local trajectory planner.
- Motion safety supervisor.
- Human-following session supervisor.
- `cmd_vel_gate` only in `ACTIVE` mode.

The launch does not duplicate hardware driver definitions. Hardware start
selection is supplied by the one-command CLI.

### 6.3 Command Ownership

The active command path is exactly:

```text
/follow/cmd_vel_planned
  -> /follow/cmd_vel_avoiding
  -> /follow/cmd_vel_safe
  -> cmd_vel_gate
  -> /cmd_vel
  -> bunker_base
```

`cmd_vel_gate` must be the only `/cmd_vel` publisher. The Bunker driver remains
the only subscriber that converts `/cmd_vel` into SDK motion commands.

The safety supervisor's `/safety/arm` service is the sole final motion
authorization. The controller continues publishing planned commands in shadow
mode so the complete decision path can be inspected without enabling motion.

The legacy `/follow/enable_cmd_vel` service and `/follow/cmd_vel` topic remain
temporarily for standalone compatibility. The live launch does not use them and
prints a deprecation warning. They are not connected to `/cmd_vel`.

## 7. Runtime Modes

### 7.1 SHADOW

`SHADOW` is the default.

- Start hardware, perception, fusion, decision, controller, avoidance, safety,
  and the session supervisor.
- Do not start `cmd_vel_gate`.
- Never call `/safety/arm`.
- Permit full debug commands and RViz visualization.
- Require that `/cmd_vel` has no publisher owned by this feature.
- Reject any request to enable active execution at runtime.

### 7.2 ACTIVE

`ACTIVE` must be explicitly requested with motion confirmation.

- Start `cmd_vel_gate`.
- Keep the motion safety supervisor disarmed after startup.
- Allow one new valid start gesture to create a pending target authorization.
- Request `/safety/arm` only after every arm precondition passes.
- Never restore a previous authorization after RC takeover, stop gesture,
  target loss, hard fault, or emergency stop.

The CLI rejects active startup unless both are supplied:

```text
--runtime-mode active
--confirm-motion
```

## 8. Human-Following Session Supervisor

### 8.1 Responsibility

Add `human_following_supervisor_node`. It manages authorization and lifecycle
only. It does not detect gestures, track targets, calculate velocity, plan a
trajectory, or publish `/cmd_vel`.

### 8.2 Inputs

```text
/human_tracking/gesture_state       track_robot_interfaces/GestureState
/human_tracking/camera_target       track_robot_interfaces/CameraTarget
/follow/decision                    track_robot_interfaces/FollowDecision
/perception/health                  track_robot_interfaces/PerceptionHealth
/follow/avoidance_state             track_robot_interfaces/AvoidanceState
/safety/state                       track_robot_interfaces/SafetyState
/bunker_status                      bunker_msgs/BunkerStatus
```

### 8.3 Service Clients

```text
/safety/arm                         std_srvs/Trigger
/safety/disarm                      std_srvs/Trigger
/human_tracking/reset_target        std_srvs/Trigger
```

### 8.4 Outputs

```text
/human_following/session_state      track_robot_interfaces/HumanFollowingSession
/human_following/supervisor_debug   std_msgs/String
/human_following/supervisor_markers visualization_msgs/MarkerArray
```

### 8.5 Typed Session Message

Add `HumanFollowingSession.msg`:

```text
uint8 MODE_SHADOW=0
uint8 MODE_ACTIVE=1

uint8 STATE_STARTING=0
uint8 STATE_WAITING_FOR_GESTURE=1
uint8 STATE_VALIDATING_TARGET=2
uint8 STATE_ARMING=3
uint8 STATE_FOLLOWING=4
uint8 STATE_BLOCKED=5
uint8 STATE_RC_OVERRIDE=6
uint8 STATE_FAULT=7
uint8 STATE_DISARMED=8

std_msgs/Header header
uint8 runtime_mode
uint8 state
int32 logical_target_id
bool motion_session_enabled
bool target_authorized
bool arm_request_pending
bool safety_armed
bool rc_override_active
float32 target_confidence
string reason
```

Safety decisions consume typed messages. JSON debug output is not used as a
control input.

## 9. Authorization State Machine

```text
STARTING
  -> WAITING_FOR_GESTURE
  -> VALIDATING_TARGET
  -> ARMING
  -> FOLLOWING
```

Additional states are `BLOCKED`, `RC_OVERRIDE`, `FAULT`, and `DISARMED`.

### 9.1 Start Gesture

The supervisor accepts only a new `GestureState.command == "start_tracking"`
event with `trigger_active == true`. It stores the gesture timestamp and visual
track ID. Repeated publication of the same event cannot create repeated arm
requests.

The gesture must be followed by a locked `CameraTarget` whose
`visual_track_id` matches the gesture track ID. The resulting logical target ID
becomes the authorization reference. `FollowDecision.logical_target_id` must
match that reference before arm is requested.

### 9.2 Initial Arm Preconditions

All conditions must hold simultaneously:

- Runtime mode is `ACTIVE` and CLI motion confirmation was supplied.
- A fresh, previously unused start gesture exists.
- Camera target lock is `LOCK_TARGET_LOCKED`.
- Camera identity is `IDENTITY_CONFIRMED`.
- Camera target is visible.
- `FollowDecision` is `BEHAVIOR_FOLLOW_CONFIRMED`.
- Follow decision logical target ID matches the authorized target ID.
- Target source is `camera_lidar`.
- Target and decision confidence satisfy the existing confirmed thresholds.
- Perception health is `HEALTHY` for initial arm.
- Bunker control mode is CAN (`1`).
- Bunker vehicle state and error code are zero.
- Safety state is disarmed without RC override, base fault, or emergency stop.
- Obstacle planner is fresh and not waiting, stale, or no-safe-trajectory.
- Every required input is within its configured freshness timeout.
- `/safety/arm` service is available.

The arm service performs the authoritative second validation of command, cloud,
Bunker, RC, planner, and odometry freshness. The supervisor sends only one arm
request per authorization attempt and waits for the asynchronous response.

An arm rejection clears the pending request and enters `FAULT`. It does not
retry continuously. A new start gesture is required after the fault clears.

### 9.3 Stop Gesture

A valid `stop_tracking` event for the authorized target causes:

1. Immediate `/safety/disarm` request.
2. `/human_tracking/reset_target` request.
3. Authorization clear.
4. Transition to `WAITING_FOR_GESTURE` after safety confirms disarmed.

### 9.4 RC Takeover

RC takeover is authoritative when Bunker control mode is RC (`3`), even when
the sticks are centered. Stick movement remains a redundant early indication
inside the safety supervisor.

On RC takeover:

1. Safety immediately publishes zero and disarms itself.
2. The session supervisor cancels any pending arm request.
3. The session supervisor clears target authorization.
4. The target reset service is requested.
5. Session state becomes `RC_OVERRIDE`.

Returning to CAN does not restore the target or arm state. The session returns
to `WAITING_FOR_GESTURE`; a new wave is required.

### 9.5 Target Evidence Changes

- `FOLLOW_CONFIRMED`: normal motion after arm.
- `FOLLOW_LIDAR_LIMITED`: permitted only for an already authorized target.
  Forward motion remains disabled; bounded angular correction is allowed.
- `UNCERTAIN_HOLD`: command is zero. Authorization may be retained for at most
  one second for the same logical target.
- `SEARCH_ROTATE`: active-mode motion authorization is cleared for the first
  hardware milestone. Search markers remain visible in shadow mode.
- `TARGET_LOST`: immediate disarm, target reset, and authorization clear.
- Logical target ID change: immediate disarm and authorization clear.

### 9.6 Obstacles

`BLOCKED` or `NO_SAFE_TRAJECTORY` produces zero output. The authorized target
and arm state may be retained for up to ten seconds so following can resume
when a temporary obstacle clears.

After ten seconds of continuous blocking, the supervisor disarms and clears
motion authorization. The target may remain visible for diagnostics, but a new
wave is required before motion.

### 9.7 Hard Faults

The following conditions immediately disarm and clear authorization:

- Perception `UNSAFE` or `STALE`.
- Safety `STATE_SENSOR_STALE`.
- Safety `STATE_BASE_FAULT`.
- Safety `STATE_EMERGENCY_STOP`.
- Stale supervisor input.
- Missing required TF reported by perception health.
- Session logical target mismatch.

Emergency stop reset never arms the robot. A new wave is required after reset.

## 10. Supervised-Test Configuration

Add explicit profiles:

```text
human_following_shadow.yaml
human_following_supervised_test.yaml
```

The supervised-test profile uses:

```yaml
max_linear_x: 0.05
max_angular_z: 0.15
follow_distance: 2.0
linear_accel_limit: 0.05
angular_accel_limit: 0.15
require_odom: true
odom_timeout_sec: 0.25
allow_lidar_only_forward_motion: false
blocked_disarm_timeout_sec: 10.0
uncertain_authorization_timeout_sec: 1.0
```

The same `max_linear_x` and `max_angular_z` values override the decision,
controller, local planner, motion safety supervisor, and command gate. Startup
fails if their effective limits differ.

The existing conservative collision footprint remains unchanged until the full
payload is measured. Ground height, self-filter footprint, collision footprint,
and braking parameters are hardware acceptance inputs, not assumed software
facts.

## 11. One-Command Operation

Add a `human_following_ctl` executable in `track_robot_bringup`. It reuses the
existing hardware selection, readiness, process ownership, environment, and
bounded shutdown patterns used by semantic-search.

### 11.1 Start Shadow

```bash
ros2 run track_robot_bringup human_following_ctl start \
  --runtime-mode shadow \
  --hardware auto
```

### 11.2 Start Active

```bash
ros2 run track_robot_bringup human_following_ctl start \
  --runtime-mode active \
  --hardware auto \
  --confirm-motion
```

### 11.3 Hardware Modes

- `--hardware auto`: reuse healthy external hardware publishers and start only
  missing required hardware modules.
- `--hardware external`: start no hardware and fail readiness if required
  external publishers are unavailable.

The CLI records only processes that it starts. It never stops externally owned
hardware.

### 11.4 Stop

```bash
ros2 run track_robot_bringup human_following_ctl stop
```

Shutdown order is:

1. Request `/safety/disarm`.
2. Verify zero safe command or issue bounded emergency stop fallback.
3. Stop `cmd_vel_gate`.
4. Stop feature nodes.
5. Stop only CLI-owned hardware processes.

## 12. Readiness Contract

Before `ACTIVE` execution is considered ready, verify:

- Exactly one image publisher on the configured ZED image topic.
- Fresh ZED camera info and IMU.
- Exactly one RoboSense point cloud publisher.
- Fresh Bunker status, RC state, and odometry.
- CAN control mode.
- Healthy base with zero error code.
- Required base-to-camera and base-to-LiDAR TF.
- Fresh `track_robot_center` and `base_link` relationship used by fusion.
- Human perception, tracklet manager, and fusion outputs are live.
- Perception health topic is live.
- Obstacle map and planner outputs are live.
- Safety services are available.
- No unexpected `/cmd_vel` publisher.
- In active mode, `cmd_vel_gate` becomes the sole `/cmd_vel` publisher.

Readiness failures prevent active execution and report the exact missing or
duplicated dependency.

## 13. Automated Validation

Automated tests must prove:

1. Shadow mode never calls `/safety/arm` after a wave.
2. Active mode without motion confirmation is rejected.
3. A wave without confirmed camera-LiDAR target evidence cannot arm.
4. A valid confirmed target calls arm exactly once.
5. Arm service rejection enters fault and does not create a retry loop.
6. Stop gesture disarms and resets the target.
7. Neutral sticks in RC control mode disarm immediately.
8. Returning to CAN does not restore authorization.
9. A new wave after CAN return may authorize a new arm attempt.
10. Target lost, health stale, base fault, and emergency stop disarm.
11. Short blocking retains authorization; blocking timeout disarms.
12. LiDAR-only evidence cannot perform initial arm.
13. Shadow launch has no feature-owned `/cmd_vel` publisher.
14. Active launch has exactly one feature-owned `/cmd_vel` publisher.
15. The supervised speed limits are equal across all five limiting nodes.
16. Existing semantic-search hardware launch contracts remain valid.
17. CLI stop requests disarm before stopping the command gate.
18. CLI process cleanup does not terminate external hardware.

ROS launch tests use isolated ROS domain IDs to prevent parallel package tests
from exchanging safety messages.

## 14. Hardware Acceptance Gates

### Gate A: Shadow

- Run the complete perception and command-planning pipeline.
- Verify target, planned trajectory, collision envelope, and session markers.
- Verify `/cmd_vel` is absent.

### Gate B: Active With Tracks Lifted

- Keep the Bunker tracks off the ground.
- Wave and verify automatic arm only after all conditions pass.
- Verify linear and angular command signs.
- Verify stop gesture, RC takeover, E-stop, stale sensor, and target loss all
  produce zero and disarm.
- Verify CAN return does not resume.

### Gate C: Open-Ground Low Speed

- Use the supervised-test profile with `0.05 m/s` linear limit.
- Keep the person two to three metres ahead.
- Test straight motion, slow lateral motion, stopping, and turning.
- Keep an operator physically ready on the hardware emergency stop.

### Gate D: Soft Obstacles

- Use boxes or foam obstacles.
- Validate clear, slowdown, avoiding, blocked, and blocking timeout states.
- Do not use people as collision-test obstacles.

Each gate produces a dated report containing commands, effective parameters,
topic rates, state transitions, failures, and operator observations. A failed
gate blocks progression to the next gate.

## 15. Documentation

Update or add:

- Human-following architecture and reinforcement status.
- One-command startup and shutdown guide.
- Shadow and active mode contract.
- Gesture authorization and RC takeover behavior.
- Hardware readiness commands.
- RViz displays and expected markers.
- Gate A-D supervised test procedure.
- Emergency stop and recovery procedure.
- Known limitations for low obstacles, drop-offs, terrain, weather, and
  LiDAR-only continuation.

## 16. Migration and Compatibility

- Existing semantic-search launch entry points remain valid.
- Existing human perception topics remain unchanged.
- Existing decision, controller, avoidance, and safety topics remain unchanged.
- `HumanFollowingSession` is additive.
- Legacy controller direct-output interfaces remain temporarily available but
  are not part of the live safety path.
- The default runtime mode is shadow.
- Existing root feature worktrees are not rewritten or cleaned.

## 17. Rollback

Each architecture stage is a separate commit. Rollback can stop at any stage
without enabling motion because:

- Shadow is the default mode.
- Active mode requires explicit motion confirmation.
- Safety starts disarmed.
- The command gate is absent in shadow mode.
- Hardware launch compatibility wrappers remain available.

Removing or disabling the new session supervisor leaves the existing perception
and dry-run command pipeline usable.

## 18. Definition of Done

Software implementation is complete when:

- The canonical main worktree contains focused, passing commits.
- One-command shadow and active startup work without duplicated hardware.
- Shadow cannot publish `/cmd_vel`.
- Active gesture authorization arms only after all prerequisites pass.
- Stop gesture, RC takeover, target loss, hard fault, and E-stop disarm.
- CAN return never resumes the old target.
- Supervised limits are consistent across the complete command chain.
- All new and existing affected package tests pass in parallel.
- Documentation contains an executable Gate A-D test procedure.

Hardware readiness is not claimed until Gates A-D are executed on the Bunker
and their reports are reviewed.
