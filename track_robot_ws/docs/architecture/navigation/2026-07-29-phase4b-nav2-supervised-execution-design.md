# Phase 4B Nav2 Supervised Execution Design

Date: 2026-07-29  
Frozen baseline: `31fc147a5105399f08ef721b676b0e984df21887`  
Initial world frame: `odom`  
Default runtime mode: `PLANNING_ONLY`

## 1. Purpose and scope

Phase 4B adds supervised, low-speed path execution to the tested Phase 0-4A
semantic-search pipeline. Nav2 owns global planning, local control, behavior
tree navigation, costmaps, lifecycle, and recovery. Track Robot adds only the
adapters and fail-closed supervision required to connect existing semantic
goals and safety components to Nav2.

The initial scope is deliberately narrow:

- static targets;
- short-range navigation in `odom`;
- manual `PoseStamped` goals before semantic execution;
- low-speed forward motion and rotation;
- no map server, AMCL, Point-LIO, or IMU dependency;
- no autonomous reverse or spin recovery;
- semantic execution disabled by default.

Phase 4A remains planning-only and its public topics and default behavior are
unchanged.

## 2. Verified starting architecture

### 2.1 Phase 4A products

The existing `phase4_approach_planner` consumes:

- `/semantic_search/phase4a/selected_target`;
- `/safety/local_obstacle_grid`;
- `/semantic_search/phase4a/localization_state`.

It publishes:

- `/semantic_search/phase4/approach_candidates` (`PoseArray`);
- `/semantic_search/phase4/selected_goal` (`PoseStamped`);
- `/semantic_search/phase4/planned_path` (`Path`);
- `/semantic_search/phase4/diagnostics` (`DiagnosticArray`).

All Phase 4A planning products are in `base_link`. The node rejects
`planning_only=false` and exposes no motion interface.

### 2.2 Odometry and TF

`bunker_base_node` publishes `/odom` and the `odom -> base_link` transform from
measured track velocities at its 50 Hz control rate. It subscribes directly to
the final `/cmd_vel`.

The semantic-search ZED launch disables ZED pose tracking and both ZED TF
publishers. It only adds a static `base_link -> zed_camera_link` transform.
Phase 4B must keep Bunker as the only `odom -> base_link` authority.

### 2.3 Existing obstacle and motion safety

`local_obstacle_map_node` consumes `/rslidar_points`, removes ground and robot
returns, and publishes `/safety/filtered_obstacle_points` and a 12 m rolling
grid in `base_link`.

The existing safety chain is:

`local trajectory planner -> motion_safety_supervisor -> cmd_vel_gate -> Bunker`

The safety supervisor already implements:

- explicit arm/disarm;
- latched software E-stop;
- RC-stick override and disarm;
- Bunker status and CAN-control checks;
- obstacle-cloud, command, base-status, RC, and planner-state watchdogs;
- collision-envelope slowdown and stop;
- hard speed limits.

Phase 4B reuses the supervisor and gate but does not reuse the
human-target-specific local trajectory planner.

## 3. Nav2 availability and option comparison

The Jetson has ROS 2 Foxy Nav2 0.4.7 packages for NavFn, DWB, Regulated Pure
Pursuit, BT Navigator, planner/controller servers, recoveries, costmaps, and
lifecycle management. Smac Planner is not installed.

| Function | Option | Benefit | Risk / cost | Decision |
| --- | --- | --- | --- | --- |
| Global planner | NavFn Dijkstra | Mature Foxy default | Explores more cells | Not selected |
| Global planner | NavFn A* | Installed, deterministic, efficient for short grids | No kinematic heading model | Selected |
| Global planner | Smac | Better kinematic paths | Not installed; new dependency and tuning | Deferred |
| Controller | DWB | Mature and flexible local trajectory search | More tuning, CPU use, oscillation risk | Tested fallback |
| Controller | Regulated Pure Pursuit | Smooth path following, curvature/approach/collision speed regulation | Relies on a good global path | Selected |
| Recovery | Spin/back-up/wait | More autonomous recovery options | Uncommanded reverse/spin is unsafe in first execution stage | Deferred |
| Recovery | Clear costmaps + wait | Observable and non-displacing | May require operator intervention | Selected |

## 4. Target architecture

### 4.1 Nav2 servers

Active navigation uses:

- `planner_server` with NavFn A* and the rolling global costmap;
- `controller_server` with Regulated Pure Pursuit and the rolling local
  costmap;
- `bt_navigator` with the supervised behavior tree;
- `recoveries_server` with wait-only recovery;
- `lifecycle_manager_navigation`.

Foxy embeds global and local costmap nodes inside planner and controller
servers respectively.

### 4.2 Standard interfaces

- manual execution: `nav2_msgs/action/NavigateToPose`;
- shadow planning: `nav2_msgs/action/ComputePathToPose`;
- internal path following: `nav2_msgs/action/FollowPath`;
- global plan visualization: `/plan`;
- local controller plan: `/local_plan`;
- raw Nav2 velocity: `/nav2/cmd_vel_raw`.

### 4.3 Mandatory velocity chain

The only allowed executable chain is:

```text
controller_server
  -> /nav2/cmd_vel_raw
  -> motion_safety_supervisor_node
  -> /nav2/cmd_vel_safe
  -> cmd_vel_gate
  -> /cmd_vel
  -> bunker_base_node
```

The controller server must remap its relative `cmd_vel` output to
`/nav2/cmd_vel_raw`. No Nav2 node may publish `/cmd_vel`. The safety supervisor
starts disarmed and the gate retains its own command timeout.

## 5. Runtime modes and invariants

### `PLANNING_ONLY`

- planner server, global costmap, and planner lifecycle manager only;
- no controller server, BT navigator, recoveries, safety supervisor, gate, or
  semantic action sender;
- must have zero `/cmd_vel` publishers.

### `MANUAL_NAV2_ACTIVE`

- complete Nav2 execution stack and mandatory safety chain;
- manual RViz `NavigateToPose` goals only;
- safety supervisor remains disarmed until an operator explicitly arms it.

### `SEMANTIC_SHADOW`

- Phase 4A semantic goal adapter plus planner server;
- transforms the selected approach goal from `base_link` to `odom`;
- calls `ComputePathToPose` and publishes the shadow path and diagnostics;
- no controller server or BT navigator;
- must have zero velocity output.

### `SEMANTIC_ACTIVE`

- complete Nav2 execution stack plus semantic goal adapter;
- disabled by default;
- requires all of:
  - `runtime_mode=SEMANTIC_ACTIVE`;
  - `enable_semantic_execution=true`;
  - stable and fresh semantic target identity;
  - fresh odometry and valid transform;
  - armed, healthy safety state.

Failure of any gate rejects or cancels the Nav2 goal.

## 6. Semantic goal supervision

The adapter does not implement planning or path following. It:

1. receives the Phase 4A selected goal and target/diagnostic metadata;
2. preserves memory epoch, global object ID, localization epoch, query ID, and
   query version in its diagnostics;
3. rejects non-finite, stale, out-of-range, or unsupported-frame goals;
4. requires repeated confirmation of the same global object ID;
5. transforms the goal to `odom` at the source timestamp;
6. sends `ComputePathToPose` in shadow mode or `NavigateToPose` in active mode;
7. cancels on target loss, target-ID change, stale odometry, safety fault,
   E-stop, RC override, or persistent blocked state;
8. rate-limits re-goaling and ignores insignificant goal jitter.

Because the selected Phase 4A pose does not itself carry object IDs, the
adapter correlates it with the selected semantic object and Phase 4
diagnostics. Any missing or inconsistent correlation fails closed.

## 7. Costmap and controller configuration

### 7.1 Frames and geometry

- global frame: `odom`;
- robot frame: `base_link`;
- odometry topic: `/odom`;
- footprint:
  `[[-0.60,-0.50],[-0.60,0.50],[0.60,0.50],[0.60,-0.50]]`;
- costmap resolution: 0.05 m;
- global rolling window: 12 m x 12 m;
- local rolling window: 6 m x 6 m;
- inflation radius: 0.25 m.

The footprint matches the existing conservative point-cloud envelope rather
than claiming a new mechanical measurement.

### 7.2 Obstacle sources

Both Nav2 costmaps use standard Nav2 obstacle/voxel layers with
`/rslidar_points`. The existing filtered obstacle cloud remains an independent
input to the safety supervisor. This avoids a custom costmap plugin and keeps
the final stop decision independent of Nav2.

Initial update rates:

- local costmap update 10 Hz, publish 5 Hz;
- global costmap update 5 Hz, publish 2 Hz;
- controller 10 Hz;
- global replanning 1 Hz.

### 7.3 Motion limits

Initial Nav2 limits:

- desired linear velocity: 0.10 m/s;
- maximum angular velocity: 0.25 rad/s;
- conservative linear acceleration/deceleration;
- approach velocity scaling enabled;
- curvature and cost regulation enabled;
- 0.8 m semantic standoff inherited from Phase 4A.

The existing safety supervisor keeps harder independent limits of 0.15 m/s
and 0.35 rad/s.

## 8. Failure handling

| Failure | Immediate effect | Nav2 outcome |
| --- | --- | --- |
| Target lost / changed | Semantic adapter rejects or cancels | Goal canceled |
| Stale odometry | Safety supervisor outputs zero; adapter cancels | Goal canceled |
| Stale obstacle cloud / map input | Safety supervisor outputs zero | Replan, then cancel/abort if persistent |
| Blocked path | Controller produces no safe progress | BT clears costmaps, waits, then aborts |
| Software E-stop | Supervisor latches zero and disarms | Active goal canceled |
| RC stick override | Supervisor outputs zero and disarms | Active goal canceled |
| Bunker fault / non-CAN mode | Supervisor outputs zero | Active goal canceled |
| Invalid semantic position / TF | No action goal is sent | Diagnostic reject |

The odometry watchdog is optional in the existing supervisor's default
configuration, but required by the Phase 4B-specific configuration. This
preserves all existing behavior outside Phase 4B.

## 9. Files

New package `src/track_robot/track_robot_navigation`:

- `package.xml`, `CMakeLists.txt`, and installed Python modules;
- `config/nav2_phase4b.yaml`;
- `behavior_trees/navigate_supervised.xml`;
- `launch/phase4b_navigation.launch.py`;
- Python mode policy and supervised semantic adapter;
- `rviz/phase4b_navigation.rviz`;
- unit and launch/config contract tests.

Modified packages:

- `track_robot_safety`: optional odometry watchdog and Nav2-specific config;
- `track_robot_bringup`: Nav2 gate config and package dependency only;
- project documentation and operator guide.

## 10. Regression gates and implementation order

### Gate 0: frozen baseline

At `31fc147`, build the Phase 0-4A dependency closure and run the tests for:

- `track_robot_core`;
- `track_robot_interfaces`;
- `track_robot_lidar_tracking`;
- `track_robot_semantic_memory`;
- `track_robot_semantic_search`;
- `track_robot_safety`;
- `track_robot_bringup`;
- `track_robot_decision`;
- `track_robot_control`.

Baseline reproduced on 2026-07-29:

- 1223 tests;
- 0 errors;
- 0 failures;
- 4 skipped ROS runtime tests;
- deterministic semantic-memory replay passed.

### Commit sequence

1. architecture and acceptance plan only;
2. odometry watchdog with tests and Phase 4B safety config;
3. Nav2 package, parameters, behavior tree, and planning/manual modes;
4. semantic shadow adapter and no-motion proof;
5. default-disabled semantic active mode and cancellation policy;
6. RViz, operator guide, and consolidated regression evidence.

### Acceptance stages

1. static package/config/launch contract tests;
2. pure policy and target-correlation unit tests;
3. mocked odometry, TF, safety-state, and Nav2 action integration;
4. `PLANNING_ONLY` with live sensors, proving no velocity publishers;
5. `SEMANTIC_SHADOW` with live Phase 0-4A inputs, proving valid paths and no
   velocity;
6. `MANUAL_NAV2_ACTIVE` with wheels clear or a controlled test lane, low speed,
   explicit arm, and operator E-stop;
7. `SEMANTIC_ACTIVE` only after all prior gates pass.

Reject a change if it:

- regresses any Phase 0-4A test or deterministic replay;
- changes existing public topics, messages, IDs, or defaults;
- creates a Nav2 publisher on final `/cmd_vel`;
- emits motion in planning-only or shadow mode;
- accepts stale target, odometry, map, or safety data;
- bypasses arm, E-stop, RC override, supervisor, or velocity gate;
- worsens target identity stability without a documented feature flag.
