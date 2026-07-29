# Phase 4B Implementation and Software Validation

## Frozen reference

- Baseline: `31fc147a5105399f08ef721b676b0e984df21887`
- Branch: `feature/phase4b-nav2-supervised`
- ROS: Foxy
- Nav2: 0.4.7
- Test date: 2026-07-29
- Hardware motion: not performed

The baseline dependency closure completed with 1223 tests, 0 errors, 0 failures and 4 skipped ROS runtime tests. Deterministic semantic-memory replay passed.

## Implemented increments

1. `53ffe5d`: architecture and acceptance design;
2. `b7e7619`: optional odometry watchdog plus Phase 4B safety configuration;
3. `a69e039`: NavFn/RPP Nav2 stack, rolling costmaps, planning/manual modes and mandatory velocity remapping;
4. `bb55a80`: semantic reference policy, shadow path action, default-disabled semantic execution and cancellation.

## Runtime evidence

### PLANNING_ONLY

- planner server reached lifecycle `active`;
- no `/cmd_vel` topic existed;
- no controller server or BT navigator ran.

### MANUAL_NAV2_ACTIVE

Observed ROS graph:

- `/nav2/cmd_vel_raw`: two publishers (`controller_server`, `recoveries_server`) and one subscriber (`motion_safety_supervisor_node`);
- `/nav2/cmd_vel_safe`: one publisher (`motion_safety_supervisor_node`) and one subscriber (`cmd_vel_gate`);
- `/cmd_vel`: one publisher (`cmd_vel_gate`);
- safety supervisor began `DISARMED`.

The Foxy recovery server was explicitly remapped after runtime inspection showed that it creates a `cmd_vel` publisher even when only the non-displacing Wait plugin is configured.

### SEMANTIC_SHADOW

- planner server and semantic supervisor ran;
- controller server, BT navigator, recovery server, safety execution chain and velocity gate did not run;
- supervisor reported `HOLD (waiting_for_correlated_inputs)` without complete upstream data;
- `/cmd_vel` did not exist;
- `/nav2/cmd_vel_raw` did not exist;
- `/semantic_navigation/diagnostics` had exactly one publisher.

## Regression result

The final Phase 0-4B regression reports 1260 tests, 0 errors, 0 failures and
4 skipped ROS runtime tests. The navigation package itself reports 29 tests,
0 errors, 0 failures and 0 skipped. Tests cover:

- all four runtime-mode component contracts;
- semantic-active explicit gate;
- NavFn A* and Regulated Pure Pursuit selection;
- rolling `odom` costmaps and Bunker footprint;
- standard LiDAR obstacle layers;
- non-displacing recovery configuration;
- raw/safe/final velocity topic chain;
- semantic identity confirmation and mismatch handling;
- target, goal, diagnostics and odometry freshness;
- target change/loss cancellation;
- safety loss cancellation;
- action dispatch retry;
- shadow-mode no-navigation policy.

## Not yet accepted

These remain hardware acceptance gates, not software claims:

- live Phase 0–4A inputs producing a Nav2 shadow path;
- manual low-speed path following;
- physical footprint and inflation clearance;
- blocked-path, RC override and E-stop injection;
- full semantic-active motion;
- path/control latency and tracking error under movement.

No executable motion was issued during this implementation validation.
