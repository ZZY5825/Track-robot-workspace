# Phase 5A One-Click Active Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make one `Start Finding` click authorize and execute bounded Nav2 Spin search without a second authorization handshake or a `Retry Stop` UI state.

**Architecture:** Keep `SearchForObject` as the search lifecycle interface and Nav2 Spin as the only rotation executor. Treat an executable `SearchMotionIntent` from a supervised action as already operator-authorized; the motion adapter performs internal safety arm and starts Spin, while RC override, E-stop and watchdog remain authoritative.

**Tech Stack:** ROS 2 Foxy, Python/rclpy, C++/rclcpp/Qt RViz plugin, Nav2 Recoveries Spin, pytest, GoogleTest.

## Global Constraints

- ROS domain remains `20`.
- Phase 5A permits rotation only; forward velocity remains forbidden.
- Nav2 commands continue through `/nav2/cmd_vel_raw` → safety supervisor → `/nav2/cmd_vel_safe` → cmd_vel gate → `/cmd_vel`.
- Passive and shadow modes remain motionless.
- Preserve RC override, E-stop, stale-state checks and watchdog stops.

---

### Task 1: Commit the proven launch and numeric-boundary fixes

**Files:**
- Modify: `src/track_robot/track_robot_bringup/launch/semantic_search_phase5a.launch.py`
- Modify: `src/track_robot_semantic_search/config/semantic_search_phase5a.yaml`
- Modify: `src/track_robot/track_robot_navigation/track_robot_navigation/search_motion_adapter.py`
- Test: `src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py`
- Test: `src/track_robot/track_robot_navigation/test/test_search_motion_adapter.py`

**Interfaces:**
- Consumes: launch arguments `search_mode`, `enable_rotation_execution`.
- Produces: live `ROTATION_SUPERVISED` parameters and float32-safe limit checks.

- [x] Verify the focused tests reproduce and cover the launch precedence and ROS float32 boundary failures.
- [x] Run both package suites and expect zero failures.
- [x] Commit launch ownership and float32 comparison as separate logical commits.

### Task 2: Make executable search intents one-click authorized

**Files:**
- Modify: `src/track_robot/track_robot_navigation/track_robot_navigation/search_motion_adapter.py`
- Modify: `src/track_robot/track_robot_navigation/track_robot_navigation/search_motion_adapter_node.py`
- Test: `src/track_robot/track_robot_navigation/test/test_search_motion_adapter.py`

**Interfaces:**
- Consumes: `SearchMotionIntent.rotation_permitted == true` from a supervised action.
- Produces: automatic internal safety arm followed by one `/spin` goal; no panel authorization RPC is required.

- [x] Add a failing policy test proving an executable supervised intent transitions directly to `rotation_authorized` while shadow and forward intents remain non-executable.
- [x] Add a failing node contract proving `_on_intent()` invokes the internal arm-and-spin path for an accepted executable intent.
- [x] Implement one shared `_arm_and_start_pending_spin()` method used by the automatic path and retained compatibility service.
- [x] Verify rejected arm, stale odometry, stale safety, RC override and E-stop still stop or reject motion.
- [x] Run `track_robot_navigation` tests and commit.

### Task 3: Simplify the RViz search session and stop behavior

**Files:**
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/active_search_session.hpp`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/src/active_search_session.cpp`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/semantic_search_panel.hpp`
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/src/semantic_search_panel.cpp`
- Test: `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_active_search_session.cpp`
- Test: `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_plugin_contract.py`

**Interfaces:**
- Consumes: SearchForObject goal feedback and result.
- Produces: `Start Finding`/`Stop Finding` only; no authorization-pending or retry-only UI state.

- [x] Replace tests that expect `AUTHORIZATION_PENDING`, `AUTHORIZED` and `Retry Stop` with tests for a single `SEARCHING` state and one-click stop.
- [x] Remove the panel's rotation-authorization client, callback, service configuration and cancellation retry timer.
- [x] Make `Stop Finding` send action/adapter cancellation once and restore `Start Finding` on the action terminal result without requiring a second click.
- [x] Keep late callbacks generation-guarded so an old action cannot corrupt a new panel session.
- [x] Run RViz plugin tests and commit.

### Task 4: Close manager state transitions and update operator documentation

**Files:**
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/active_search_manager_node.py`
- Test: `src/track_robot_semantic_search/test/test_active_search_manager_state.py`
- Modify: `docs/guides/semantic-search/phase5a-bounded-active-search-test.md`

**Interfaces:**
- Consumes: adapter states `AUTHORIZED`, `SPIN_REQUESTED`, `SPINNING`, `SPIN_COMPLETED` and terminal faults.
- Produces: one rotation decision per heading and deterministic action termination/cancellation.

- [x] Preserve the unmasked search state and block duplicate rotation decisions while waiting, rotating or settling.
- [x] Document one-click start, one-click stop and expected status sequence.
- [x] Run semantic-search tests and commit.

### Task 5: Regression and live acceptance

**Files:**
- No production files.

**Interfaces:**
- Consumes: the complete Phase 5A launch.
- Produces: live evidence that one click reaches the Bunker command input without forward motion.

- [ ] Build and test `track_robot_semantic_search`, `track_robot_navigation`, `track_robot_semantic_search_rviz_plugins` and `track_robot_bringup`; require zero failures.
- [ ] Start `semantic_search_ctl run phase5a --rotation-supervised` on domain 20.
- [ ] Click `Start Finding` once and verify `SPIN_REQUESTED`/`SPINNING` plus non-zero angular velocity on raw, safe and final command topics.
- [ ] Verify all linear velocity values remain zero.
- [ ] Click `Stop Finding` once and verify zero final command and the button returns to `Start Finding`.
- [ ] Stop all ROS nodes after the live test.
