# Phase 5A Unidirectional Search and Phase 4B Handoff Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Phase 5A gather evidence through one-direction bounded Spin goals and hand a confirmed target directly to the already-tested Phase 4B supervised approach stack without restarting the launch.

**Architecture:** Use the full Phase 4B Nav2 stack as the single motion runtime for supervised Phase 5A, add the search-motion adapter beside it, and enable both `wait` and `spin` on the same recoveries server. Generate monotonic absolute evidence headings whose relative Spin deltas all have the same sign.

**Tech Stack:** ROS 2 Foxy, Python/rclpy, ROS 2 launch, Nav2 planner/controller/BT navigator/recoveries, pytest, GoogleTest RViz plugin tests.

## Global Constraints

- ROS domain remains `20`.
- Phase 5A Finding permits rotation only and never publishes forward motion.
- Phase 4B Approach starts only after explicit `Start Approach`.
- Keep one Nav2 stack, one motion safety supervisor and one cmd_vel gate.
- Preserve all public topics, messages, query/global IDs and target-reference fields.
- Preserve RC override, E-stop, watchdog and the complete velocity safety chain.

---

### Task 1: Make the heading policy unidirectional

**Files:**
- Modify: `src/track_robot_semantic_search/test/test_active_search_policy.py`
- Modify: `src/track_robot_semantic_search/test/test_active_search_manager_contract.py`
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/active_search_policy.py`
- Modify: `src/track_robot_semantic_search/track_robot_semantic_search/active_search_manager_node.py`
- Modify: `src/track_robot_semantic_search/config/semantic_search_phase5a.yaml`

**Interfaces:**
- Consumes: configured absolute heading offsets and `SearchForObject.maximum_rotation_angle` as the per-Spin limit.
- Produces: `HeadingDecision.rotation_delta_deg` values with one sign and a maximum cumulative rotation of `270°`.

- [ ] Replace the policy expectation with headings `[45, 90, 135, 180, 225, 270]`, deltas `[45, 45, 45, 45, 45, 45]`, and cumulative rotation `270°`.
- [ ] Run `test_active_search_policy.py` and verify it fails against the alternating sequence.
- [ ] Change the default and YAML heading/evidence offsets to `[45.0, 90.0, 135.0, 180.0, 225.0, 270.0]`.
- [ ] Remove the manager's absolute-offset filter; construct `SearchPolicyConfig` with the goal angle applied only to `maximum_individual_rotation_deg` while `BoundedHeadingPolicy` enforces cumulative budget.
- [ ] Add assertions that every nonzero delta has the same sign, each delta is within the goal limit, and the sequence stops at the cumulative budget.
- [ ] Run the full `track_robot_semantic_search` suite and commit as `fix(phase5a): search headings in one direction`.

### Task 2: Add Spin to the existing Phase 4B Nav2 runtime

**Files:**
- Modify: `src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py`
- Modify: `src/track_robot/track_robot_navigation/test/test_phase5a_nav2_config_contract.py`
- Modify: `src/track_robot/track_robot_navigation/config/nav2_phase4b.yaml`

**Interfaces:**
- Consumes: Nav2 `/spin` and existing `/navigate_to_pose` actions from one recoveries/planner/controller runtime.
- Produces: one recoveries server with `recovery_plugins: [wait, spin]`, bounded Spin parameters, and unchanged Phase 4B planning behavior.

- [ ] Change the contract tests to require both `wait` and `spin` in `nav2_phase4b.yaml`, with `max_rotational_vel: 0.30`, `min_rotational_vel <= 0.10`, `rotational_acc_lim <= 0.50` and `simulate_ahead_time >= 1.0`.
- [ ] Run the two focused config tests and verify they fail because Phase 4B is wait-only.
- [ ] Add the tested Phase 5A `nav2_recoveries/Spin` configuration beside the existing Wait plugin.
- [ ] Run the full `track_robot_navigation` suite and commit as `feat(navigation): share Nav2 spin with semantic approach`.

### Task 3: Replace the isolated rotation launch with the full handoff runtime

**Files:**
- Modify: `src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py`
- Modify: `src/track_robot/track_robot_bringup/test/test_phase5a_no_motion_contract.py`
- Modify: `src/track_robot/track_robot_bringup/test/test_control_cli.py`
- Modify: `src/track_robot/track_robot_bringup/launch/semantic_search_phase5a.launch.py`
- Modify: `src/track_robot/track_robot_bringup/track_robot_bringup/control_cli.py`

**Interfaces:**
- Consumes: `phase4b_navigation.launch.py` in `SEMANTIC_ACTIVE` mode and the existing search-motion adapter executable/config.
- Produces: supervised Phase 5A with one full Nav2 stack plus one search adapter; passive/shadow modes remain motionless.

- [ ] Write launch contract tests requiring `phase4b_navigation.launch.py`, `SEMANTIC_ACTIVE`, semantic execution enabled, a single `search_motion_adapter`, and no `phase5a_rotation.launch.py` include.
- [ ] Add no-motion assertions proving passive/shadow map to `PLANNING_ONLY`, disable semantic and rotation execution, and do not start the adapter or base.
- [ ] Run the focused bringup tests and verify they fail against the isolated rotation launch.
- [ ] Include `phase4b_navigation.launch.py` from Phase 5A, pass `SEMANTIC_ACTIVE` only for supervised mode, and add a conditionally started `search_motion_adapter` node using `active_search_motion.yaml`.
- [ ] Update `semantic_search_ctl run phase5a --rotation-supervised` arguments to select the unified runtime while preserving the user command.
- [ ] Run the full `track_robot_bringup` suite and commit as `feat(phase5a): hand confirmed targets to phase4b navigation`.

### Task 4: Lock the RViz handover contract and update the operator guide

**Files:**
- Modify: `src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_plugin_contract.py`
- Modify: `docs/guides/semantic-search/phase5a-bounded-active-search-test.md`

**Interfaces:**
- Consumes: confirmed `/semantic_search/phase4a/selected_target` and available `/semantic_navigation/authorize_approach`.
- Produces: unchanged two-button workflow: Finding first, Approach only after confirmation.

- [ ] Add a plugin contract assertion that Finding never invokes approach automatically and `Start Approach` continues to use the complete selected target reference.
- [ ] Document the one-direction scan and the expected `Finding confirmed → Start Approach enabled → approach_requested/authorized` sequence.
- [ ] Run the RViz plugin tests and documentation-related bringup tests.
- [ ] Commit as `docs(phase5a): document continuous approach handoff`.

### Task 5: Regression and supervised live acceptance

**Files:**
- No production files.

**Interfaces:**
- Consumes: the complete Phase 0–5A launch on ROS domain 20.
- Produces: measured evidence for monotonic rotation and Phase 4B handoff.

- [ ] Build `track_robot_semantic_search`, `track_robot_navigation`, `track_robot_semantic_search_rviz_plugins` and `track_robot_bringup`.
- [ ] Run all four package suites and require zero failures.
- [ ] Start `semantic_search_ctl run phase5a --rotation-supervised`; verify the ROS graph has one each of planner, controller, recoveries, BT navigator, semantic supervisor, safety supervisor, gate and search adapter.
- [ ] With the target initially outside view, click `Start Finding` once and record Spin deltas; require all nonzero angular commands to use one direction and all linear commands to remain zero.
- [ ] After target confirmation, verify `/semantic_navigation/authorize_approach` exists and the RViz button is enabled for the same memory/global/localization/query reference.
- [ ] Click `Start Approach` once; verify the service accepts the reference and Phase 4B publishes a path through the existing supervised chain.
- [ ] Stop the managed stack and verify no owned ROS processes remain.

