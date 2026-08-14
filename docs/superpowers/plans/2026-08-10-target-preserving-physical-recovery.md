# Target-Preserving Physical Recovery Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add bounded Nav2 Spin and collision-checked BackUp recovery to an authorized static semantic approach without losing the pursued target, frozen odom goal, global ID, query, or operator authorization.

**Architecture:** `semantic_navigation_supervisor` remains the sole owner of the semantic mission and coordinates a pure recovery state machine. Nav2 Foxy executes path following, costmap clearing, Spin, and BackUp; every velocity remains `/nav2/cmd_vel_raw -> motion_safety_supervisor -> cmd_vel_gate -> /cmd_vel`. The feature is disabled by default and is enabled only in `SEMANTIC_ACTIVE` through an explicit launch/CLI flag.

**Tech Stack:** ROS 2 Foxy, Python 3/rclpy actions, Nav2 0.4.7 (`NavigateToPose`, `Spin`, `BackUp`), diagnostic_msgs, Qt/RViz2, pytest/ament_cmake_pytest, gtest.

## Global Constraints

- Preserve `docs/superpowers/specs/2026-08-10-target-preserving-physical-recovery-design.md`.
- Do not change Phase 1–3 target selection, Phase 2 ID/lifecycle ownership, or the frozen Phase 4B odom target.
- Never publish recovery velocity directly to `/cmd_vel`.
- `PLANNING_ONLY` and `SEMANTIC_SHADOW` must produce no motion.
- Use the existing `0.88 m x 0.80 m` footprint for recovery collision prediction.
- Spin defaults to one fixed mission direction, `30 deg`, at most `0.30 rad/s`.
- BackUp defaults to `0.25 m` at `0.10 m/s`; accept only `0.20–0.30 m`.
- Stale odometry/cloud/base state, RC override, E-stop, base fault, or predicted collision prohibits physical recovery.
- One logical change per commit; no unrelated perception, TF, URDF, planner, or controller changes.
- Stop all ROS nodes after every live test. Never run physical recovery unattended.

## File and Responsibility Map

- Create `track_robot_ws/src/track_robot/track_robot_navigation/track_robot_navigation/physical_recovery.py`: pure stages, decisions, cooldown and cycle bounds; no ROS or target ownership.
- Create `track_robot_ws/src/track_robot/track_robot_navigation/test/test_physical_recovery.py`: deterministic policy tests.
- Modify `track_robot_ws/src/track_robot/track_robot_navigation/config/nav2_phase4b.yaml`: load Foxy BackUp.
- Modify `track_robot_ws/src/track_robot/track_robot_navigation/config/semantic_navigation.yaml`: disabled-by-default parameters.
- Modify `track_robot_ws/src/track_robot/track_robot_navigation/launch/phase4b_navigation.launch.py`: rewrite the explicit feature flag.
- Modify `semantic_navigation_supervisor_node.py`: dispatch `/spin` and `/back_up`, preserve the mission and publish diagnostics.
- Modify Phase 4B/5A bringup launch/CLI: expose an explicit opt-in flag.
- Modify the semantic-search RViz panel: read-only recovery state display, no new motion button.
- Update Phase 4B/5A test guides and create an evidence template.

---

## Pre-Implementation Baseline Gate

- [ ] **Step 1: Create an isolated feature branch**

```bash
cd /home/track-robot/track_robot_ws/.worktrees/main-integration
git status --short --branch
git switch -c feature/target-preserving-physical-recovery
```

Expected: clean tree; branch starts at or after `66da685`.

- [ ] **Step 2: Build and test the current baseline**

```bash
cd /home/track-robot/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select \
  track_robot_navigation track_robot_bringup \
  track_robot_semantic_search_rviz_plugins --symlink-install
source install/setup.bash
colcon test --packages-select \
  track_robot_navigation track_robot_bringup \
  track_robot_semantic_search_rviz_plugins \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failures. Stop if the baseline fails; record the existing failure instead of changing recovery code.

---

### Task 1: Add the Pure Bounded Recovery State Machine

**Files:**
- Create: `track_robot_ws/src/track_robot/track_robot_navigation/track_robot_navigation/physical_recovery.py`
- Create: `track_robot_ws/src/track_robot/track_robot_navigation/test/test_physical_recovery.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/CMakeLists.txt`

**Interfaces:**
- Produces `RecoveryStage`, `RecoveryCommand`, `RecoveryDecision`, `PhysicalRecoveryPolicy`.
- `next_command(now_s)` returns only `NAVIGATE`, `SPIN`, `BACK_UP`, or `HOLD`.
- The module never receives or mutates target IDs, queries, poses, ROS messages, or action handles.

- [ ] **Step 1: Write the failing policy tests**

Create tests covering this exact sequence:

```python
value = PhysicalRecoveryPolicy(
    enabled=True, cooldown_sec=2.0, maximum_cycles=2)
assert value.next_command(10.0).command is RecoveryCommand.NAVIGATE

assert value.navigation_aborted(11.0).command is RecoveryCommand.SPIN
assert value.recovery_finished(
    RecoveryCommand.SPIN, True, 12.0).command is RecoveryCommand.NAVIGATE

assert value.navigation_aborted(13.0).command is RecoveryCommand.BACK_UP
assert value.recovery_finished(
    RecoveryCommand.BACK_UP, True, 14.0).command is RecoveryCommand.NAVIGATE

hold = value.navigation_aborted(15.0)
assert hold.command is RecoveryCommand.HOLD
assert hold.cycle == 1
assert value.next_command(16.9).command is RecoveryCommand.HOLD
assert value.next_command(17.0).command is RecoveryCommand.NAVIGATE
```

Also test:

- disabled policy always returns legacy Navigate;
- failed Spin advances to BackUp without changing `spin_sign`;
- failed BackUp enters Hold;
- after maximum physical cycles, future retries are replan-only and never Spin/BackUp;
- cooldown outside `[0.1, 10.0]` and cycles outside `[1, 5]` raise `ValueError`.

- [ ] **Step 2: Register and run the failing test**

Add `test_physical_recovery` to `CMakeLists.txt` using the existing `ament_add_pytest_test` pattern.

```bash
cd /home/track-robot/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
PYTHONPATH=src/track_robot/track_robot_navigation \
pytest -q src/track_robot/track_robot_navigation/test/test_physical_recovery.py
```

Expected: FAIL because the module does not exist.

- [ ] **Step 3: Implement the minimal pure policy**

Use these exact public enums/data:

```python
class RecoveryStage(Enum):
    IDLE = 'idle'
    SPIN = 'spin'
    NAVIGATE_AFTER_SPIN = 'navigate_after_spin'
    BACK_UP = 'back_up'
    NAVIGATE_AFTER_BACK_UP = 'navigate_after_back_up'
    HOLD = 'hold'
    REPLAN_ONLY = 'replan_only'


class RecoveryCommand(Enum):
    NAVIGATE = 'navigate'
    SPIN = 'spin'
    BACK_UP = 'back_up'
    HOLD = 'hold'


@dataclass(frozen=True)
class RecoveryDecision:
    command: RecoveryCommand
    stage: RecoveryStage
    cycle: int
    reason: str
    not_before_s: float
```

Transitions:

```text
Navigate abort -> SPIN
Spin success -> Navigate-after-Spin
Spin failure -> BACK_UP
Navigate-after-Spin abort -> BACK_UP
BackUp success -> Navigate-after-BackUp
BackUp failure or Navigate-after-BackUp abort -> HOLD
Hold cooldown -> next cycle Navigate
maximum physical cycles -> replan-only Hold/Navigate loop
```

`reset()` returns to `IDLE`, cycle zero and retains the configured spin direction.

- [ ] **Step 4: Run tests and commit**

```bash
PYTHONPATH=src/track_robot/track_robot_navigation \
pytest -q src/track_robot/track_robot_navigation/test/test_physical_recovery.py
git add \
  src/track_robot/track_robot_navigation/track_robot_navigation/physical_recovery.py \
  src/track_robot/track_robot_navigation/test/test_physical_recovery.py \
  src/track_robot/track_robot_navigation/CMakeLists.txt
git commit -m "feat(navigation): add bounded physical recovery policy"
```

Expected: all policy tests PASS.

---

### Task 2: Configure Foxy BackUp and the Explicit Feature Gate

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/config/nav2_phase4b.yaml`
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/config/semantic_navigation.yaml`
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/launch/phase4b_navigation.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/test/test_launch_contract.py`

**Interfaces:**
- Nav2 provides `/spin` (`nav2_msgs/action/Spin`) and `/back_up` (`nav2_msgs/action/BackUp`).
- Supervisor receives bounded physical-recovery parameters.
- Default behavior remains false/unchanged.

- [ ] **Step 1: Change config/launch tests first**

Replace the old no-translation recovery assertion with:

```python
recoveries = _params()['recoveries_server']['ros__parameters']
assert recoveries['recovery_plugins'] == ['wait', 'spin', 'back_up']
assert recoveries['back_up']['plugin'] == 'nav2_recoveries/BackUp'
assert recoveries['costmap_topic'] == 'local_costmap/costmap_raw'
assert recoveries['footprint_topic'] == 'local_costmap/published_footprint'
```

Extend supervisor config assertions:

```python
assert params['physical_recovery_enabled'] is False
assert params['recovery_spin_angle_rad'] == pytest.approx(0.523599)
assert params['recovery_backup_distance_m'] == pytest.approx(0.25)
assert params['recovery_backup_speed_mps'] == pytest.approx(0.10)
assert params['maximum_physical_recovery_cycles'] == 2
```

Assert the launch declares `physical_recovery_enabled` with `default_value='false'` and rewrites it into `semantic_params`.

- [ ] **Step 2: Confirm tests fail**

```bash
PYTHONPATH=src/track_robot/track_robot_navigation \
pytest -q \
  src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py \
  src/track_robot/track_robot_navigation/test/test_launch_contract.py
```

Expected: FAIL because BackUp and the flag are absent.

- [ ] **Step 3: Add BackUp and conservative parameters**

Use this recoveries block:

```yaml
recovery_plugins: [wait, spin, back_up]
wait:
  plugin: nav2_recoveries/Wait
spin:
  plugin: nav2_recoveries/Spin
  simulate_ahead_time: 1.5
  max_rotational_vel: 0.30
  min_rotational_vel: 0.08
  rotational_acc_lim: 0.50
back_up:
  plugin: nav2_recoveries/BackUp
```

Add to `semantic_navigation.yaml`:

```yaml
physical_recovery_enabled: false
recovery_spin_angle_rad: 0.523599
recovery_spin_clockwise: false
recovery_backup_distance_m: 0.25
recovery_backup_speed_mps: 0.10
recovery_cooldown_sec: 2.0
maximum_physical_recovery_cycles: 2
```

Rewrite the launch argument into the supervisor YAML. Keep both controller and recoveries server on `NAV2_CMD_REMAPPINGS`.

- [ ] **Step 4: Run tests and commit**

```bash
PYTHONPATH=src/track_robot/track_robot_navigation \
pytest -q \
  src/track_robot/track_robot_navigation/test/test_nav2_config_contract.py \
  src/track_robot/track_robot_navigation/test/test_launch_contract.py
git add src/track_robot/track_robot_navigation/{config,launch,test}
git commit -m "feat(navigation): gate Nav2 physical recoveries"
```

Expected: both test files PASS.

---

### Task 3: Integrate Recovery Actions Without Losing the Mission

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/track_robot_navigation/semantic_navigation_supervisor_node.py`
- Create: `track_robot_ws/src/track_robot/track_robot_navigation/test/test_physical_recovery_supervisor.py`
- Modify: `test/test_semantic_authorization.py`
- Modify: `test/test_static_target_mission.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/CMakeLists.txt`

**Interfaces:**
- Consumes the policy, `/spin`, `/back_up`, SafetyState and frozen mission goal.
- Retains `_authorized_reference`, `_authorized_target_anchor_xy`, `_mission_goal` and operator authorization through recovery.
- `BackUp.Goal.target.x` is negative; `speed` is positive.

- [ ] **Step 1: Write mission-preservation harness tests**

Test this sequence:

```python
original_reference = (11, 22, 33, 44, 1)
original_anchor = (2.30, 0.0)
original_goal = 'frozen-odom-goal'

harness.on_navigation_result(GoalStatus.STATUS_ABORTED)
assert harness.pending_recovery is RecoveryCommand.SPIN
harness.on_recovery_result(RecoveryCommand.SPIN, GoalStatus.STATUS_SUCCEEDED)
assert harness.next_command is RecoveryCommand.NAVIGATE
harness.on_navigation_result(GoalStatus.STATUS_ABORTED)
assert harness.pending_recovery is RecoveryCommand.BACK_UP

assert harness._authorized_reference == original_reference
assert harness._authorized_target_anchor_xy == original_anchor
assert harness._mission_goal == original_goal
assert harness.disarm_requests == 0
```

Also test failed Spin -> BackUp, failed/rejected BackUp -> Hold, successful Navigate -> mission completion, explicit cancel -> mission clear.

- [ ] **Step 2: Write fail-closed preflight tests**

Add a pure `_physical_recovery_preflight_failure(safety, odom_age_sec)` helper and test:

```python
assert preflight(clear_safety, 0.05) is None
assert preflight(cloud_stale, 0.05) == 'recovery_cloud_stale'
assert preflight(rc_override, 0.05) == 'recovery_rc_override'
assert preflight(estop, 0.05) == 'recovery_emergency_stop'
assert preflight(base_fault, 0.05) == 'recovery_base_fault'
assert preflight(clear_safety, 0.30) == 'recovery_odometry_stale'
```

Allow fresh `STATE_BLOCKED` to submit a recovery action because the downstream safety supervisor evaluates the actual reverse/rotation corridor. Do not override a downstream block.

- [ ] **Step 3: Confirm tests fail**

```bash
PYTHONPATH=src/track_robot/track_robot_navigation \
pytest -q \
  src/track_robot/track_robot_navigation/test/test_physical_recovery_supervisor.py \
  src/track_robot/track_robot_navigation/test/test_semantic_authorization.py \
  src/track_robot/track_robot_navigation/test/test_static_target_mission.py
```

Expected: FAIL because the clients and transitions are absent.

- [ ] **Step 4: Add parameters, clients and goal construction**

Validate:

```python
0.0 < recovery_spin_angle_rad <= math.pi / 2.0
0.20 <= recovery_backup_distance_m <= 0.30
0.10 <= recovery_backup_speed_mps <= 0.15
1 <= maximum_physical_recovery_cycles <= 5
```

Create only Nav2 action clients:

```python
self._spin_client = ActionClient(self, Spin, 'spin')
self._back_up_client = ActionClient(self, BackUp, 'back_up')
```

Construct goals:

```python
spin = Spin.Goal()
spin.target_yaw = (
    self._physical_recovery.spin_sign * self._recovery_spin_angle_rad)

backup = BackUp.Goal()
backup.target.x = -self._recovery_backup_distance_m
backup.target.y = 0.0
backup.target.z = 0.0
backup.speed = self._recovery_backup_speed_mps
```

Do not create a Twist publisher.

- [ ] **Step 5: Route navigation and recovery results**

When enabled, a terminal Navigate abort calls:

```python
self._physical_recovery.navigation_aborted(time.monotonic())
self._policy.mark_dispatch_failed()
```

Spin/BackUp terminal results call:

```python
self._physical_recovery.recovery_finished(
    action,
    int(wrapped.status) == GoalStatus.STATUS_SUCCEEDED,
    time.monotonic(),
)
```

The supervision timer evaluates hard preflight first: authorization, RC,
E-stop, base health, cloud freshness and odometry freshness.  When a recovery
command is already pending, dispatch it before the ordinary
`StaticTargetMissionPolicy` obstacle-Hold branch; otherwise a robot currently
inside `STATE_BLOCKED` could never submit a reverse/rotation command for the
downstream safety supervisor to evaluate.  This ordering does not override a
downstream collision block.  When the feature is false, preserve the current
`_classify_nav2_result` retry behavior exactly.

Call `reset()` only when locking a new mission, completing it, or explicitly clearing it. Never reset on Nav2 abort, costmap clear, Spin, BackUp, transient target loss or cooldown.

- [ ] **Step 6: Make cancellation apply to every motion action**

RC override, E-stop and base fault immediately cancel Navigate/Spin/BackUp, clear the mission and request disarm. Stale odom/cloud cancels motion but preserves the frozen mission. Extend `destroy_node()` to destroy both new action clients before the node handle.

- [ ] **Step 7: Run tests and commit**

```bash
PYTHONPATH=src/track_robot/track_robot_navigation \
pytest -q \
  src/track_robot/track_robot_navigation/test/test_physical_recovery_supervisor.py \
  src/track_robot/track_robot_navigation/test/test_semantic_authorization.py \
  src/track_robot/track_robot_navigation/test/test_static_target_mission.py
git add \
  src/track_robot/track_robot_navigation/track_robot_navigation/semantic_navigation_supervisor_node.py \
  src/track_robot/track_robot_navigation/test \
  src/track_robot/track_robot_navigation/CMakeLists.txt
git commit -m "feat(navigation): recover while preserving semantic mission"
```

Expected: focused tests PASS and target/reference assertions remain unchanged.

---

### Task 4: Propagate the Opt-In Flag Through Standard Commands

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase4b.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/launch/semantic_search_phase5a.launch.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/track_robot_bringup/control_cli.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_bringup/test/test_control_cli.py`
- Modify: Phase 4B/5A launch and no-motion contract tests.

**Interfaces:**
- Produces `semantic_search_ctl run phase4b --physical-recovery` and `run phase5a --rotation-supervised --physical-recovery`.
- Default command explicitly passes false.

- [ ] **Step 1: Write failing CLI/launch tests**

```python
assert 'physical_recovery_enabled:=false' in default_command
assert 'physical_recovery_enabled:=true' in opted_in_command
```

Both bringup launch files must declare the flag default false and forward it to `phase4b_navigation.launch.py`.

- [ ] **Step 2: Confirm failure**

```bash
PYTHONPATH=src/track_robot/track_robot_bringup \
pytest -q \
  src/track_robot/track_robot_bringup/test/test_control_cli.py \
  src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py \
  src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py \
  src/track_robot/track_robot_bringup/test/test_phase5a_no_motion_contract.py
```

Expected: FAIL because the option is absent.

- [ ] **Step 3: Add the explicit option and pass-through**

```python
run.add_argument(
    '--physical-recovery',
    action='store_true',
    help='enable bounded Nav2 Spin/BackUp recovery for an authorized mission',
)
```

Both command builders append:

```python
'physical_recovery_enabled:={}'.format(
    str(bool(args.physical_recovery)).lower()),
```

Do not implicitly enable it in rotation-supervised mode.

- [ ] **Step 4: Preserve no-motion modes, run tests and commit**

Add assertions that Planning-only and Semantic-shadow omit controller, BT navigator, recoveries server and safety velocity chain regardless of the flag.

```bash
PYTHONPATH=src/track_robot/track_robot_bringup \
pytest -q \
  src/track_robot/track_robot_bringup/test/test_control_cli.py \
  src/track_robot/track_robot_bringup/test/test_phase4b_launch_contract.py \
  src/track_robot/track_robot_bringup/test/test_phase5a_launch_contract.py \
  src/track_robot/track_robot_bringup/test/test_phase5a_no_motion_contract.py
git add src/track_robot/track_robot_bringup
git commit -m "feat(bringup): expose opt-in physical recovery"
```

Expected: all bringup tests PASS.

---

### Task 5: Publish Recovery Evidence and Show It in RViz

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/track_robot_navigation/semantic_navigation_supervisor_node.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/test/test_physical_recovery_supervisor.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_semantic_search_rviz_plugins/package.xml`
- Modify: `track_robot_ws/src/track_robot/track_robot_semantic_search_rviz_plugins/include/track_robot_semantic_search_rviz_plugins/semantic_search_panel.hpp`
- Modify: `track_robot_ws/src/track_robot/track_robot_semantic_search_rviz_plugins/src/semantic_search_panel.cpp`
- Modify: `track_robot_ws/src/track_robot/track_robot_semantic_search_rviz_plugins/test/test_plugin_contract.py`

**Interfaces:**
- Consumes `/semantic_navigation/diagnostics` as `diagnostic_msgs/msg/DiagnosticArray`.
- Produces one read-only “Navigation recovery” label; no service or motion button.

- [ ] **Step 1: Write failing diagnostic contracts**

Require these keys:

```text
recovery_stage
recovery_cycle
recovery_attempt
recovery_last_failure
mission_target_global_id
mission_target_anchor_x
mission_target_anchor_y
mission_authorization_preserved
backup_permitted
```

Require the RViz panel to subscribe to `/semantic_navigation/diagnostics` and own `recovery_status_`.

- [ ] **Step 2: Add supervisor evidence**

Populate values only from the pure recovery policy and frozen mission fields. When no mission exists, publish empty/zero values; never substitute the current live candidate.

- [ ] **Step 3: Add read-only RViz rendering**

Add `diagnostic_msgs` dependency, select `semantic_navigation/supervisor`, convert key/value pairs to a map and render:

```text
<stage>; cycle <n>; <last failure>; target <global id>
```

Add:

```cpp
form->addRow(tr("Navigation recovery"), recovery_status_);
```

Do not change query, Start Finding, Start Approach, or Cancel & Disarm behavior.

- [ ] **Step 4: Build, test and commit**

```bash
cd /home/track-robot/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select \
  track_robot_navigation track_robot_semantic_search_rviz_plugins \
  --symlink-install
source install/setup.bash
colcon test --packages-select \
  track_robot_navigation track_robot_semantic_search_rviz_plugins \
  --event-handlers console_direct+
colcon test-result --verbose
git add \
  src/track_robot/track_robot_navigation \
  src/track_robot/track_robot_semantic_search_rviz_plugins
git commit -m "feat(rviz): show semantic recovery state"
```

Expected: zero failures.

---

### Task 6: Document and Execute Regression-Gated Commissioning

**Files:**
- Modify: `track_robot_ws/docs/guides/semantic-search/phase4b-nav2-supervised-test.md`
- Modify: `track_robot_ws/docs/guides/semantic-search/phase5a-bounded-active-search-test.md`
- Create: `track_robot_ws/artifacts/semantic-search/phase4b-physical-recovery-2026-08-10/README.md`

**Interfaces:**
- Produces exact commands, physical layout, expected evidence, shutdown and rollback steps.

- [ ] **Step 1: Document default and opt-in commands**

Default regression:

```bash
cd ~/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export TRACK_ROBOT_WS=~/track_robot_ws
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/track_robot_ws/src/track_robot/track_robot_bringup/config/fastdds_semantic_search.xml
ros2 run track_robot_bringup semantic_search_ctl run phase5a --rotation-supervised
```

Opt-in test:

```bash
ros2 run track_robot_bringup semantic_search_ctl run phase5a \
  --rotation-supervised --physical-recovery
```

- [ ] **Step 2: Run the full automated regression**

```bash
cd /home/track-robot/track_robot_ws/.worktrees/main-integration/track_robot_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select \
  track_robot_navigation track_robot_bringup \
  track_robot_semantic_search_rviz_plugins --symlink-install
source install/setup.bash
colcon test --packages-select \
  track_robot_navigation track_robot_bringup \
  track_robot_semantic_search_rviz_plugins \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failures and unchanged public topics/services.

- [ ] **Step 3: Execute staged live tests with an operator present**

Run in order and stop at the first unexpected motion:

1. Flag off: green-bottle approach equals baseline.
2. Flag on: clear/replan retains target reference and odom anchor.
3. At least `1.0 m` observed rotation/rear free space: one `30 deg` Spin.
4. At least `1.0 m` observed rear free space: one `0.25 m @ 0.10 m/s` BackUp.
5. Visible rear obstacle: BackUp rejected and final `/cmd_vel` remains zero.
6. RC override during Spin and BackUp: immediate cancel.
7. E-stop during Spin and BackUp: immediate cancel and mission termination.
8. Three recovery runs on one static target: no new Start Approach click; domain/ID/anchor remain stable.

Capture:

```bash
ros2 topic echo /semantic_navigation/diagnostics
ros2 topic echo /safety/state
ros2 action list -t
ros2 topic hz /odom
```

- [ ] **Step 4: Stop every run and commit the runbook**

```bash
ros2 run track_robot_bringup semantic_search_ctl stop
ros2 node list
git add \
  docs/guides/semantic-search \
  artifacts/semantic-search/phase4b-physical-recovery-2026-08-10/README.md
git commit -m "docs: add physical recovery commissioning runbook"
```

Expected: no managed Phase 1–5A, Nav2, ZED, RoboSense or Bunker nodes remain after shutdown.

---

## Final Regression and Acceptance Gate

- [ ] Affected builds and tests report zero failures.
- [ ] Default launch keeps physical recovery disabled.
- [ ] Planning-only and shadow modes cannot execute recovery motion.
- [ ] Spin/BackUp output passes through the existing safety supervisor and velocity gate.
- [ ] No recovery source publishes directly to `/cmd_vel`.
- [ ] Stale odom/cloud/base state and RC/E-stop/base fault block physical recovery.
- [ ] The Nav2 footprint blocks BackUp when a rear obstacle is present.
- [ ] RC override and E-stop cancel active recovery immediately.
- [ ] Target domain/global ID, frozen odom goal and operator authorization survive Clear, Spin, BackUp and cooldown.
- [ ] Mission completion and explicit cancel reset recovery state.
- [ ] All live nodes are stopped before delivery.

If a gate fails, revert only the responsible task commit. Keeping `physical_recovery_enabled:=false` must reproduce the current tested Phase 4B/5A behavior while the defect is corrected.
