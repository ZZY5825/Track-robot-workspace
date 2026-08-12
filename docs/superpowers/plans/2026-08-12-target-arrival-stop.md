# Target Arrival Stop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop and latch an authorized semantic approach when the robot reference center remains strictly closer than `0.70 m` to the frozen static target for three supervision cycles.

**Architecture:** Add a small deterministic arrival-confirmation policy beside the existing static mission policy. `semantic_navigation_supervisor` evaluates frozen target `odom` anchor against fresh robot odometry before recovery/navigation dispatch, cancels Nav2 and disarms through the existing safety chain on arrival, then holds the same mission in `target_reached` until an explicit mission reset.

**Tech Stack:** ROS 2 Foxy, Python 3, rclpy, Nav2 `NavigateToPose`, pytest/ament.

## Global Constraints

- Follow `docs/superpowers/specs/2026-08-12-target-arrival-stop-design.md`.
- The distance condition is strictly `distance < 0.70 m`, measured in the navigation frame between the robot reference center and frozen target anchor.
- Require three consecutive valid supervision cycles; one invalid or out-of-range sample resets the count.
- Do not change Phase 1–3 perception, global ID ownership, target localization, URDF/TF, Nav2 plugins, footprint, costmaps, or controller tuning.
- Preserve public topics, messages, services, actions and the final velocity chain.
- Do not publish `/cmd_vel`; stop through Nav2 cancellation and the existing safety disarm service.
- Preserve the locked target reference and anchor after arrival; do not start recovery or redispatch navigation until explicit mission reset.
- Do not stage or commit unrelated dirty-worktree files.

---

### Task 1: Add Target-Relative Arrival Termination

**Files:**
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/track_robot_navigation/semantic_navigation_supervisor_node.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/config/semantic_navigation.yaml`
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/test/test_static_target_mission.py`
- Modify: `track_robot_ws/src/track_robot/track_robot_navigation/test/test_launch_contract.py`
- Modify: `track_robot_ws/docs/guides/semantic-search/phase4b-nav2-supervised-test.md`

**Interfaces:**
- Consumes: frozen target anchor `self._authorized_target_anchor_xy`, `/odom`, `self._maximum_odom_age_sec`, current supervised Nav2 action and existing safety disarm client.
- Produces: private `_TargetArrivalConfirmation.observe(robot_xy, target_xy) -> bool`, node state `self._mission_arrived`, parameters `target_arrival_distance_m` and `target_arrival_confirmation_cycles`, diagnostic reason `target_reached` plus `target_distance_m`.

- [ ] **Step 1: Write failing pure-policy tests**

Add tests to `test_static_target_mission.py` that instantiate the private confirmation helper and prove strict threshold, consecutive confirmation, and reset behavior:

```python
def test_target_arrival_requires_three_strictly_inside_samples():
    policy = supervisor._TargetArrivalConfirmation(0.70, 3)

    assert policy.observe((0.0, 0.0), (0.70, 0.0)) is False
    assert policy.observe((0.0, 0.0), (0.69, 0.0)) is False
    assert policy.observe((0.0, 0.0), (0.69, 0.0)) is False
    assert policy.observe((0.0, 0.0), (0.69, 0.0)) is True
    assert policy.last_distance_m == pytest.approx(0.69)


def test_target_arrival_resets_on_outside_or_invalid_sample():
    policy = supervisor._TargetArrivalConfirmation(0.70, 3)

    assert policy.observe((0.0, 0.0), (0.60, 0.0)) is False
    assert policy.observe((0.0, 0.0), None) is False
    assert policy.observe((0.0, 0.0), (0.60, 0.0)) is False
    assert policy.observe((0.0, 0.0), (0.80, 0.0)) is False
    assert policy.observe((0.0, 0.0), (0.60, 0.0)) is False
```

- [ ] **Step 2: Run the tests and verify the expected failure**

```bash
cd /home/track-robot/track_robot_ws/.worktrees/main-test/track_robot_ws
source /opt/ros/foxy/setup.bash
PYTHONPATH=src/track_robot/track_robot_navigation \
pytest -q src/track_robot/track_robot_navigation/test/test_static_target_mission.py \
  -k target_arrival
```

Expected: FAIL because `_TargetArrivalConfirmation` does not exist.

- [ ] **Step 3: Implement the pure confirmation helper**

Add the helper near the existing pure supervisor helpers:

```python
class _TargetArrivalConfirmation:
    def __init__(self, distance_m, confirmation_cycles):
        self.distance_m = float(distance_m)
        self.confirmation_cycles = int(confirmation_cycles)
        if not math.isfinite(self.distance_m) or not 0.0 < self.distance_m <= 2.0:
            raise ValueError('target arrival distance must be in (0, 2.0]')
        if not 1 <= self.confirmation_cycles <= 20:
            raise ValueError('target arrival confirmation cycles must be in [1, 20]')
        self.reset()

    def reset(self):
        self._inside_count = 0
        self.last_distance_m = float('nan')

    def observe(self, robot_xy, target_xy):
        if robot_xy is None or target_xy is None:
            self.reset()
            return False
        values = tuple(robot_xy) + tuple(target_xy)
        if len(values) != 4 or not all(math.isfinite(float(v)) for v in values):
            self.reset()
            return False
        self.last_distance_m = math.hypot(
            float(target_xy[0]) - float(robot_xy[0]),
            float(target_xy[1]) - float(robot_xy[1]),
        )
        if self.last_distance_m >= self.distance_m:
            self._inside_count = 0
            return False
        self._inside_count += 1
        return self._inside_count >= self.confirmation_cycles
```

- [ ] **Step 4: Run the focused pure-policy tests**

Run the Step 2 command again.

Expected: `2 passed`.

- [ ] **Step 5: Write failing supervisor behavior tests**

Extend the existing `_MissionHarness` constructor with arrival policy, frozen anchor, fresh robot XY, and mission-arrived state:

```python
def __init__(
        self, mission_snapshot, robot_xy=(2.0, 0.0),
        target_xy=(0.0, 0.0)):
    self._authorized_reference = (11, 22, 33, 44, 1)
    self._mission_policy = StaticTargetMissionPolicy(0.25)
    self._snapshot_value = mission_snapshot
    self.dispatched = []
    self.cancelled = []
    self.cleared = []
    self.disarmed = 0
    self.diagnostics = []
    self._mission_arrived = False
    self._arrival_confirmation = supervisor._TargetArrivalConfirmation(
        0.70, 3)
    self._authorized_target_anchor_xy = target_xy
    self._robot_xy = robot_xy

def _robot_xy_in_navigation_frame(self):
    return self._robot_xy
```

Then add tests proving:

```python
def test_supervisor_latches_arrival_and_stops_once_after_confirmation():
    harness = _MissionHarness(
        snapshot(), robot_xy=(0.0, 0.0), target_xy=(0.60, 0.0))

    for _ in range(3):
        supervisor.SemanticNavigationSupervisorNode._supervise_static_mission(harness)

    assert harness._mission_arrived is True
    assert harness.cancelled == ['target_reached']
    assert harness.disarmed == 1
    assert harness._authorized_reference == (11, 22, 33, 44, 1)
    assert harness._authorized_target_anchor_xy == (0.60, 0.0)
    assert harness.dispatched == [GoalAction.NAVIGATE, GoalAction.NAVIGATE]


def test_arrived_mission_holds_without_navigation_or_recovery():
    harness = _MissionHarness(
        snapshot(), robot_xy=(0.0, 0.0), target_xy=(0.60, 0.0))
    harness._mission_arrived = True

    supervisor.SemanticNavigationSupervisorNode._supervise_static_mission(harness)

    assert harness.dispatched == []
    assert harness.diagnostics[-1][1] == 'target_reached'
```

Also assert `_lock_static_mission()` and `_clear_authorization()` reset the confirmation counter and `self._mission_arrived`.

- [ ] **Step 6: Run the supervisor tests and verify the expected failure**

```bash
PYTHONPATH=src/track_robot/track_robot_navigation \
pytest -q src/track_robot/track_robot_navigation/test/test_static_target_mission.py
```

Expected: new arrival tests FAIL while pre-existing tests remain green.

- [ ] **Step 7: Integrate arrival state before recovery/navigation dispatch**

In node initialization, declare and validate parameters, create `_TargetArrivalConfirmation`, and initialize `_mission_arrived = False`. Add a private robot-position reader that accepts only fresh odometry in `self._navigation_frame`:

```python
def _robot_xy_in_navigation_frame(self):
    if self._odom is None or str(self._odom.header.frame_id) != self._navigation_frame:
        return None
    if self._age_from_stamp(self._odom.header.stamp, self._odom_received_s) > self._maximum_odom_age_sec:
        return None
    values = (
        float(self._odom.pose.pose.position.x),
        float(self._odom.pose.pose.position.y),
    )
    return values if all(math.isfinite(value) for value in values) else None
```

At the beginning of `_supervise_static_mission()`, before mission-policy and physical-recovery evaluation:

```python
if self._mission_arrived:
    self._publish_diagnostics(
        GoalAction.HOLD, 'target_reached', mission_snapshot.key)
    return True
if self._arrival_confirmation.observe(
        self._robot_xy_in_navigation_frame(),
        self._authorized_target_anchor_xy):
    self._mission_arrived = True
    self._publish_diagnostics(
        GoalAction.HOLD, 'target_reached', mission_snapshot.key)
    self._cancel_action('target_reached')
    self._request_safety_disarm()
    return True
```

Add `target_reached` to authorization-preserving interruption reasons so the canceled Nav2 result cannot erase the frozen mission. Reset arrival state and policy only in mission lock/clear paths. Extend diagnostics with `target_distance_m`, emitting an empty value when unavailable.

- [ ] **Step 8: Configure defaults and launch-contract assertions**

Add to `config/semantic_navigation.yaml`:

```yaml
    # Robot reference-center distance to the frozen static target. Confirmed
    # for three 10 Hz supervision cycles before Nav2 is cancelled and held.
    target_arrival_distance_m: 0.70
    target_arrival_confirmation_cycles: 3
```

Extend `test_launch_contract.py` to require exact values `0.70` and `3` without changing launch arguments or public interfaces.

- [ ] **Step 9: Run focused and package regression tests**

```bash
PYTHONPATH=src/track_robot/track_robot_navigation \
pytest -q \
  src/track_robot/track_robot_navigation/test/test_static_target_mission.py \
  src/track_robot/track_robot_navigation/test/test_launch_contract.py

colcon build --packages-select \
  track_robot_navigation track_robot_bringup --symlink-install
source install/setup.bash
colcon test --packages-select \
  track_robot_navigation track_robot_bringup \
  --event-handlers console_direct+
colcon test-result --verbose
```

Expected: zero failures. If the workspace has a pre-existing unrelated failure, record it separately and require all arrival-focused tests to pass.

- [ ] **Step 10: Update the operator test guide**

Document in `phase4b-nav2-supervised-test.md`:

- `target_reached` means robot-center distance is strictly below `0.70 m` for three cycles;
- the robot cancels Nav2 and disarms without deleting semantic target memory;
- no automatic recovery/replan occurs after arrival;
- reset with existing Cancel/Disarm or a new query before another approach;
- live acceptance procedure and diagnostic command for `/semantic_navigation/diagnostics`.

- [ ] **Step 11: Run final diff and regression gate**

```bash
git diff --check
git diff -- \
  track_robot_ws/src/track_robot/track_robot_navigation \
  track_robot_ws/docs/guides/semantic-search/phase4b-nav2-supervised-test.md
```

Expected: only arrival-stop implementation, tests, config and guide changes; no perception, TF, URDF, planner or controller diff.

- [ ] **Step 12: Commit only the scoped implementation**

```bash
git add \
  track_robot_ws/src/track_robot/track_robot_navigation/track_robot_navigation/semantic_navigation_supervisor_node.py \
  track_robot_ws/src/track_robot/track_robot_navigation/config/semantic_navigation.yaml \
  track_robot_ws/src/track_robot/track_robot_navigation/test/test_static_target_mission.py \
  track_robot_ws/src/track_robot/track_robot_navigation/test/test_launch_contract.py \
  track_robot_ws/docs/guides/semantic-search/phase4b-nav2-supervised-test.md
git commit -m "feat(navigation): stop at target arrival distance"
```

Expected: one implementation commit; unrelated dirty files remain unstaged and unchanged.
