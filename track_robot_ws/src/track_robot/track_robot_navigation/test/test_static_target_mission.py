import math
from types import SimpleNamespace

import pytest

import track_robot_navigation.semantic_navigation_supervisor_node as supervisor
from track_robot_navigation.semantic_goal_policy import GoalAction
from track_robot_navigation.static_target_mission import (
    StaticMissionSnapshot,
    StaticTargetMissionPolicy,
    static_mission_reference_failure,
)
from track_robot_interfaces.msg import SafetyState


def snapshot(**overrides):
    values = {
        'memory_epoch_id': 11,
        'global_object_id': 22,
        'localization_epoch_id': 33,
        'observed_localization_epoch_id': 33,
        'odom_age_sec': 0.05,
        'safety_armed': True,
        'safety_permits_motion': True,
        'safety_temporarily_blocked': False,
        'goal_in_flight': False,
    }
    values.update(overrides)
    return StaticMissionSnapshot(**values)


def test_locked_static_mission_navigates_without_live_perception_input():
    policy = StaticTargetMissionPolicy(maximum_odom_age_sec=0.25)

    decision = policy.evaluate(snapshot())

    assert decision.action is GoalAction.NAVIGATE
    assert decision.reason == 'static_mission_ready'
    assert decision.terminate_mission is False


def test_active_goal_and_obstacle_block_hold_the_same_mission():
    policy = StaticTargetMissionPolicy(maximum_odom_age_sec=0.25)

    active = policy.evaluate(snapshot(goal_in_flight=True))
    blocked = policy.evaluate(snapshot(safety_temporarily_blocked=True))

    assert active.action is GoalAction.HOLD
    assert active.reason == 'static_mission_goal_active'
    assert blocked.action is GoalAction.HOLD
    assert blocked.reason == 'static_mission_obstacle_blocked'


def test_transient_odom_or_safety_loss_cancels_motion_but_keeps_mission():
    policy = StaticTargetMissionPolicy(maximum_odom_age_sec=0.25)

    stale = policy.evaluate(snapshot(odom_age_sec=0.30))
    unavailable = policy.evaluate(snapshot(safety_permits_motion=False))

    assert stale.action is GoalAction.CANCEL
    assert stale.reason == 'odometry_stale'
    assert stale.terminate_mission is False
    assert unavailable.action is GoalAction.CANCEL
    assert unavailable.reason == 'safety_motion_not_permitted'
    assert unavailable.terminate_mission is False


def test_localization_reset_or_disarm_terminates_static_mission():
    policy = StaticTargetMissionPolicy(maximum_odom_age_sec=0.25)

    reset = policy.evaluate(snapshot(observed_localization_epoch_id=34))
    disarmed = policy.evaluate(snapshot(safety_armed=False))

    assert reset.action is GoalAction.CANCEL
    assert reset.reason == 'localization_epoch_changed'
    assert reset.terminate_mission is True
    assert disarmed.action is GoalAction.CANCEL
    assert disarmed.reason == 'safety_not_armed'
    assert disarmed.terminate_mission is True


def test_static_mission_ignores_live_visibility_and_global_id_changes_only():
    locked = (11, 22, 33, 44, 1)

    assert static_mission_reference_failure(locked, None) is None
    assert static_mission_reference_failure(
        locked, (11, 99, 33, 44, 1)) is None
    assert static_mission_reference_failure(
        locked, (11, 99, 34, 44, 1)) == 'localization_epoch_changed'
    assert static_mission_reference_failure(
        locked, (11, 99, 33, 45, 1)) == 'query_changed'


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


class _MissionHarness:
    def __init__(
            self, mission_snapshot, robot_xy=(2.0, 0.0),
            target_xy=(0.0, 0.0)):
        self._authorized_reference = (11, 22, 33, 44, 1)
        self._mission_policy = StaticTargetMissionPolicy(0.25)
        self._snapshot_value = mission_snapshot
        self._mission_arrived = False
        self._arrival_confirmation = supervisor._TargetArrivalConfirmation(
            0.70, 3)
        self._authorized_target_anchor_xy = target_xy
        self._robot_xy = robot_xy
        self.dispatched = []
        self.cancelled = []
        self.cleared = []
        self.disarmed = 0
        self.diagnostics = []

    def _static_mission_snapshot(self):
        return self._snapshot_value

    def _robot_xy_in_navigation_frame(self):
        return self._robot_xy

    def _dispatch(self, action):
        self.dispatched.append(action)

    def _cancel_action(self, reason):
        self.cancelled.append(reason)

    def _clear_authorization(self, reason):
        self.cleared.append(reason)
        self._authorized_reference = None

    def _request_safety_disarm(self):
        self.disarmed += 1

    def _publish_diagnostics(self, action, reason, key):
        self.diagnostics.append((action, reason, key))


def test_supervisor_dispatches_locked_mission_without_live_target_snapshot():
    harness = _MissionHarness(snapshot())

    handled = supervisor.SemanticNavigationSupervisorNode._supervise_static_mission(
        harness)

    assert handled is True
    assert harness.dispatched == [GoalAction.NAVIGATE]
    assert harness.cancelled == []
    assert harness.cleared == []


def test_supervisor_preserves_mission_for_stale_odom_but_ends_on_epoch_reset():
    stale = _MissionHarness(snapshot(odom_age_sec=0.30))
    supervisor.SemanticNavigationSupervisorNode._supervise_static_mission(stale)

    assert stale.cancelled == ['odometry_stale']
    assert stale.cleared == []
    assert stale.disarmed == 0

    reset = _MissionHarness(snapshot(observed_localization_epoch_id=34))
    supervisor.SemanticNavigationSupervisorNode._supervise_static_mission(reset)

    assert reset.cleared == ['localization_epoch_changed']
    assert reset.cancelled == ['localization_epoch_changed']
    assert reset.disarmed == 1


def test_supervisor_latches_arrival_and_stops_once_after_confirmation():
    harness = _MissionHarness(
        snapshot(), robot_xy=(0.0, 0.0), target_xy=(0.60, 0.0))

    for _ in range(3):
        supervisor.SemanticNavigationSupervisorNode._supervise_static_mission(
            harness)

    assert harness._mission_arrived is True
    assert harness.cancelled == ['target_reached']
    assert harness.disarmed == 1
    assert harness._authorized_reference == (11, 22, 33, 44, 1)
    assert harness._authorized_target_anchor_xy == (0.60, 0.0)
    assert harness.dispatched == [GoalAction.NAVIGATE, GoalAction.NAVIGATE]


def test_arrived_mission_holds_without_navigation_or_recovery():
    harness = _MissionHarness(
        snapshot(safety_armed=False),
        robot_xy=(0.0, 0.0), target_xy=(0.60, 0.0))
    harness._mission_arrived = True

    supervisor.SemanticNavigationSupervisorNode._supervise_static_mission(
        harness)

    assert harness.dispatched == []
    assert harness.cancelled == []
    assert harness.diagnostics[-1][1] == 'target_reached'


def test_arrived_mission_still_terminates_on_localization_reset():
    harness = _MissionHarness(
        snapshot(observed_localization_epoch_id=34),
        robot_xy=(0.0, 0.0), target_xy=(0.60, 0.0))
    harness._mission_arrived = True

    supervisor.SemanticNavigationSupervisorNode._supervise_static_mission(
        harness)

    assert harness.cleared == ['localization_epoch_changed']
    assert harness.cancelled == ['localization_epoch_changed']
    assert harness.disarmed == 1


def test_locked_mission_goal_is_used_without_retransforming_live_goal():
    class FailingBuffer:
        def transform(self, *_args, **_kwargs):
            raise AssertionError('live goal must not be transformed after lock')

    harness = SimpleNamespace(
        _mission_goal='locked-odom-goal',
        _goal='new-live-goal',
        _tf_buffer=FailingBuffer(),
        _navigation_frame='odom',
        _transform_timeout_sec=0.1,
    )

    result = supervisor.SemanticNavigationSupervisorNode._goal_in_navigation_frame(
        harness)

    assert result == 'locked-odom-goal'


def test_supervisor_builds_mission_snapshot_without_live_target_or_goal():
    harness = SimpleNamespace(
        _authorized_reference=(11, 22, 33, 44, 1),
        _mission_goal='locked-odom-goal',
        _odom=SimpleNamespace(header=SimpleNamespace(stamp=object())),
        _odom_received_s=1.0,
        _localization_state=SimpleNamespace(localization_epoch_id=33),
        _safety=SimpleNamespace(
            armed=True,
            state=SafetyState.STATE_CLEAR,
        ),
        _pending_goal_kind=None,
        _active_goal_handle=None,
        _age_from_stamp=lambda _stamp, _received: 0.05,
    )

    result = supervisor.SemanticNavigationSupervisorNode._static_mission_snapshot(
        harness)

    assert result == snapshot()


class _TargetUpdateHarness:
    def __init__(self):
        self._target_array = None
        self._target = None
        self._mission_goal = 'locked-odom-goal'
        self._authorized_reference = (11, 22, 33, 44, 1)
        self._pending_authorization = None
        self._authorized_target_anchor_xy = (2.3, 0.0)
        self._pending_target_anchor_xy = None
        self._static_target_mode = False
        self._static_target_position_reacquisition_enabled = False
        self._static_target_reacquisition_radius_m = 0.45
        self.cleared = []
        self.cancelled = []
        self.disarmed = 0

    def _clear_authorization(self, reason):
        self.cleared.append(reason)
        self._authorized_reference = None

    def _cancel_action(self, reason):
        self.cancelled.append(reason)

    def _request_safety_disarm(self):
        self.disarmed += 1

    def _target_anchor_in_navigation_frame(self, _target):
        return (9.0, 9.0)


def target_array(global_id=99, localization_epoch=33, query_id=44):
    target = SimpleNamespace(
        global_object_id=global_id,
        localization_epoch_id=localization_epoch,
        active_query_id=query_id,
        active_query_version=1,
    )
    return SimpleNamespace(
        memory_epoch_id=11,
        snapshot_sequence=2,
        objects=[target],
    )


def test_live_candidate_change_does_not_replace_locked_static_mission():
    harness = _TargetUpdateHarness()

    supervisor.SemanticNavigationSupervisorNode._on_target(
        harness, target_array(global_id=99))

    assert harness.cleared == []
    assert harness.cancelled == []


def test_query_or_localization_change_terminates_locked_static_mission():
    query = _TargetUpdateHarness()
    supervisor.SemanticNavigationSupervisorNode._on_target(
        query, target_array(query_id=45))
    assert query.cleared == ['query_changed']
    assert query.cancelled == ['query_changed']
    assert query.disarmed == 1

    localization = _TargetUpdateHarness()
    supervisor.SemanticNavigationSupervisorNode._on_target(
        localization, target_array(localization_epoch=34))
    assert localization.cleared == ['localization_epoch_changed']


def test_lock_static_mission_freezes_reference_anchor_and_odom_goal():
    arrival = supervisor._TargetArrivalConfirmation(0.70, 3)
    arrival.observe((0.0, 0.0), (0.60, 0.0))
    harness = SimpleNamespace(
        _nav2_retry_count=1,
        _authorized_reference=None,
        _pending_authorization=('old', 1),
        _authorized_target_anchor_xy=None,
        _pending_target_anchor_xy=(9.0, 9.0),
        _mission_goal=None,
        _pending_mission_goal='old-goal',
        _mission_arrived=True,
        _arrival_confirmation=arrival,
    )

    supervisor.SemanticNavigationSupervisorNode._lock_static_mission(
        harness,
        (11, 22, 33, 44, 1),
        (2.3, 0.0),
        'odom-goal',
    )

    assert harness._nav2_retry_count == 0
    assert harness._authorized_reference == (11, 22, 33, 44, 1)
    assert harness._authorized_target_anchor_xy == (2.3, 0.0)
    assert harness._mission_goal == 'odom-goal'
    assert harness._pending_authorization is None
    assert harness._pending_target_anchor_xy is None
    assert harness._pending_mission_goal is None
    assert harness._mission_arrived is False
    assert math.isnan(harness._arrival_confirmation.last_distance_m)


def test_clear_authorization_resets_arrival_latch_and_counter():
    class Resettable:
        def __init__(self):
            self.calls = 0

        def reset(self):
            self.calls += 1

    arrival = supervisor._TargetArrivalConfirmation(0.70, 3)
    arrival.observe((0.0, 0.0), (0.60, 0.0))
    grace = Resettable()
    recovery = Resettable()
    harness = SimpleNamespace(
        _authorized_reference=(11, 22, 33, 44, 1),
        _pending_authorization=None,
        _authorized_target_anchor_xy=(0.60, 0.0),
        _pending_target_anchor_xy=None,
        _mission_goal='odom-goal',
        _pending_mission_goal=None,
        _mission_arrived=True,
        _arrival_confirmation=arrival,
        _nav2_retry_count=1,
        _nav2_retry_not_before_s=2.0,
        _target_grace=grace,
        _physical_recovery=recovery,
        get_logger=lambda: SimpleNamespace(warn=lambda _message: None),
    )

    supervisor.SemanticNavigationSupervisorNode._clear_authorization(
        harness, 'operator_cancel')

    assert harness._mission_arrived is False
    assert math.isnan(harness._arrival_confirmation.last_distance_m)
    assert harness._authorized_reference is None
    assert harness._authorized_target_anchor_xy is None
    assert grace.calls == 1
    assert recovery.calls == 1


def test_supervision_uses_locked_mission_before_live_semantic_snapshot():
    class Harness:
        def __init__(self):
            self.mission_calls = 0

        def _supervise_static_mission(self):
            self.mission_calls += 1
            return True

        def _snapshot(self):
            raise AssertionError('live semantic snapshot must not gate mission')

    harness = Harness()

    supervisor.SemanticNavigationSupervisorNode._supervise(harness)

    assert harness.mission_calls == 1
