import track_robot_navigation.semantic_navigation_supervisor_node as supervisor
from action_msgs.msg import GoalStatus
from types import SimpleNamespace

from track_robot_navigation.semantic_navigation_supervisor_node import (
    _authorization_reference_is_current,
    _authorization_survives_interruption,
)


REFERENCE = (11, 22, 33, 44, 1)


def test_same_target_allows_a_lagging_rviz_snapshot_sequence():
    assert _authorization_reference_is_current(
        REFERENCE, 10582, REFERENCE, 10581)


def test_authorization_rejects_wrong_identity_future_or_zero_sequence():
    assert not _authorization_reference_is_current(
        REFERENCE, 10582, (11, 23, 33, 44, 1), 10581)
    assert not _authorization_reference_is_current(
        REFERENCE, 10582, REFERENCE, 10583)
    assert not _authorization_reference_is_current(
        REFERENCE, 10582, REFERENCE, 0)


def test_authorization_survives_transient_target_and_safety_interruptions():
    for reason in (
            'target_stale',
            'target_position_invalid',
            'target_reference_mismatch',
            'waiting_for_correlated_inputs',
            'planner_not_ready',
            'safety_motion_not_permitted'):
        assert _authorization_survives_interruption(reason)


def test_authorization_does_not_survive_explicit_or_identity_stop():
    for reason in (
            'operator_cancel',
            'target_reference_changed',
            'safety_hard_stop'):
        assert not _authorization_survives_interruption(reason)


def test_static_target_reacquires_changed_global_id_at_same_odom_position():
    matcher = getattr(supervisor, '_same_static_target_location', None)
    assert matcher is not None
    original = (11, 22, 33, 44, 1)
    changed_id = (11, 23, 33, 44, 1)

    assert matcher(original, changed_id, (2.30, -0.10), (2.43, -0.16), 0.45)


def test_static_target_reacquisition_rejects_far_or_different_query_target():
    matcher = getattr(supervisor, '_same_static_target_location', None)
    assert matcher is not None
    original = (11, 22, 33, 44, 1)

    assert not matcher(original, (11, 23, 33, 44, 1),
                       (2.30, -0.10), (3.0, -0.10), 0.45)
    assert not matcher(original, (11, 23, 33, 45, 1),
                       (2.30, -0.10), (2.31, -0.10), 0.45)


def test_target_dropout_grace_is_bounded_and_target_only():
    grace_type = getattr(supervisor, '_TransientTargetGrace', None)
    assert grace_type is not None
    grace = grace_type(1.0)

    assert grace.should_hold('waiting_for_correlated_inputs', 10.0)
    assert grace.should_hold('target_position_invalid', 10.8)
    assert not grace.should_hold('target_position_invalid', 11.01)
    grace.reset()
    assert not grace.should_hold('odometry_stale', 20.0)


def test_nav2_abort_preserves_authorization_while_retry_budget_remains():
    classify = getattr(supervisor, '_classify_nav2_result', None)
    assert classify is not None

    assert classify(GoalStatus.STATUS_ABORTED, 0, 2) == 'retry'
    assert classify(GoalStatus.STATUS_ABORTED, 1, 2) == 'retry'


def test_nav2_terminal_result_only_retries_bounded_aborts():
    classify = getattr(supervisor, '_classify_nav2_result', None)
    assert classify is not None

    assert classify(GoalStatus.STATUS_SUCCEEDED, 0, 2) == 'complete'
    assert classify(GoalStatus.STATUS_ABORTED, 2, 2) == 'stop'
    assert classify(GoalStatus.STATUS_CANCELED, 0, 2) == 'stop'
    assert classify(GoalStatus.STATUS_UNKNOWN, 0, 2) == 'stop'


class _ResultPolicy:
    def __init__(self):
        self.dispatch_failures = 0

    def mark_dispatch_failed(self):
        self.dispatch_failures += 1


class _ResultLogger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class _ResultHarness:
    def __init__(self, retry_count=0, maximum_retries=2):
        self._pending_goal_kind = supervisor.GoalAction.NAVIGATE
        self._active_goal_handle = object()
        self._cancel_preserves_authorization = False
        self._authorized_reference = REFERENCE
        self._nav2_retry_count = retry_count
        self._maximum_nav2_retries = maximum_retries
        self._nav2_retry_cooldown_sec = 2.0
        self._nav2_retry_not_before_s = 0.0
        self._preserve_authorization_after_retry_exhaustion = True
        self._policy = _ResultPolicy()
        self.cleared_reason = None
        self.disarm_requests = 0

    def get_logger(self):
        return _ResultLogger()

    def _clear_authorization(self, reason):
        self.cleared_reason = reason
        self._authorized_reference = None

    def _request_safety_disarm(self):
        self.disarm_requests += 1


class _ResultFuture:
    def __init__(self, status):
        self._status = status

    def result(self):
        return SimpleNamespace(status=self._status, result=None)


def test_nav2_abort_retries_without_clearing_operator_authorization():
    harness = _ResultHarness()

    supervisor.SemanticNavigationSupervisorNode._on_action_result(
        harness, _ResultFuture(GoalStatus.STATUS_ABORTED))

    assert harness._nav2_retry_count == 1
    assert harness._nav2_retry_not_before_s > 0.0
    assert harness._policy.dispatch_failures == 1
    assert harness.cleared_reason is None
    assert harness.disarm_requests == 0


def test_nav2_success_ends_mission_but_exhausted_abort_enters_cooldown():
    succeeded = _ResultHarness(retry_count=1)
    supervisor.SemanticNavigationSupervisorNode._on_action_result(
        succeeded, _ResultFuture(GoalStatus.STATUS_SUCCEEDED))

    assert succeeded._nav2_retry_count == 0
    assert succeeded.cleared_reason == 'nav2_action_succeeded'
    assert succeeded.disarm_requests == 1

    exhausted = _ResultHarness(retry_count=2)
    supervisor.SemanticNavigationSupervisorNode._on_action_result(
        exhausted, _ResultFuture(GoalStatus.STATUS_ABORTED))

    assert exhausted._nav2_retry_count == 0
    assert exhausted._nav2_retry_not_before_s > 0.0
    assert exhausted.cleared_reason is None
    assert exhausted.disarm_requests == 0
