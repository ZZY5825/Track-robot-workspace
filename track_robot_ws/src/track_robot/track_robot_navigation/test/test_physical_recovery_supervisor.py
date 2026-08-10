from types import SimpleNamespace

from action_msgs.msg import GoalStatus
import pytest

import track_robot_navigation.semantic_navigation_supervisor_node as supervisor
from track_robot_interfaces.msg import SafetyState
from track_robot_navigation.physical_recovery import (
    PhysicalRecoveryPolicy,
    RecoveryCommand,
    RecoveryStage,
)
from track_robot_navigation.semantic_goal_policy import GoalAction
from track_robot_navigation.static_target_mission import (
    StaticMissionSnapshot,
    StaticTargetMissionPolicy,
)


REFERENCE = (11, 22, 33, 44, 1)
ANCHOR = (2.30, 0.0)
GOAL = 'frozen-odom-goal'


def clear_safety(**overrides):
    values = {
        'armed': True,
        'state': SafetyState.STATE_CLEAR,
        'cloud_fresh': True,
        'base_status_fresh': True,
        'base_status_ok': True,
        'rc_override_active': False,
        'emergency_stop_latched': False,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_physical_recovery_preflight_is_fail_closed_but_allows_blocked_state():
    preflight = supervisor._physical_recovery_preflight_failure

    assert preflight(clear_safety(), 0.05) is None
    assert preflight(
        clear_safety(state=SafetyState.STATE_BLOCKED), 0.05) is None
    assert preflight(clear_safety(cloud_fresh=False), 0.05) == (
        'recovery_cloud_stale')
    assert preflight(clear_safety(rc_override_active=True), 0.05) == (
        'recovery_rc_override')
    assert preflight(
        clear_safety(emergency_stop_latched=True), 0.05) == (
        'recovery_emergency_stop')
    assert preflight(clear_safety(base_status_ok=False), 0.05) == (
        'recovery_base_fault')
    assert preflight(clear_safety(base_status_fresh=False), 0.05) == (
        'recovery_base_status_stale')
    assert preflight(clear_safety(), 0.30) == 'recovery_odometry_stale'
    assert preflight(None, 0.05) == 'recovery_safety_unavailable'


class _ResultPolicy:
    def __init__(self):
        self.dispatch_failures = 0

    def mark_dispatch_failed(self):
        self.dispatch_failures += 1


class _Logger:
    def info(self, _message):
        pass

    def warn(self, _message):
        pass

    def error(self, _message):
        pass


class _ResultFuture:
    def __init__(self, status):
        self._status = status

    def result(self):
        return SimpleNamespace(status=self._status, result=None)


class _RecoveryResultHarness:
    def __init__(self):
        self._pending_goal_kind = GoalAction.NAVIGATE
        self._active_goal_handle = object()
        self._cancel_preserves_authorization = False
        self._authorized_reference = REFERENCE
        self._authorized_target_anchor_xy = ANCHOR
        self._mission_goal = GOAL
        self._physical_recovery_enabled = True
        self._physical_recovery = PhysicalRecoveryPolicy(
            enabled=True,
            cooldown_sec=2.0,
            maximum_cycles=2,
        )
        self._nav2_retry_count = 0
        self._nav2_retry_not_before_s = 0.0
        self._maximum_nav2_retries = 2
        self._nav2_retry_cooldown_sec = 2.0
        self._preserve_authorization_after_retry_exhaustion = True
        self._policy = _ResultPolicy()
        self.cleared_reason = None
        self.disarm_requests = 0

    def get_logger(self):
        return _Logger()

    def _clear_authorization(self, reason):
        self.cleared_reason = reason
        self._authorized_reference = None

    def _request_safety_disarm(self):
        self.disarm_requests += 1


def complete(harness, action, status):
    harness._pending_goal_kind = action
    harness._active_goal_handle = object()
    supervisor.SemanticNavigationSupervisorNode._on_action_result(
        harness, _ResultFuture(status))


def test_navigation_recovery_sequence_preserves_target_anchor_goal_and_arm():
    harness = _RecoveryResultHarness()

    complete(harness, GoalAction.NAVIGATE, GoalStatus.STATUS_ABORTED)
    assert harness._physical_recovery.stage is RecoveryStage.SPIN

    complete(harness, RecoveryCommand.SPIN, GoalStatus.STATUS_SUCCEEDED)
    assert harness._physical_recovery.stage is RecoveryStage.NAVIGATE_AFTER_SPIN

    complete(harness, GoalAction.NAVIGATE, GoalStatus.STATUS_ABORTED)
    assert harness._physical_recovery.stage is RecoveryStage.BACK_UP

    assert harness._authorized_reference == REFERENCE
    assert harness._authorized_target_anchor_xy == ANCHOR
    assert harness._mission_goal == GOAL
    assert harness.cleared_reason is None
    assert harness.disarm_requests == 0


def test_failed_recovery_advances_then_holds_without_clearing_mission():
    harness = _RecoveryResultHarness()
    complete(harness, GoalAction.NAVIGATE, GoalStatus.STATUS_ABORTED)

    complete(harness, RecoveryCommand.SPIN, GoalStatus.STATUS_ABORTED)
    assert harness._physical_recovery.stage is RecoveryStage.BACK_UP

    complete(harness, RecoveryCommand.BACK_UP, GoalStatus.STATUS_ABORTED)
    assert harness._physical_recovery.stage is RecoveryStage.HOLD
    assert harness._authorized_reference == REFERENCE
    assert harness._mission_goal == GOAL
    assert harness.disarm_requests == 0


def test_successful_navigation_still_completes_and_disarms_the_mission():
    harness = _RecoveryResultHarness()

    complete(harness, GoalAction.NAVIGATE, GoalStatus.STATUS_SUCCEEDED)

    assert harness.cleared_reason == 'nav2_action_succeeded'
    assert harness.disarm_requests == 1


class _PendingFuture:
    def __init__(self):
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback


class _ActionClient:
    def __init__(self):
        self.requests = []

    def server_is_ready(self):
        return True

    def send_goal_async(self, request):
        self.requests.append(request)
        return _PendingFuture()


class _DispatchHarness:
    def __init__(self, spin_sign=-1):
        self._pending_goal_kind = None
        self._active_goal_handle = None
        self._cancel_when_accepted = False
        self._recovery_spin_angle_rad = 0.523599
        self._recovery_backup_distance_m = 0.25
        self._recovery_backup_speed_mps = 0.10
        self._physical_recovery = PhysicalRecoveryPolicy(
            enabled=True,
            cooldown_sec=2.0,
            maximum_cycles=2,
            spin_sign=spin_sign,
        )
        self._spin_client = _ActionClient()
        self._back_up_client = _ActionClient()
        self._policy = _ResultPolicy()

    def get_logger(self):
        return _Logger()

    def _on_goal_response(self, _future):
        pass


def test_recovery_goal_construction_is_bounded_and_reverse_is_negative_x():
    harness = _DispatchHarness(spin_sign=-1)

    supervisor.SemanticNavigationSupervisorNode._dispatch_recovery(
        harness, RecoveryCommand.SPIN)
    spin = harness._spin_client.requests[0]
    assert spin.target_yaw == pytest.approx(-0.523599)
    assert harness._pending_goal_kind is RecoveryCommand.SPIN

    harness._pending_goal_kind = None
    supervisor.SemanticNavigationSupervisorNode._dispatch_recovery(
        harness, RecoveryCommand.BACK_UP)
    backup = harness._back_up_client.requests[0]
    assert backup.target.x == pytest.approx(-0.25)
    assert backup.target.y == pytest.approx(0.0)
    assert backup.target.z == pytest.approx(0.0)
    assert backup.speed == pytest.approx(0.10)
    assert harness._pending_goal_kind is RecoveryCommand.BACK_UP


def mission_snapshot(blocked=False, goal_in_flight=False):
    return StaticMissionSnapshot(
        memory_epoch_id=11,
        global_object_id=22,
        localization_epoch_id=33,
        observed_localization_epoch_id=33,
        odom_age_sec=0.05,
        safety_armed=True,
        safety_permits_motion=not blocked,
        safety_temporarily_blocked=blocked,
        goal_in_flight=goal_in_flight,
    )


class _SupervisionHarness:
    def __init__(self, blocked=False, active=False, cloud_fresh=True):
        self._physical_recovery_enabled = True
        self._physical_recovery = PhysicalRecoveryPolicy(
            enabled=True,
            cooldown_sec=2.0,
            maximum_cycles=2,
        )
        self._physical_recovery.navigation_aborted(1.0)
        self._mission_policy = StaticTargetMissionPolicy(0.25)
        self._snapshot_value = mission_snapshot(blocked, active)
        self._pending_goal_kind = (
            RecoveryCommand.SPIN if active else None)
        self._active_goal_handle = object() if active else None
        self._maximum_odom_age_sec = 0.25
        self._safety = clear_safety(
            state=(SafetyState.STATE_BLOCKED
                   if blocked else SafetyState.STATE_CLEAR),
            cloud_fresh=cloud_fresh,
        )
        self.dispatched = []
        self.cancelled = []
        self.cleared = []
        self.disarmed = 0
        self.diagnostics = []

    def _static_mission_snapshot(self):
        return self._snapshot_value

    def _dispatch_recovery(self, command):
        self.dispatched.append(command)

    def _dispatch(self, action):
        self.dispatched.append(action)

    def _cancel_action(self, reason):
        self.cancelled.append(reason)

    def _clear_authorization(self, reason):
        self.cleared.append(reason)

    def _request_safety_disarm(self):
        self.disarmed += 1

    def _publish_diagnostics(self, action, reason, key):
        self.diagnostics.append((action, reason, key))


def test_pending_recovery_is_dispatched_even_when_static_policy_is_blocked():
    harness = _SupervisionHarness(blocked=True)

    handled = supervisor.SemanticNavigationSupervisorNode._supervise_static_mission(
        harness)

    assert handled is True
    assert harness.dispatched == [RecoveryCommand.SPIN]
    assert harness.cancelled == []


def test_stale_cloud_cancels_an_active_recovery_but_preserves_the_mission():
    harness = _SupervisionHarness(active=True, cloud_fresh=False)

    supervisor.SemanticNavigationSupervisorNode._supervise_static_mission(
        harness)

    assert harness.cancelled == ['recovery_cloud_stale']
    assert harness.cleared == []
    assert harness.disarmed == 0


def test_hard_safety_callback_cancels_every_motion_action_immediately():
    class Harness:
        def __init__(self):
            self._pending_authorization = None
            self.cleared = []
            self.cancelled = []
            self.disarmed = 0

        def _clear_authorization(self, reason):
            self.cleared.append(reason)

        def _cancel_action(self, reason):
            self.cancelled.append(reason)

        def _request_safety_disarm(self):
            self.disarmed += 1

    harness = Harness()
    safety = clear_safety(rc_override_active=True)

    supervisor.SemanticNavigationSupervisorNode._on_safety(harness, safety)

    assert harness.cleared == ['safety_hard_stop']
    assert harness.cancelled == ['safety_hard_stop']
    assert harness.disarmed == 1
