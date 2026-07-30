"""Pure fail-closed policy for turning Phase 4A goals into Nav2 requests."""

from dataclasses import dataclass
from enum import Enum

from .runtime_modes import RuntimeMode


class GoalAction(Enum):
    HOLD = 'HOLD'
    COMPUTE_PATH = 'COMPUTE_PATH'
    NAVIGATE = 'NAVIGATE'
    CANCEL = 'CANCEL'


@dataclass(frozen=True)
class SemanticGoalSnapshot:
    memory_epoch_id: int
    global_object_id: int
    localization_epoch_id: int
    query_id: int
    query_version: int
    snapshot_sequence: int
    target_age_sec: float
    goal_age_sec: float
    diagnostics_age_sec: float
    odom_age_sec: float
    goal_frame_id: str
    target_frame_id: str
    lifecycle_confirmed: bool
    position_valid: bool
    references_match: bool
    safety_armed: bool
    safety_permits_motion: bool
    safety_temporarily_blocked: bool

    @property
    def key(self):
        return (int(self.memory_epoch_id), int(self.global_object_id))


@dataclass(frozen=True)
class GoalDecision:
    action: GoalAction
    reason: str
    key: tuple = (0, 0)


class SemanticGoalPolicy:
    """Require fresh correlated inputs and stable identity before dispatch."""

    def __init__(
            self,
            runtime_mode,
            semantic_execution_enabled,
            confirmation_snapshots=2,
            maximum_target_age_sec=1.0,
            maximum_goal_age_sec=0.5,
            maximum_diagnostics_age_sec=0.5,
            maximum_odom_age_sec=0.25):
        self._mode = RuntimeMode.parse(runtime_mode)
        if self._mode not in (
                RuntimeMode.SEMANTIC_SHADOW,
                RuntimeMode.SEMANTIC_ACTIVE):
            raise ValueError(
                'semantic goal policy requires a semantic runtime mode')
        self._execution_enabled = bool(semantic_execution_enabled)
        self._confirmation_snapshots = max(1, int(confirmation_snapshots))
        self._maximum_target_age_sec = float(maximum_target_age_sec)
        self._maximum_goal_age_sec = float(maximum_goal_age_sec)
        self._maximum_diagnostics_age_sec = float(
            maximum_diagnostics_age_sec)
        self._maximum_odom_age_sec = float(maximum_odom_age_sec)
        self._candidate_key = (0, 0)
        self._last_sequence = None
        self._confirmation_count = 0
        self._dispatched = False

    @staticmethod
    def _hold(reason, key=(0, 0)):
        return GoalDecision(GoalAction.HOLD, reason, key)

    def _input_failure(self, value):
        if value.memory_epoch_id <= 0 or value.global_object_id <= 0:
            return 'invalid_target_reference'
        if (
                value.localization_epoch_id <= 0
                or value.query_id <= 0
                or value.query_version <= 0):
            return 'invalid_reference_epoch'
        if not value.references_match:
            return 'target_reference_mismatch'
        if not value.position_valid:
            return 'target_position_invalid'
        if not value.lifecycle_confirmed:
            return 'target_not_confirmed'
        if (
                not value.goal_frame_id
                or value.goal_frame_id != value.target_frame_id):
            return 'target_goal_frame_mismatch'
        ages = (
            value.target_age_sec,
            value.goal_age_sec,
            value.diagnostics_age_sec,
            value.odom_age_sec,
        )
        if any(age < 0.0 for age in ages):
            return 'timestamp_regression'
        if value.target_age_sec > self._maximum_target_age_sec:
            return 'target_stale'
        if value.goal_age_sec > self._maximum_goal_age_sec:
            return 'goal_stale'
        if value.diagnostics_age_sec > self._maximum_diagnostics_age_sec:
            return 'planner_diagnostics_stale'
        if value.odom_age_sec > self._maximum_odom_age_sec:
            return 'odometry_stale'
        return None

    def _reject_or_cancel(self, reason, key):
        if self._dispatched:
            self._dispatched = False
            return GoalDecision(GoalAction.CANCEL, reason, key)
        return self._hold(reason, key)

    def evaluate(self, value):
        failure = self._input_failure(value)
        if failure is not None:
            return self._reject_or_cancel(failure, value.key)

        key = value.key
        if key != self._candidate_key:
            previously_dispatched = self._dispatched
            self._candidate_key = key
            self._last_sequence = int(value.snapshot_sequence)
            self._confirmation_count = 1
            self._dispatched = False
            if previously_dispatched:
                return GoalDecision(
                    GoalAction.CANCEL, 'target_reference_changed', key)
        elif self._last_sequence != int(value.snapshot_sequence):
            self._last_sequence = int(value.snapshot_sequence)
            self._confirmation_count += 1

        if self._confirmation_count < self._confirmation_snapshots:
            return self._hold('confirming_target_reference', key)

        if self._mode is RuntimeMode.SEMANTIC_ACTIVE:
            if not self._execution_enabled:
                return self._reject_or_cancel(
                    'semantic_execution_disabled', key)
            if not value.safety_armed:
                return self._reject_or_cancel('safety_not_armed', key)
            if value.safety_temporarily_blocked:
                return self._hold('safety_obstacle_blocked', key)
            if not value.safety_permits_motion:
                return self._reject_or_cancel(
                    'safety_motion_not_permitted', key)

        if self._dispatched:
            return self._hold('goal_already_dispatched', key)

        self._dispatched = True
        action = (
            GoalAction.NAVIGATE
            if self._mode is RuntimeMode.SEMANTIC_ACTIVE
            else GoalAction.COMPUTE_PATH)
        return GoalDecision(action, 'goal_accepted', key)

    def mark_dispatch_failed(self):
        """Allow retry after TF or action-server dispatch failed."""
        self._dispatched = False

    def invalidate(self, reason):
        """Fail closed when a complete correlated snapshot cannot be built."""
        return self._reject_or_cancel(str(reason), self._candidate_key)
