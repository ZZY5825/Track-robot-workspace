"""Pure Phase 5A execution policy for one locked stationary target."""

from dataclasses import dataclass
import math

from .semantic_goal_policy import GoalAction


def static_mission_reference_failure(locked_reference, live_reference):
    """Reject domain resets while ignoring visibility and transient object IDs."""

    if live_reference is None:
        return None
    if len(locked_reference) != 5 or len(live_reference) != 5:
        return 'invalid_target_reference'
    if int(live_reference[0]) != int(locked_reference[0]):
        return 'memory_epoch_changed'
    if int(live_reference[2]) != int(locked_reference[2]):
        return 'localization_epoch_changed'
    if (
            int(live_reference[3]) != int(locked_reference[3])
            or int(live_reference[4]) != int(locked_reference[4])):
        return 'query_changed'
    return None


@dataclass(frozen=True)
class StaticMissionSnapshot:
    memory_epoch_id: int
    global_object_id: int
    localization_epoch_id: int
    observed_localization_epoch_id: int
    odom_age_sec: float
    safety_armed: bool
    safety_permits_motion: bool
    safety_temporarily_blocked: bool
    goal_in_flight: bool

    @property
    def key(self):
        return self.memory_epoch_id, self.global_object_id


@dataclass(frozen=True)
class StaticMissionDecision:
    action: GoalAction
    reason: str
    terminate_mission: bool = False


class StaticTargetMissionPolicy:
    """Continue a locked odom-frame mission without a perception heartbeat."""

    def __init__(self, maximum_odom_age_sec):
        maximum_odom_age_sec = float(maximum_odom_age_sec)
        if (
                not math.isfinite(maximum_odom_age_sec)
                or not 0.0 < maximum_odom_age_sec <= 1.0):
            raise ValueError('maximum_odom_age_sec must be in (0, 1]')
        self._maximum_odom_age_sec = maximum_odom_age_sec

    def evaluate(self, snapshot):
        if not isinstance(snapshot, StaticMissionSnapshot):
            raise ValueError('snapshot must be StaticMissionSnapshot')
        if (
                snapshot.observed_localization_epoch_id > 0
                and snapshot.observed_localization_epoch_id
                != snapshot.localization_epoch_id):
            return StaticMissionDecision(
                GoalAction.CANCEL,
                'localization_epoch_changed',
                terminate_mission=True,
            )
        if not snapshot.safety_armed:
            return StaticMissionDecision(
                GoalAction.CANCEL,
                'safety_not_armed',
                terminate_mission=True,
            )
        if (
                not math.isfinite(snapshot.odom_age_sec)
                or snapshot.odom_age_sec > self._maximum_odom_age_sec):
            return StaticMissionDecision(
                GoalAction.CANCEL,
                'odometry_stale',
            )
        if snapshot.safety_temporarily_blocked:
            return StaticMissionDecision(
                GoalAction.HOLD,
                'static_mission_obstacle_blocked',
            )
        if not snapshot.safety_permits_motion:
            return StaticMissionDecision(
                GoalAction.CANCEL,
                'safety_motion_not_permitted',
            )
        if snapshot.goal_in_flight:
            return StaticMissionDecision(
                GoalAction.HOLD,
                'static_mission_goal_active',
            )
        return StaticMissionDecision(
            GoalAction.NAVIGATE,
            'static_mission_ready',
        )
