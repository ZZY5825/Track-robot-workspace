"""Pure authorization and watchdog policy for Phase 5A rotation."""

from dataclasses import dataclass
import math


_ROS_FLOAT32_REL_TOL = 1e-6
_ROS_FLOAT32_ABS_TOL = 1e-7


def _finite(value, name):
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError('{} must be finite'.format(name))
    return parsed


def _exceeds_ros_float32_limit(value, limit):
    """Compare a float32 message value with a double-precision parameter."""
    return value > limit and not math.isclose(
        value,
        limit,
        rel_tol=_ROS_FLOAT32_REL_TOL,
        abs_tol=_ROS_FLOAT32_ABS_TOL,
    )


@dataclass(frozen=True)
class MotionLimits:
    maximum_individual_rotation_rad: float
    maximum_angular_speed_rad_s: float
    odometry_timeout_sec: float
    safety_timeout_sec: float
    intent_timeout_sec: float

    def __post_init__(self):
        for name in (
                'maximum_individual_rotation_rad',
                'maximum_angular_speed_rad_s',
                'odometry_timeout_sec',
                'safety_timeout_sec',
                'intent_timeout_sec'):
            if _finite(getattr(self, name), name) <= 0.0:
                raise ValueError('{} must be positive'.format(name))

    @classmethod
    def defaults(cls):
        return cls(
            maximum_individual_rotation_rad=math.radians(90.0),
            maximum_angular_speed_rad_s=0.30,
            odometry_timeout_sec=0.25,
            safety_timeout_sec=0.25,
            intent_timeout_sec=0.50,
        )


@dataclass(frozen=True)
class MotionIntentRequest:
    query_id: int
    signed_rotation_rad: float
    maximum_rotation_rad: float
    maximum_angular_speed_rad_s: float
    deadline_monotonic: float
    rotation_permitted: bool
    forward_permitted: bool
    stop: bool = False


@dataclass(frozen=True)
class MotionTransition:
    accepted: bool
    reason: str
    cancel_spin: bool = False
    disarm: bool = False


class SearchMotionPolicy:
    """Bind one operator authorization to one search query."""

    def __init__(self, limits):
        if not isinstance(limits, MotionLimits):
            raise TypeError('limits must be MotionLimits')
        self._limits = limits
        self._pending = None
        self._authorized_query_id = None
        self._active_query_id = None

    @property
    def pending(self):
        return self._pending

    @property
    def limits(self):
        return self._limits

    @property
    def authorized_query_id(self):
        return self._authorized_query_id

    @property
    def active_query_id(self):
        return self._active_query_id

    def _clear(self):
        self._pending = None
        self._authorized_query_id = None
        self._active_query_id = None

    def accept_intent(self, intent, now_monotonic):
        if not isinstance(intent, MotionIntentRequest):
            raise TypeError('intent must be MotionIntentRequest')
        now = _finite(now_monotonic, 'now_monotonic')
        if not isinstance(intent.query_id, int) or intent.query_id < 1:
            return MotionTransition(False, 'invalid_query_id')
        if intent.stop:
            if (
                    self._authorized_query_id is not None and
                    intent.query_id != self._authorized_query_id):
                return MotionTransition(False, 'stop_query_mismatch')
            self._clear()
            return MotionTransition(
                True, 'stop_intent', cancel_spin=True, disarm=True)

        angle = _finite(intent.signed_rotation_rad, 'signed_rotation_rad')
        requested_limit = _finite(
            intent.maximum_rotation_rad, 'maximum_rotation_rad')
        speed = _finite(
            intent.maximum_angular_speed_rad_s,
            'maximum_angular_speed_rad_s')
        deadline = _finite(intent.deadline_monotonic, 'deadline_monotonic')
        if intent.forward_permitted:
            return MotionTransition(False, 'forward_motion_forbidden')
        if not intent.rotation_permitted:
            self._pending = None
            return MotionTransition(True, 'shadow_intent_recorded')
        if abs(angle) <= 1e-6:
            return MotionTransition(False, 'rotation_angle_too_small')
        allowed_angle = min(
            requested_limit,
            self._limits.maximum_individual_rotation_rad)
        if requested_limit <= 0.0 or _exceeds_ros_float32_limit(
                abs(angle), allowed_angle):
            return MotionTransition(False, 'rotation_angle_limit_exceeded')
        if speed <= 0.0 or _exceeds_ros_float32_limit(
                speed, self._limits.maximum_angular_speed_rad_s):
            return MotionTransition(False, 'rotation_speed_limit_exceeded')
        if deadline <= now:
            return MotionTransition(False, 'intent_deadline_expired')
        if (
                self._authorized_query_id is not None and
                intent.query_id != self._authorized_query_id):
            return MotionTransition(False, 'authorized_query_conflict')
        self._pending = intent
        self._authorized_query_id = intent.query_id
        return MotionTransition(True, 'authorized_intent_ready')

    def authorize(self, query_id, now_monotonic):
        now = _finite(now_monotonic, 'now_monotonic')
        if self._pending is None:
            return MotionTransition(False, 'no_pending_rotation_intent')
        if int(query_id) != self._pending.query_id:
            return MotionTransition(False, 'authorization_query_mismatch')
        if self._pending.deadline_monotonic <= now:
            self._pending = None
            return MotionTransition(False, 'intent_deadline_expired')
        self._authorized_query_id = self._pending.query_id
        return MotionTransition(True, 'rotation_authorized')

    def begin_spin(self, query_id, now_monotonic):
        now = _finite(now_monotonic, 'now_monotonic')
        if self._pending is None:
            return MotionTransition(False, 'no_pending_rotation_intent')
        if self._authorized_query_id != int(query_id):
            return MotionTransition(False, 'rotation_not_authorized')
        if self._pending.query_id != int(query_id):
            return MotionTransition(False, 'pending_query_mismatch')
        if self._pending.deadline_monotonic <= now:
            self._clear()
            return MotionTransition(
                False,
                'intent_deadline_expired',
                cancel_spin=True,
                disarm=True,
            )
        self._active_query_id = int(query_id)
        return MotionTransition(True, 'spin_ready')

    def complete_spin(self, query_id):
        if self._active_query_id != int(query_id):
            return MotionTransition(False, 'active_query_mismatch')
        self._active_query_id = None
        self._pending = None
        return MotionTransition(True, 'spin_completed')

    def update_safety(self, healthy, reason):
        if healthy:
            return MotionTransition(True, 'safety_healthy')
        self._clear()
        return MotionTransition(
            False,
            str(reason) or 'safety_unhealthy',
            cancel_spin=True,
            disarm=True,
        )

    def expire(self, now_monotonic):
        now = _finite(now_monotonic, 'now_monotonic')
        if self._pending is None or self._pending.deadline_monotonic > now:
            return MotionTransition(True, 'intent_fresh')
        self._clear()
        return MotionTransition(
            False,
            'intent_deadline_expired',
            cancel_spin=True,
            disarm=True,
        )

    def cancel(self, reason):
        self._clear()
        return MotionTransition(
            True,
            str(reason) or 'cancelled',
            cancel_spin=True,
            disarm=True,
        )
