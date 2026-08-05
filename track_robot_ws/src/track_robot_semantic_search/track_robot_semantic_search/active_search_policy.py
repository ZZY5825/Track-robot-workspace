"""Deterministic, bounded Phase 5A active-search policy."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple


class SearchMode(Enum):
    """Explicit active-search runtime modes."""

    PASSIVE_ONLY = 'PASSIVE_ONLY'
    SEARCH_SHADOW = 'SEARCH_SHADOW'
    ROTATION_SUPERVISED = 'ROTATION_SUPERVISED'
    NEXT_BEST_VIEW_SHADOW = 'NEXT_BEST_VIEW_SHADOW'
    NEXT_BEST_VIEW_ACTIVE = 'NEXT_BEST_VIEW_ACTIVE'

    @classmethod
    def parse(cls, value):
        try:
            return cls(str(value).strip().upper())
        except ValueError as error:
            choices = ', '.join(item.value for item in cls)
            raise ValueError(
                'invalid search_mode {!r}; expected one of {}'.format(
                    value, choices)
            ) from error

    @property
    def motion_enabled(self):
        return self is SearchMode.ROTATION_SUPERVISED

    @property
    def available_in_phase5a(self):
        return self in (
            SearchMode.PASSIVE_ONLY,
            SearchMode.SEARCH_SHADOW,
            SearchMode.ROTATION_SUPERVISED,
        )


class SearchState(Enum):
    """Observable states for one active-search task."""

    IDLE = 'IDLE'
    QUERY_ACCEPTED = 'QUERY_ACCEPTED'
    PASSIVE_OBSERVATION = 'PASSIVE_OBSERVATION'
    EVALUATING_EVIDENCE = 'EVALUATING_EVIDENCE'
    ACTIVE_SEARCH_REQUIRED = 'ACTIVE_SEARCH_REQUIRED'
    SELECTING_VIEW = 'SELECTING_VIEW'
    WAITING_FOR_AUTHORIZATION = 'WAITING_FOR_AUTHORIZATION'
    ROTATING = 'ROTATING'
    SETTLING = 'SETTLING'
    OBSERVING = 'OBSERVING'
    UPDATING_MEMORY = 'UPDATING_MEMORY'
    TARGET_CONFIRMED = 'TARGET_CONFIRMED'
    HANDOFF_TO_PHASE4 = 'HANDOFF_TO_PHASE4'
    CONFIRMED = 'CONFIRMED'
    NOT_FOUND = 'NOT_FOUND'
    UNCERTAIN = 'UNCERTAIN'
    CANCELLED = 'CANCELLED'
    TIMEOUT = 'TIMEOUT'
    SAFETY_REJECTED = 'SAFETY_REJECTED'
    SENSOR_UNAVAILABLE = 'SENSOR_UNAVAILABLE'
    MODEL_UNAVAILABLE = 'MODEL_UNAVAILABLE'
    LOCALIZATION_UNAVAILABLE = 'LOCALIZATION_UNAVAILABLE'
    SEARCH_SPACE_EXHAUSTED = 'SEARCH_SPACE_EXHAUSTED'
    INTERNAL_FAULT = 'INTERNAL_FAULT'

    @property
    def terminal(self):
        return self in (
            SearchState.CONFIRMED,
            SearchState.NOT_FOUND,
            SearchState.UNCERTAIN,
            SearchState.CANCELLED,
            SearchState.TIMEOUT,
            SearchState.SAFETY_REJECTED,
            SearchState.SENSOR_UNAVAILABLE,
            SearchState.MODEL_UNAVAILABLE,
            SearchState.LOCALIZATION_UNAVAILABLE,
            SearchState.SEARCH_SPACE_EXHAUSTED,
            SearchState.INTERNAL_FAULT,
        )


def _finite(value, name):
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError('{} must be finite'.format(name))
    return parsed


def _normalize_radians(value):
    return (value + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class SearchPolicyConfig:
    heading_offsets_deg: Tuple[float, ...]
    evidence_headings_deg: Tuple[float, ...]
    maximum_individual_rotation_deg: float
    maximum_cumulative_rotation_deg: float
    maximum_angular_speed_rad_s: float
    duplicate_heading_tolerance_deg: float
    default_deadline_sec: float

    def __post_init__(self):
        if not self.heading_offsets_deg:
            return
        positive_fields = (
            ('maximum_individual_rotation_deg',
             self.maximum_individual_rotation_deg),
            ('maximum_cumulative_rotation_deg',
             self.maximum_cumulative_rotation_deg),
            ('maximum_angular_speed_rad_s',
             self.maximum_angular_speed_rad_s),
            ('duplicate_heading_tolerance_deg',
             self.duplicate_heading_tolerance_deg),
            ('default_deadline_sec', self.default_deadline_sec),
        )
        for name, value in positive_fields:
            if _finite(value, name) <= 0.0:
                raise ValueError('{} must be positive'.format(name))
        for value in self.heading_offsets_deg + self.evidence_headings_deg:
            _finite(value, 'heading offset')

    @classmethod
    def defaults(cls, maximum_rotation_angle_deg=90.0):
        requested = _finite(
            maximum_rotation_angle_deg, 'maximum_rotation_angle_deg')
        if requested <= 0.0:
            raise ValueError('maximum_rotation_angle_deg must be positive')
        envelope = min(90.0, requested)
        evidence = tuple(
            value for value in (45.0, 90.0, -45.0, -90.0)
            if abs(value) <= envelope
        )
        offsets = tuple(
            value for value in (45.0, 90.0, 0.0, -45.0, -90.0)
            if abs(value) <= envelope
        ) if evidence else tuple()
        return cls(
            heading_offsets_deg=offsets,
            evidence_headings_deg=evidence,
            maximum_individual_rotation_deg=min(90.0, envelope),
            maximum_cumulative_rotation_deg=270.0,
            maximum_angular_speed_rad_s=0.30,
            duplicate_heading_tolerance_deg=10.0,
            default_deadline_sec=60.0,
        )


@dataclass(frozen=True)
class HeadingDecision:
    target_yaw_rad: float
    relative_heading_deg: float
    rotation_delta_deg: float
    cumulative_rotation_deg: float
    collect_evidence: bool


class BoundedHeadingPolicy:
    """Generate a fixed heading sequence without exceeding rotation budgets."""

    def __init__(self, config):
        if not isinstance(config, SearchPolicyConfig):
            raise TypeError('config must be SearchPolicyConfig')
        self._config = config
        self._cursor = 0
        self._cumulative_rotation_deg = 0.0
        self._pending_decision = None

    @property
    def cumulative_rotation_deg(self):
        return self._cumulative_rotation_deg

    @property
    def pending_decision(self):
        return self._pending_decision

    @property
    def exhausted(self):
        return self._cursor >= len(self._config.heading_offsets_deg)

    def next_heading(self, state, initial_yaw, current_yaw):
        if not isinstance(state, SearchState):
            raise TypeError('state must be SearchState')
        if state.terminal or self.exhausted:
            return None
        if self._pending_decision is not None:
            return self._pending_decision

        initial = _finite(initial_yaw, 'initial_yaw')
        current = _finite(current_yaw, 'current_yaw')
        offset = self._config.heading_offsets_deg[self._cursor]
        target = _normalize_radians(initial + math.radians(offset))
        delta_deg = round(math.degrees(
            _normalize_radians(target - current)), 12)
        next_cumulative = round(
            self._cumulative_rotation_deg + abs(delta_deg), 12)
        if (
                abs(delta_deg)
                > self._config.maximum_individual_rotation_deg + 1e-9 or
                next_cumulative
                > self._config.maximum_cumulative_rotation_deg + 1e-9):
            self._cursor = len(self._config.heading_offsets_deg)
            return None

        self._pending_decision = HeadingDecision(
            target_yaw_rad=target,
            relative_heading_deg=offset,
            rotation_delta_deg=delta_deg,
            cumulative_rotation_deg=next_cumulative,
            collect_evidence=offset in self._config.evidence_headings_deg,
        )
        return self._pending_decision

    def mark_completed(self, decision):
        if decision != self._pending_decision:
            raise ValueError('decision is not the pending heading')
        self._cumulative_rotation_deg = decision.cumulative_rotation_deg
        self._cursor += 1
        self._pending_decision = None

    def complete_sequence(self, initial_yaw):
        current_yaw = _finite(initial_yaw, 'initial_yaw')
        decisions = []
        while True:
            decision = self.next_heading(
                SearchState.SELECTING_VIEW,
                initial_yaw,
                current_yaw,
            )
            if decision is None:
                break
            decisions.append(decision)
            self.mark_completed(decision)
            current_yaw = decision.target_yaw_rad
        return tuple(decisions)
