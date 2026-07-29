"""Conservative test-only target selection for a fixed-base session."""

from collections import deque
from dataclasses import dataclass
import math
import statistics
from typing import Optional, Tuple


@dataclass(frozen=True)
class SelectorConfig:
    minimum_relevance: float = 0.50
    minimum_margin: float = 0.08
    maximum_uncertainty: float = 0.50
    confirmation_snapshots: int = 3
    position_window: int = 5
    maximum_xy_spread_m: float = 0.35
    maximum_age_ns: int = 1_000_000_000
    frame_id: str = 'base_link'

    def validate(self):
        if not 0.0 <= self.minimum_relevance <= 1.0:
            raise ValueError('minimum_relevance must be in [0, 1]')
        if not 0.0 <= self.minimum_margin <= 1.0:
            raise ValueError('minimum_margin must be in [0, 1]')
        if not 0.0 <= self.maximum_uncertainty <= 1.0:
            raise ValueError('maximum_uncertainty must be in [0, 1]')
        if self.confirmation_snapshots <= 0 or self.position_window <= 0:
            raise ValueError('confirmation and position windows must be positive')
        if self.maximum_xy_spread_m < 0.0 or self.maximum_age_ns <= 0:
            raise ValueError('spread and age limits must be positive')
        if not self.frame_id:
            raise ValueError('frame_id is required')


@dataclass(frozen=True)
class ObjectCandidate:
    memory_epoch_id: int
    global_object_id: int
    localization_epoch_id: int
    query_id: int
    query_version: int
    lifecycle: str
    support: str
    position_frame_id: str
    position_valid: bool
    x: float
    y: float
    z: float
    relevance: float
    uncertainty: float
    last_seen_ns: int

    @property
    def key(self) -> Tuple[int, int]:
        return self.memory_epoch_id, self.global_object_id


@dataclass(frozen=True)
class SelectionSnapshot:
    now_ns: int
    query_id: int
    query_version: int
    candidates: Tuple[ObjectCandidate, ...]


@dataclass(frozen=True)
class SelectionResult:
    status: str
    reason: str
    target: Optional[ObjectCandidate] = None
    confirmation_count: int = 0
    xy_spread_m: float = 0.0


class FixedBaseTargetSelector:
    """Require fresh, stable camera/LiDAR evidence before test selection."""

    def __init__(self, config: SelectorConfig):
        config.validate()
        self._config = config
        self._key = None
        self._confirmation_count = 0
        self._positions = deque(maxlen=config.position_window)

    def _clear(self):
        self._key = None
        self._confirmation_count = 0
        self._positions.clear()

    def _fail(self, reason: str) -> SelectionResult:
        self._clear()
        return SelectionResult(status='NOT_READY', reason=reason)

    def update(self, snapshot: SelectionSnapshot) -> SelectionResult:
        if not snapshot.candidates:
            return self._fail('no_target')
        ordered = sorted(
            snapshot.candidates,
            key=lambda item: (
                -item.relevance,
                item.memory_epoch_id,
                item.global_object_id,
            ),
        )
        top = ordered[0]
        if (
                snapshot.query_id <= 0
                or snapshot.query_version <= 0
                or top.query_id != snapshot.query_id
                or top.query_version != snapshot.query_version):
            return self._fail('query_mismatch')
        if top.lifecycle != 'confirmed':
            return self._fail('target_not_confirmed')
        if top.support not in ('camera_lidar', 'camera_depth'):
            return self._fail('no_spatial_support')
        if not top.position_valid or not all(
                math.isfinite(value) for value in (top.x, top.y, top.z)):
            return self._fail('invalid_position')
        if top.position_frame_id != self._config.frame_id:
            return self._fail('frame_mismatch')
        if not math.isfinite(top.relevance) or (
                top.relevance < self._config.minimum_relevance):
            return self._fail('below_test_relevance')
        if not math.isfinite(top.uncertainty) or (
                top.uncertainty > self._config.maximum_uncertainty):
            return self._fail('uncertainty_too_high')
        age_ns = snapshot.now_ns - top.last_seen_ns
        if age_ns < 0 or age_ns > self._config.maximum_age_ns:
            return self._fail('stale_target')
        if len(ordered) > 1 and (
                top.relevance - ordered[1].relevance
                < self._config.minimum_margin):
            return self._fail('ambiguous_target')

        if top.key != self._key:
            self._key = top.key
            self._confirmation_count = 0
            self._positions.clear()
        self._confirmation_count += 1
        self._positions.append((top.x, top.y))
        median_x = statistics.median(item[0] for item in self._positions)
        median_y = statistics.median(item[1] for item in self._positions)
        spread = max(
            math.hypot(x - median_x, y - median_y)
            for x, y in self._positions)
        if spread > self._config.maximum_xy_spread_m:
            self._clear()
            return SelectionResult(
                status='NOT_READY',
                reason='unstable_position',
                xy_spread_m=spread,
            )
        if self._confirmation_count < self._config.confirmation_snapshots:
            return SelectionResult(
                status='NOT_READY',
                reason='confirming_target',
                confirmation_count=self._confirmation_count,
                xy_spread_m=spread,
            )
        return SelectionResult(
            status='READY',
            reason='ready',
            target=top,
            confirmation_count=self._confirmation_count,
            xy_spread_m=spread,
        )
