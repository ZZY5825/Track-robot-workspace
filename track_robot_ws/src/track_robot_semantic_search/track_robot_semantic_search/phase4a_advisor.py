"""Pure human-readable advisory generation for Phase 4A."""

from dataclasses import dataclass
import math
from typing import Optional, Tuple


@dataclass(frozen=True)
class AdvisoryTarget:
    query_text: str
    memory_epoch_id: int
    global_object_id: int
    localization_epoch_id: int
    query_id: int
    query_version: int
    x: float
    y: float
    z: float
    confidence: float
    uncertainty: float


@dataclass(frozen=True)
class AdvisoryGoal:
    x: float
    y: float


@dataclass(frozen=True)
class AdvisoryInput:
    planner_status: str
    planner_reason: str
    planner_memory_epoch_id: int
    planner_global_object_id: int
    planner_localization_epoch_id: int
    planner_query_id: int
    planner_query_version: int
    target: Optional[AdvisoryTarget]
    goal: Optional[AdvisoryGoal]
    path: Tuple[Tuple[float, float], ...]
    standoff_distance: float


@dataclass(frozen=True)
class AdvisoryResult:
    status: str
    reason: str
    text: str
    range_m: float = 0.0
    bearing_deg: float = 0.0
    path_length_m: float = 0.0


def _not_ready(reason: str) -> AdvisoryResult:
    bounded = ''.join(
        character for character in str(reason)
        if 0x20 <= ord(character) <= 0x7e)[:128]
    bounded = bounded or 'invalid_state'
    return AdvisoryResult(
        status='NOT_READY',
        reason=bounded,
        text='NOT_READY reason={} ADVISORY_ONLY'.format(bounded),
    )


def _longitudinal(x: float) -> str:
    return 'front' if x >= 0.0 else 'behind'


def _lateral(y: float) -> str:
    if abs(y) < 0.05:
        return 'center'
    return 'left' if y > 0.0 else 'right'


def _path_length(path):
    return sum(
        math.hypot(
            path[index][0] - path[index - 1][0],
            path[index][1] - path[index - 1][1],
        )
        for index in range(1, len(path))
    )


def build_advice(value: AdvisoryInput) -> AdvisoryResult:
    """Build one bounded advisory, failing closed on incomplete references."""

    if value.planner_status != 'PASS' or value.planner_reason != 'planned':
        return _not_ready(value.planner_reason)
    if value.target is None:
        return _not_ready('missing_target')
    target = value.target
    if (
            not target.query_text
            or len(target.query_text) > 512
            or any(
                ord(character) < 0x20 or ord(character) > 0x7e
                for character in target.query_text)):
        return _not_ready('invalid_query_text')
    expected = (
        target.memory_epoch_id,
        target.global_object_id,
        target.localization_epoch_id,
        target.query_id,
        target.query_version,
    )
    observed = (
        value.planner_memory_epoch_id,
        value.planner_global_object_id,
        value.planner_localization_epoch_id,
        value.planner_query_id,
        value.planner_query_version,
    )
    if expected != observed:
        return _not_ready('reference_mismatch')
    if value.goal is None:
        return _not_ready('missing_goal')
    if not value.path:
        return _not_ready('missing_path')
    finite_values = (
        target.x, target.y, target.z,
        target.confidence, target.uncertainty,
        value.goal.x, value.goal.y,
        value.standoff_distance,
    )
    if not all(math.isfinite(item) for item in finite_values):
        return _not_ready('invalid_numeric_value')
    if any(
            not math.isfinite(coordinate)
            for point in value.path for coordinate in point):
        return _not_ready('invalid_path')

    range_m = math.hypot(target.x, target.y)
    bearing_deg = math.degrees(math.atan2(target.y, target.x))
    path_length_m = _path_length(value.path)
    longitudinal = _longitudinal(target.x)
    lateral = _lateral(target.y)
    position = '{} {:.2f}m'.format(longitudinal, abs(target.x))
    if lateral != 'center':
        position += ',{} {:.2f}m'.format(lateral, abs(target.y))
    approach = '{}-{}'.format(
        _longitudinal(value.goal.x), _lateral(value.goal.y))
    text = (
        'READY target="{query}" position={position} range={range:.2f}m '
        'bearing={bearing:.1f}deg approach={approach} '
        'goal=({goal_x:.2f},{goal_y:.2f})m standoff={standoff:.2f}m '
        'path=clear path_length={path_length:.2f}m '
        'confidence={confidence:.2f} uncertainty={uncertainty:.2f} '
        'ADVISORY_ONLY'
    ).format(
        query=target.query_text,
        position=position,
        range=range_m,
        bearing=bearing_deg,
        approach=approach,
        goal_x=value.goal.x,
        goal_y=value.goal.y,
        standoff=value.standoff_distance,
        path_length=path_length_m,
        confidence=target.confidence,
        uncertainty=target.uncertainty,
    )
    if len(text) > 512:
        return _not_ready('advice_too_long')
    return AdvisoryResult(
        status='READY',
        reason='ready',
        text=text,
        range_m=range_m,
        bearing_deg=bearing_deg,
        path_length_m=path_length_m,
    )
