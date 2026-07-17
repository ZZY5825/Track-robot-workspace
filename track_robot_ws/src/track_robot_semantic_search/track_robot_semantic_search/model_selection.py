from dataclasses import dataclass
import math
from typing import Dict, List, Mapping, Optional, Sequence


GATE_FIELDS = (
    'available',
    'python_compatible',
    'licence_approved',
    'memory_pass',
    'latency_pass',
)


@dataclass(frozen=True)
class SelectionResult:
    status: str
    selected_candidate_id: Optional[str]
    rejected: Dict[str, List[str]]
    accuracy_status: str


def validate_benchmark(payload: Mapping[str, object]) -> None:
    if not isinstance(payload, Mapping):
        raise ValueError('benchmark must be a JSON object')
    if payload.get('schema_version') != '1.0.0':
        raise ValueError('schema_version must be 1.0.0')
    run_id = payload.get('run_id')
    if not isinstance(run_id, str) or not run_id.strip():
        raise ValueError('run_id must not be empty')
    platform = payload.get('platform')
    if not isinstance(platform, Mapping):
        raise ValueError('platform must be an object')
    for field in ('python', 'pytorch', 'device'):
        value = platform.get(field)
        if not isinstance(value, str) or not value.strip():
            raise ValueError('platform.{} must not be empty'.format(field))
    candidates = payload.get('candidates')
    if not isinstance(candidates, list) or not candidates:
        raise ValueError('candidates must be a non-empty array')
    select_candidate(candidates)


def _finite_number(
        candidate: Mapping[str, object], field: str, lower: float,
        upper: Optional[float] = None) -> float:
    if field not in candidate:
        raise ValueError('candidate is missing {}'.format(field))
    value = candidate[field]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('{} must be a finite number'.format(field))
    number = float(value)
    if not math.isfinite(number) or number < lower or (
            upper is not None and number > upper):
        raise ValueError('{} is outside its valid range'.format(field))
    return number


def select_candidate(
        candidates: Sequence[Mapping[str, object]]) -> SelectionResult:
    seen = set()
    passing = []
    rejected: Dict[str, List[str]] = {}
    for candidate in candidates:
        candidate_id = candidate.get('candidate_id')
        if not isinstance(candidate_id, str) or not candidate_id.strip():
            raise ValueError('candidate_id must not be empty')
        candidate_id = candidate_id.strip()
        if candidate_id in seen:
            raise ValueError('duplicate candidate_id: {}'.format(candidate_id))
        seen.add(candidate_id)

        failed_gates = []
        for field in GATE_FIELDS:
            value = candidate.get(field)
            if not isinstance(value, bool):
                raise ValueError('{} must be boolean'.format(field))
            if not value:
                failed_gates.append(field)
        recall_value = candidate.get('phrase_region_recall')
        recall = None if recall_value is None else _finite_number(
            candidate, 'phrase_region_recall', 0.0, upper=1.0)
        latency = _finite_number(candidate, 'p95_latency_ms', 0.0)
        if failed_gates:
            rejected[candidate_id] = failed_gates
        else:
            passing.append((candidate_id, recall, latency))

    if not passing:
        return SelectionResult(
            status='unavailable',
            selected_candidate_id=None,
            rejected=rejected,
            accuracy_status='not_evaluated')
    evaluated = [item for item in passing if item[1] is not None]
    if evaluated:
        evaluated.sort(key=lambda item: (-item[1], item[2], item[0]))
        return SelectionResult(
            status='selected',
            selected_candidate_id=evaluated[0][0],
            rejected=rejected,
            accuracy_status='evaluated')
    passing.sort(key=lambda item: (item[2], item[0]))
    return SelectionResult(
        status='provisional_selected',
        selected_candidate_id=passing[0][0],
        rejected=rejected,
        accuracy_status='not_evaluated')
