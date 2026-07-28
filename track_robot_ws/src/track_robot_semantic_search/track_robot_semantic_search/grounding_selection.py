from dataclasses import dataclass
import math
import re
from types import MappingProxyType
from typing import Dict, Mapping, Optional, Sequence, Tuple


_MISSING = object()
_SHA256 = re.compile(r'^[0-9a-f]{64}$')


def _nested_value(
        report: Mapping[str, object], section: str, field: str) -> object:
    section_value = report.get(section, _MISSING)
    if not isinstance(section_value, Mapping):
        return _MISSING
    return section_value.get(field, _MISSING)


def _finite_number(
        value: object, minimum: float,
        maximum: Optional[float] = None) -> Optional[float]:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    try:
        number = float(value)
    except (OverflowError, ValueError):
        return None
    if not math.isfinite(number) or number < minimum:
        return None
    if maximum is not None and number > maximum:
        return None
    return number


def _number_at(
        report: Mapping[str, object], section: str, field: str,
        minimum: float, maximum: Optional[float] = None) -> Optional[float]:
    return _finite_number(
        _nested_value(report, section, field), minimum, maximum)


def _at_least(
        report: Mapping[str, object], section: str, field: str,
        threshold: float, maximum: Optional[float] = None) -> bool:
    value = _number_at(report, section, field, 0.0, maximum)
    return value is not None and value >= threshold


def _at_most(
        report: Mapping[str, object], section: str, field: str,
        threshold: float, maximum: Optional[float] = None) -> bool:
    value = _number_at(report, section, field, 0.0, maximum)
    return value is not None and value <= threshold


def _validation_threshold_selected(report: Mapping[str, object]) -> bool:
    selection = report.get('validation_selection', _MISSING)
    if not isinstance(selection, Mapping):
        return False
    status = selection.get('status', _MISSING)
    if not isinstance(status, str) or status != 'selected':
        return False
    threshold = _finite_number(
        selection.get('threshold', _MISSING), 0.0, 1.0)
    return threshold is not None


GATES = {
    'validation_threshold_selected': _validation_threshold_selected,
    'runtime_available': (
        lambda report: report.get('runtime_available', _MISSING) is True),
    'platform_compatible': (
        lambda report: report.get('platform_compatible', _MISSING) is True),
    'licence_approved': (
        lambda report: report.get('licence_approved', _MISSING) is True),
    'human_reviewed_test_labels': (
        lambda report:
        report.get('human_reviewed_test_labels', _MISSING) is True),
    'top1_recall_iou_50_at_least_0_85': (
        lambda report: _at_least(
            report, 'test_metrics', 'top1_recall_iou_50', 0.85, 1.0)),
    'false_accept_rate_at_most_0_05': (
        lambda report: _at_most(
            report, 'test_metrics', 'target_absent_false_accept_rate',
            0.05, 1.0)),
    'median_iou_at_least_0_50': (
        lambda report: _at_least(
            report, 'test_metrics', 'median_accepted_positive_iou',
            0.50, 1.0)),
    'latency_p95_at_most_150_ms': (
        lambda report: _at_most(
            report, 'resources', 'p95_complete_path_ms', 150.0)),
    'semantic_rate_at_least_5_hz': (
        lambda report: _at_least(
            report, 'resources', 'semantic_rate_hz', 5.0)),
    'incremental_cuda_at_most_1536_mib': (
        lambda report: _at_most(
            report, 'resources', 'incremental_cuda_reserved_mib', 1536.0)),
}


@dataclass(frozen=True)
class GroundingSelection:
    status: str
    selected_candidate_id: Optional[str]
    rejected: Mapping[str, Tuple[str, ...]]
    ranking: Tuple[str, ...]


def _identity(report: Mapping[str, object], field: str) -> str:
    value = report.get(field, _MISSING)
    if not isinstance(value, str) or not value or len(value) > 256:
        raise ValueError(
            '{} must be a bounded non-empty string'.format(field))
    return value


def _dataset_checksum(report: Mapping[str, object]) -> str:
    value = report.get('dataset_checksum', _MISSING)
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise ValueError(
            'dataset_checksum must be lowercase 64-character hex')
    return value


def _ranking_values(
        report: Mapping[str, object]) -> Tuple[float, float, float, float]:
    recall = _number_at(
        report, 'test_metrics', 'top1_recall_iou_50', 0.0, 1.0)
    false_accept = _number_at(
        report, 'test_metrics', 'target_absent_false_accept_rate',
        0.0, 1.0)
    median_iou = _number_at(
        report, 'test_metrics', 'median_accepted_positive_iou', 0.0, 1.0)
    latency = _number_at(
        report, 'resources', 'p95_complete_path_ms', 0.0)
    if (
            recall is None or false_accept is None or
            median_iou is None or latency is None):
        raise ValueError('passing report is missing finite ranking metrics')
    return recall, false_accept, median_iou, latency


def select_grounding_candidate(
        reports: Sequence[Mapping[str, object]]) -> GroundingSelection:
    """Apply all release gates and deterministically rank passing reports."""
    prepared = []
    seen_candidate_ids = set()
    dataset_ids = set()
    dataset_checksums = set()
    for report in reports:
        if not isinstance(report, Mapping):
            raise ValueError('grounding evaluation report must be an object')
        candidate_id = _identity(report, 'candidate_id')
        dataset_id = _identity(report, 'dataset_id')
        dataset_checksum = _dataset_checksum(report)
        if candidate_id in seen_candidate_ids:
            raise ValueError(
                'duplicate candidate_id: {}'.format(candidate_id))
        seen_candidate_ids.add(candidate_id)
        dataset_ids.add(dataset_id)
        dataset_checksums.add(dataset_checksum)
        prepared.append((candidate_id, report))

    if len(dataset_ids) > 1:
        raise ValueError('all reports must use the same dataset_id')
    if len(dataset_checksums) > 1:
        raise ValueError('all reports must use the same dataset_checksum')

    rejected: Dict[str, Tuple[str, ...]] = {}
    passing = []
    for candidate_id, report in prepared:
        reasons = [
            reason for reason, gate in GATES.items()
            if not gate(report)
        ]
        if reasons:
            rejected[candidate_id] = tuple(reasons)
            continue
        recall, false_accept, median_iou, latency = _ranking_values(report)
        passing.append((
            candidate_id, recall, false_accept, median_iou, latency))

    passing.sort(key=lambda item: (
        -item[1],
        item[2],
        -item[3],
        item[4],
        item[0],
    ))
    ranking = tuple(item[0] for item in passing)
    if not ranking:
        return GroundingSelection(
            status='unavailable',
            selected_candidate_id=None,
            rejected=MappingProxyType(rejected),
            ranking=(),
        )
    return GroundingSelection(
        status='selected',
        selected_candidate_id=ranking[0],
        rejected=MappingProxyType(rejected),
        ranking=ranking,
    )
