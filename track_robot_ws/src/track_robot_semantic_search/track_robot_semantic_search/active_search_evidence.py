"""Bounded multi-view evidence for Phase 5A active search."""

from collections import Counter, deque
from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple


def _finite(value, name):
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError('{} must be finite'.format(name))
    return parsed


def _positive_int(value, name):
    if not isinstance(value, int) or isinstance(value, bool) or value < 1:
        raise ValueError('{} must be a positive integer'.format(name))
    return value


def _angular_distance_deg(first, second):
    return abs((first - second + 180.0) % 360.0 - 180.0)


@dataclass(frozen=True)
class ObjectEvidenceKey:
    memory_epoch_id: int
    global_object_id: int
    localization_epoch_id: int
    query_id: int
    query_version: int

    def __post_init__(self):
        for name in (
                'memory_epoch_id', 'global_object_id',
                'localization_epoch_id', 'query_id', 'query_version'):
            _positive_int(getattr(self, name), name)

    @property
    def domain(self):
        return (
            self.memory_epoch_id,
            self.localization_epoch_id,
            self.query_id,
            self.query_version,
        )


@dataclass(frozen=True)
class ViewEvidence:
    key: ObjectEvidenceKey
    heading_deg: float
    horizontal_fov_deg: Optional[float]
    source_stamp_sec: float
    task_relevance: float
    uncertainty: float
    phase3_selected: bool

    def __post_init__(self):
        if not isinstance(self.key, ObjectEvidenceKey):
            raise TypeError('key must be ObjectEvidenceKey')
        _finite(self.heading_deg, 'heading_deg')
        _finite(self.source_stamp_sec, 'source_stamp_sec')
        _finite(self.task_relevance, 'task_relevance')
        _finite(self.uncertainty, 'uncertainty')
        if self.horizontal_fov_deg is not None:
            fov = _finite(self.horizontal_fov_deg, 'horizontal_fov_deg')
            if fov <= 0.0 or fov > 360.0:
                raise ValueError(
                    'horizontal_fov_deg must be in (0, 360]')
        if not isinstance(self.phase3_selected, bool):
            raise TypeError('phase3_selected must be bool')


@dataclass(frozen=True)
class EvidenceConfig:
    confirmation_snapshots: int
    duplicate_heading_tolerance_deg: float
    evidence_ttl_sec: float
    maximum_records: int

    def __post_init__(self):
        _positive_int(self.confirmation_snapshots, 'confirmation_snapshots')
        _positive_int(self.maximum_records, 'maximum_records')
        tolerance = _finite(
            self.duplicate_heading_tolerance_deg,
            'duplicate_heading_tolerance_deg')
        if tolerance <= 0.0 or tolerance > 180.0:
            raise ValueError(
                'duplicate_heading_tolerance_deg must be in (0, 180]')
        if _finite(self.evidence_ttl_sec, 'evidence_ttl_sec') <= 0.0:
            raise ValueError('evidence_ttl_sec must be positive')

    @classmethod
    def defaults(cls):
        return cls(
            confirmation_snapshots=3,
            duplicate_heading_tolerance_deg=10.0,
            evidence_ttl_sec=12.0,
            maximum_records=40,
        )


class EvidenceStatus(Enum):
    OBSERVING = 'OBSERVING'
    CONFIRMED = 'CONFIRMED'
    NOT_FOUND = 'NOT_FOUND'
    UNCERTAIN = 'UNCERTAIN'


@dataclass(frozen=True)
class EvidenceDecision:
    status: EvidenceStatus
    selected_key: Optional[ObjectEvidenceKey]
    candidate_count: int
    covered_heading_count: int
    reason: str


class BoundedEvidenceBook:
    """Retain task-correlated evidence without owning object identity."""

    def __init__(self, config):
        if not isinstance(config, EvidenceConfig):
            raise TypeError('config must be EvidenceConfig')
        self._config = config
        self._domain = None
        self._records = deque(maxlen=config.maximum_records)
        self._covered_headings = []
        self._coverage_intervals = []

    @property
    def record_count(self):
        return len(self._records)

    @property
    def covered_heading_count(self):
        return len(self._covered_headings)

    @property
    def coverage_intervals_deg(self):
        return tuple(self._coverage_intervals)

    def bind_domain(
            self,
            memory_epoch_id,
            localization_epoch_id,
            query_id,
            query_version):
        domain = (
            _positive_int(memory_epoch_id, 'memory_epoch_id'),
            _positive_int(localization_epoch_id, 'localization_epoch_id'),
            _positive_int(query_id, 'query_id'),
            _positive_int(query_version, 'query_version'),
        )
        if self._domain is not None and domain != self._domain:
            raise ValueError('evidence domain is already bound')
        self._domain = domain

    def domain_changed(
            self,
            memory_epoch_id,
            localization_epoch_id,
            query_id,
            query_version):
        if self._domain is None:
            return True
        return self._domain != (
            int(memory_epoch_id),
            int(localization_epoch_id),
            int(query_id),
            int(query_version),
        )

    def _record_coverage(self, evidence):
        heading = float(evidence.heading_deg)
        if any(
                _angular_distance_deg(heading, existing)
                <= self._config.duplicate_heading_tolerance_deg
                for existing in self._covered_headings):
            return
        self._covered_headings.append(heading)
        if evidence.horizontal_fov_deg is None:
            interval = (heading, heading)
        else:
            half_fov = float(evidence.horizontal_fov_deg) / 2.0
            interval = (heading - half_fov, heading + half_fov)
        self._coverage_intervals.append(interval)

    def add(self, evidence, settled_after):
        if not isinstance(evidence, ViewEvidence):
            raise TypeError('evidence must be ViewEvidence')
        settled = _finite(settled_after, 'settled_after')
        if self._domain is None:
            raise ValueError('evidence domain is not bound')
        if evidence.key.domain != self._domain:
            return False
        if evidence.source_stamp_sec < settled:
            return False
        self._records.append(evidence)
        self._record_coverage(evidence)
        return True

    def expire(self, now_sec):
        now = _finite(now_sec, 'now_sec')
        oldest = now - self._config.evidence_ttl_sec
        retained = (
            item for item in self._records
            if item.source_stamp_sec >= oldest
        )
        self._records = deque(
            retained, maxlen=self._config.maximum_records)

    def evaluate(self, search_exhausted):
        if not isinstance(search_exhausted, bool):
            raise TypeError('search_exhausted must be bool')
        candidates = {item.key for item in self._records}
        selected_counts = Counter(
            item.key for item in self._records if item.phase3_selected)
        confirmed = tuple(
            key for key, count in selected_counts.items()
            if count >= self._config.confirmation_snapshots
        )
        common = {
            'candidate_count': len(candidates),
            'covered_heading_count': self.covered_heading_count,
        }
        if len(confirmed) == 1:
            return EvidenceDecision(
                EvidenceStatus.CONFIRMED,
                confirmed[0],
                reason='phase3_stable_target_confirmed',
                **common
            )
        if len(confirmed) > 1:
            return EvidenceDecision(
                EvidenceStatus.UNCERTAIN,
                None,
                reason='multiple_confirmed_object_keys',
                **common
            )
        if not search_exhausted:
            return EvidenceDecision(
                EvidenceStatus.OBSERVING,
                None,
                reason='more_evidence_required',
                **common
            )
        if not candidates:
            return EvidenceDecision(
                EvidenceStatus.NOT_FOUND,
                None,
                reason='search_exhausted_without_candidates',
                **common
            )
        return EvidenceDecision(
            EvidenceStatus.UNCERTAIN,
            None,
            reason='search_exhausted_with_unresolved_candidates',
            **common
        )
