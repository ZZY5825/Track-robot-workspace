"""Deterministic bounded tracking for language-grounded camera candidates."""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .yolo_world_backend import GroundedDetection


@dataclass(frozen=True)
class AppearanceDescriptor:
    values: np.ndarray
    quality: float
    encoder_id: str
    checkpoint_id: str
    version: int

    def __post_init__(self):
        values = np.asarray(self.values, dtype=np.float32)
        if (
                values.ndim != 1 or not 1 <= values.size <= 1024 or
                not np.isfinite(values).all()):
            raise ValueError('appearance descriptor values are invalid')
        norm = float(np.linalg.norm(values))
        if not math.isfinite(norm) or abs(norm - 1.0) > 1e-4:
            raise ValueError('appearance descriptor must be unit normalized')
        if (
                not isinstance(self.quality, (int, float)) or
                isinstance(self.quality, bool) or
                not math.isfinite(self.quality) or
                not 0.0 <= float(self.quality) <= 1.0):
            raise ValueError('appearance descriptor quality is invalid')
        if (
                not isinstance(self.encoder_id, str) or
                not self.encoder_id or len(self.encoder_id) > 128 or
                not isinstance(self.checkpoint_id, str) or
                not self.checkpoint_id or len(self.checkpoint_id) > 128 or
                isinstance(self.version, bool) or
                not isinstance(self.version, int) or self.version <= 0):
            raise ValueError('appearance descriptor identity is invalid')
        copied = values.copy()
        copied.setflags(write=False)
        object.__setattr__(self, 'values', copied)
        object.__setattr__(self, 'quality', float(self.quality))


@dataclass(frozen=True)
class DetectionInput:
    detection: GroundedDetection
    descriptor: Optional[AppearanceDescriptor] = None

    def __post_init__(self):
        if not isinstance(self.detection, GroundedDetection):
            raise ValueError('detection must be GroundedDetection')
        if (
                self.descriptor is not None and
                not isinstance(self.descriptor, AppearanceDescriptor)):
            raise ValueError('descriptor type is invalid')


@dataclass(frozen=True)
class CameraCandidate:
    candidate_id: int
    camera_track_id: int
    detection: GroundedDetection
    descriptor: Optional[AppearanceDescriptor]


@dataclass(frozen=True)
class CameraTrackingResult:
    producer_epoch_id: int
    rollback_count: int
    candidates: Tuple[CameraCandidate, ...]


@dataclass(frozen=True)
class CameraTrackingConfig:
    minimum_iou: float = 0.20
    maximum_normalized_center_distance: float = 0.30
    ambiguity_margin: float = 0.05
    minimum_appearance_similarity: float = 0.80
    maximum_missed_frames: int = 8
    maximum_tracks: int = 64

    def __post_init__(self):
        probabilities = (
            self.minimum_iou,
            self.maximum_normalized_center_distance,
            self.ambiguity_margin,
            self.minimum_appearance_similarity,
        )
        if any(
                not isinstance(value, (int, float)) or
                isinstance(value, bool) or not math.isfinite(value) or
                not 0.0 <= float(value) <= 1.0
                for value in probabilities):
            raise ValueError('camera tracking thresholds are invalid')
        if (
                isinstance(self.maximum_missed_frames, bool) or
                not isinstance(self.maximum_missed_frames, int) or
                not 0 <= self.maximum_missed_frames <= 120 or
                isinstance(self.maximum_tracks, bool) or
                not isinstance(self.maximum_tracks, int) or
                not 1 <= self.maximum_tracks <= 64):
            raise ValueError('camera tracking bounds are invalid')


@dataclass
class _Track:
    track_id: int
    detection: GroundedDetection
    descriptor: Optional[AppearanceDescriptor]
    missed_frames: int = 0
    last_stamp_ns: int = 0
    box_velocity: Tuple[float, float, float, float] = (
        0.0, 0.0, 0.0, 0.0)


def _box_area(value):
    return max(0.0, value.x2 - value.x1) * max(0.0, value.y2 - value.y1)


def _iou(first, second):
    left = max(first.x1, second.x1)
    top = max(first.y1, second.y1)
    right = min(first.x2, second.x2)
    bottom = min(first.y2, second.y2)
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    union = _box_area(first) + _box_area(second) - intersection
    return intersection / union if union > 0.0 else 0.0


def _normalized_center_distance(first, second):
    first_x = (first.x1 + first.x2) * 0.5
    first_y = (first.y1 + first.y2) * 0.5
    second_x = (second.x1 + second.x2) * 0.5
    second_y = (second.y1 + second.y2) * 0.5
    scale = max(
        math.hypot(first.x2 - first.x1, first.y2 - first.y1),
        math.hypot(second.x2 - second.x1, second.y2 - second.y1),
        1e-6,
    )
    return math.hypot(first_x - second_x, first_y - second_y) / scale


def _descriptor_cosine(first, second):
    if first is None or second is None:
        return None
    if (
            first.encoder_id != second.encoder_id or
            first.checkpoint_id != second.checkpoint_id or
            first.version != second.version or
            first.values.shape != second.values.shape):
        return None
    value = float(np.dot(first.values, second.values))
    return min(max(value, -1.0), 1.0) if math.isfinite(value) else None


def _predicted_detection(track, stamp_ns):
    if track.last_stamp_ns <= 0 or stamp_ns <= track.last_stamp_ns:
        return track.detection
    # Prediction is deliberately bounded: it bridges normal detector gaps and
    # camera motion without allowing an old track to extrapolate indefinitely.
    delta_sec = min(
        float(stamp_ns - track.last_stamp_ns) / 1_000_000_000.0,
        2.0,
    )
    velocity = track.box_velocity
    return GroundedDetection(
        track.detection.x1 + velocity[0] * delta_sec,
        track.detection.y1 + velocity[1] * delta_sec,
        track.detection.x2 + velocity[2] * delta_sec,
        track.detection.y2 + velocity[3] * delta_sec,
        track.detection.score,
        track.detection.label,
    )


def _detection_key(value):
    detection = value.detection
    return (
        -detection.score,
        detection.x1,
        detection.y1,
        detection.x2,
        detection.y2,
        detection.label,
    )


class CameraTrackManager:
    def __init__(self, config: CameraTrackingConfig):
        if not isinstance(config, CameraTrackingConfig):
            raise ValueError('config must be CameraTrackingConfig')
        self._config = config
        self._producer_epoch_id = 1
        self._rollback_count = 0
        self._query_key = None
        self._last_stamp_ns = None
        self._next_track_id = 1
        self._next_candidate_id = 1
        self._tracks = {}

    def reset(self, producer_epoch_id=None):
        if producer_epoch_id is None:
            producer_epoch_id = self._producer_epoch_id + 1
        if (
                isinstance(producer_epoch_id, bool) or
                not isinstance(producer_epoch_id, int) or
                producer_epoch_id <= self._producer_epoch_id):
            raise ValueError('producer epoch must advance')
        self._producer_epoch_id = producer_epoch_id
        self._query_key = None
        self._last_stamp_ns = None
        self._next_track_id = 1
        self._next_candidate_id = 1
        self._tracks = {}

    @staticmethod
    def _validate_query_key(query_key):
        if (
                not isinstance(query_key, tuple) or len(query_key) != 2 or
                any(isinstance(value, bool) or not isinstance(value, int) or
                    value <= 0 for value in query_key)):
            raise ValueError('query key must contain positive integers')

    def _clear_for_new_epoch(self):
        self._producer_epoch_id += 1
        self._next_track_id = 1
        self._next_candidate_id = 1
        self._tracks = {}

    def _pair_score(self, track, detection, stamp_ns):
        if track.detection.label != detection.detection.label:
            return None
        predicted = _predicted_detection(track, stamp_ns)
        overlap = _iou(predicted, detection.detection)
        distance = _normalized_center_distance(
            predicted, detection.detection)
        center_score = max(0.0, 1.0 - distance)
        geometry = 0.65 * overlap + 0.35 * center_score
        appearance = _descriptor_cosine(
            track.descriptor, detection.descriptor)
        geometry_match = (
            overlap >= self._config.minimum_iou or
            distance <= self._config.maximum_normalized_center_distance)
        appearance_match = (
            appearance is not None and
            appearance >= self._config.minimum_appearance_similarity)
        if not geometry_match and not appearance_match:
            return None
        if appearance is None:
            return geometry
        # Geometry is useful for ordinary frame-to-frame motion, while DINOv3
        # is the stronger identity cue after a detector gap or a camera turn.
        return 0.25 * geometry + 0.75 * max(0.0, appearance)

    def update(self, stamp_ns, query_key, detections):
        self._validate_query_key(query_key)
        if (
                isinstance(stamp_ns, bool) or
                not isinstance(stamp_ns, int) or stamp_ns < 0):
            raise ValueError('source timestamp must be non-negative integer')
        detections = tuple(detections)
        if (
                len(detections) > self._config.maximum_tracks or
                any(not isinstance(value, DetectionInput)
                    for value in detections)):
            raise ValueError('detection input exceeds bounded contract')
        detections = tuple(sorted(detections, key=_detection_key))

        if self._query_key is not None and query_key != self._query_key:
            self._clear_for_new_epoch()
            self._last_stamp_ns = None
        elif (
                self._last_stamp_ns is not None and
                stamp_ns < self._last_stamp_ns):
            self._rollback_count += 1
            self._clear_for_new_epoch()
            self._last_stamp_ns = None
        elif self._last_stamp_ns is not None and stamp_ns == self._last_stamp_ns:
            raise ValueError('duplicate source timestamp')
        self._query_key = query_key

        pairs_by_detection = {}
        for detection_index, detection in enumerate(detections):
            pairs = []
            for track_id, track in sorted(self._tracks.items()):
                score = self._pair_score(track, detection, stamp_ns)
                if score is not None:
                    pairs.append((score, track_id, detection_index))
            pairs.sort(key=lambda value: (-value[0], value[1]))
            if (
                    len(pairs) >= 2 and
                    pairs[0][0] - pairs[1][0] <
                    self._config.ambiguity_margin):
                pairs = []
            pairs_by_detection[detection_index] = pairs

        all_pairs = [
            value
            for pairs in pairs_by_detection.values()
            for value in pairs
        ]
        all_pairs.sort(key=lambda value: (-value[0], value[1], value[2]))
        matched_tracks = set()
        matched_detections = set()
        assignments = {}
        for _score, track_id, detection_index in all_pairs:
            if (
                    track_id in matched_tracks or
                    detection_index in matched_detections):
                continue
            matched_tracks.add(track_id)
            matched_detections.add(detection_index)
            assignments[detection_index] = track_id

        next_tracks = {}
        for track_id, track in sorted(self._tracks.items()):
            if track_id in matched_tracks:
                continue
            missed = track.missed_frames + 1
            if missed <= self._config.maximum_missed_frames:
                next_tracks[track_id] = _Track(
                    track_id=track.track_id,
                    detection=track.detection,
                    descriptor=track.descriptor,
                    missed_frames=missed,
                    last_stamp_ns=track.last_stamp_ns,
                    box_velocity=track.box_velocity,
                )

        candidates = []
        for detection_index, detection in enumerate(detections):
            track_id = assignments.get(detection_index)
            if track_id is None:
                if len(next_tracks) >= self._config.maximum_tracks:
                    raise ValueError('camera track capacity is exhausted')
                track_id = self._next_track_id
                self._next_track_id += 1
            descriptor = detection.descriptor
            velocity = (0.0, 0.0, 0.0, 0.0)
            if descriptor is None and track_id in self._tracks:
                descriptor = self._tracks[track_id].descriptor
            if track_id in self._tracks:
                previous = self._tracks[track_id]
                delta_sec = float(
                    stamp_ns - previous.last_stamp_ns) / 1_000_000_000.0
                if delta_sec > 0.0:
                    current = detection.detection
                    old = previous.detection
                    velocity = (
                        (current.x1 - old.x1) / delta_sec,
                        (current.y1 - old.y1) / delta_sec,
                        (current.x2 - old.x2) / delta_sec,
                        (current.y2 - old.y2) / delta_sec,
                    )
            next_tracks[track_id] = _Track(
                track_id=track_id,
                detection=detection.detection,
                descriptor=descriptor,
                missed_frames=0,
                last_stamp_ns=stamp_ns,
                box_velocity=velocity,
            )
            candidates.append(CameraCandidate(
                candidate_id=self._next_candidate_id,
                camera_track_id=track_id,
                detection=detection.detection,
                descriptor=detection.descriptor,
            ))
            self._next_candidate_id += 1

        self._tracks = next_tracks
        self._last_stamp_ns = stamp_ns
        return CameraTrackingResult(
            producer_epoch_id=self._producer_epoch_id,
            rollback_count=self._rollback_count,
            candidates=tuple(candidates),
        )
