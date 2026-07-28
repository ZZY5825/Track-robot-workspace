"""ROS-independent coordinator for Phase 1R YOLO-World perception."""

import math
import time
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .camera_tracking import CameraCandidate, DetectionInput
from .query_transport import ActiveQuery


@dataclass(frozen=True)
class YoloWorldPerceptionResult:
    producer_epoch_id: int
    source_stamp_ns: int
    query_id: int
    query_version: int
    query_text: str
    candidates: Tuple[CameraCandidate, ...]
    task_descriptor: np.ndarray
    model_latency_ms: float
    appearance_latency_ms: float
    rollback_count: int
    appearance_available: bool
    appearance_reason: str


class YoloWorldPerceptionCore:
    def __init__(self, backend, dino_backend, tracker, clock_ns=None):
        for value, name in (
                (backend, 'backend'),
                (dino_backend, 'dino_backend'),
                (tracker, 'tracker')):
            if value is None:
                raise ValueError('{} is required'.format(name))
        self._backend = backend
        self._dino = dino_backend
        self._tracker = tracker
        self._clock_ns = clock_ns or time.perf_counter_ns
        self._query = None

    @staticmethod
    def _validate_query(query):
        if not isinstance(query, ActiveQuery):
            raise ValueError('query must be ActiveQuery')
        if query.query_id <= 0 or query.query_version <= 0:
            raise ValueError('query identifiers must be positive')
        if (
                len(query.query_text) > 160 or
                any(ord(value) < 0x20 or ord(value) > 0x7e
                    for value in query.query_text)):
            raise ValueError('query must be bounded printable ASCII')

    def accept_query(self, query):
        self._validate_query(query)
        self._query = query

    @staticmethod
    def _validate_image(image_bgr):
        if (
                not isinstance(image_bgr, np.ndarray) or
                image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or
                image_bgr.shape[0] <= 0 or image_bgr.shape[1] <= 0):
            raise ValueError('image_bgr must be non-empty H,W,3')

    def process(self, image_bgr, source_stamp_ns):
        self._validate_image(image_bgr)
        if (
                isinstance(source_stamp_ns, bool) or
                not isinstance(source_stamp_ns, int) or source_stamp_ns < 0):
            raise ValueError('source timestamp must be non-negative integer')
        if self._query is None:
            return None

        model_started = self._clock_ns()
        detections = tuple(self._backend.predict(
            image_bgr, self._query.query_text))
        model_finished = self._clock_ns()

        appearance_started = self._clock_ns()
        descriptors = tuple(self._dino.encode(
            image_bgr, detections[:3]))
        appearance_finished = self._clock_ns()
        if len(descriptors) > min(3, len(detections)):
            raise ValueError('DINO returned too many descriptors')

        inputs = []
        for index, detection in enumerate(detections):
            descriptor = descriptors[index] if index < len(descriptors) else None
            inputs.append(DetectionInput(
                detection=detection,
                descriptor=descriptor,
            ))
        tracking = self._tracker.update(
            source_stamp_ns,
            (self._query.query_id, self._query.query_version),
            tuple(inputs),
        )
        task_descriptor = np.asarray(
            self._backend.active_text_descriptor(), dtype=np.float32)
        if (
                task_descriptor.shape != (512,) or
                not np.isfinite(task_descriptor).all()):
            raise ValueError('YOLO-World task descriptor is invalid')
        norm = float(np.linalg.norm(task_descriptor))
        if not math.isfinite(norm) or abs(norm - 1.0) > 1e-4:
            raise ValueError('YOLO-World task descriptor is not normalized')
        task_descriptor = task_descriptor.copy()
        task_descriptor.setflags(write=False)

        return YoloWorldPerceptionResult(
            producer_epoch_id=tracking.producer_epoch_id,
            source_stamp_ns=source_stamp_ns,
            query_id=self._query.query_id,
            query_version=self._query.query_version,
            query_text=self._query.query_text,
            candidates=tracking.candidates,
            task_descriptor=task_descriptor,
            model_latency_ms=(
                float(model_finished - model_started) / 1_000_000.0),
            appearance_latency_ms=(
                float(appearance_finished - appearance_started) / 1_000_000.0),
            rollback_count=tracking.rollback_count,
            appearance_available=bool(self._dino.available),
            appearance_reason=str(self._dino.unavailable_reason),
        )
