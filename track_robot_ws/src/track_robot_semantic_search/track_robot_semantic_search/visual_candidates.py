from dataclasses import dataclass
import math
import secrets
from typing import Tuple

import numpy as np
from track_robot_interfaces.msg import VisualDescriptor


def normalize_descriptor(vector: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=np.float32)
    if value.ndim != 1 or value.size == 0:
        raise ValueError('descriptor must be a non-empty one-dimensional vector')
    if not np.isfinite(value).all():
        raise ValueError('descriptor must be finite')
    norm = float(np.linalg.norm(value))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError('descriptor norm must be positive')
    normalized = np.asarray(value / norm, dtype=np.float32)
    normalized.setflags(write=False)
    return normalized


def build_visual_descriptor_message(
        vector: np.ndarray,
        quality: float,
        encoder_id: str,
        checkpoint_id: str,
        version: int) -> VisualDescriptor:
    value = np.asarray(vector, dtype=np.float32)
    if value.ndim != 1 or value.size == 0:
        raise ValueError('visual descriptor must be one-dimensional and non-empty')
    if value.size > 1024:
        raise ValueError('visual descriptor exceeds the 1024 value bound')
    if not np.isfinite(value).all():
        raise ValueError('visual descriptor must be finite')
    norm = float(np.linalg.norm(value))
    if not math.isfinite(norm) or not math.isclose(
            norm, 1.0, rel_tol=1e-5, abs_tol=1e-5):
        raise ValueError('visual descriptor must be unit normalized')
    if not math.isfinite(float(quality)) or not 0.0 <= float(quality) <= 1.0:
        raise ValueError('visual descriptor quality must be between zero and one')
    if not isinstance(encoder_id, str) or not encoder_id or len(encoder_id) > 128:
        raise ValueError('encoder_id must contain 1 to 128 characters')
    if not isinstance(checkpoint_id, str) or len(checkpoint_id) > 128:
        raise ValueError('checkpoint_id must contain at most 128 characters')
    if not isinstance(version, int) or isinstance(version, bool) or not (
            0 <= version <= (1 << 32) - 1):
        raise ValueError('descriptor version must fit uint32')
    return VisualDescriptor(
        encoder_id=encoder_id,
        checkpoint_id=checkpoint_id,
        version=version,
        dimension=int(value.size),
        l2_normalized=True,
        quality=float(quality),
        values=[float(item) for item in value],
    )


@dataclass(frozen=True)
class CandidateLabel:
    label: str
    confidence: float
    provenance: str
    evidence_kind: int


@dataclass(frozen=True)
class CandidateProposal:
    producer_epoch_id: int
    proposal_id: int
    roi: Tuple[int, int, int, int]
    proposal_source: int
    detector_confidence: float
    mask_encoding: int = 0
    compressed_mask: bytes = b''
    labels: Tuple[CandidateLabel, ...] = ()

    def __post_init__(self):
        for name, value in (
                ('producer_epoch_id', self.producer_epoch_id),
                ('proposal_id', self.proposal_id),
                ('proposal_source', self.proposal_source)):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError('{} must be a non-negative integer'.format(name))
        if len(self.roi) != 4 or any(
                not isinstance(value, int) or isinstance(value, bool)
                for value in self.roi):
            raise ValueError('roi must contain four integers')
        if self.roi[2] <= 0 or self.roi[3] <= 0:
            raise ValueError('roi width and height must be positive')
        if not math.isfinite(self.detector_confidence) or not (
                0.0 <= self.detector_confidence <= 1.0):
            raise ValueError('detector confidence must be between zero and one')
        if len(self.compressed_mask) > 65536:
            raise ValueError('compressed mask exceeds Phase 2 bound')
        if len(self.labels) > 16:
            raise ValueError('proposal labels exceed Phase 2 bound')


@dataclass(frozen=True)
class VisualCandidateResult:
    visual_candidate_id: int
    roi: Tuple[int, int, int, int]
    proposal_source: int
    detector_confidence: float
    language_score: float
    localization_score: float
    descriptor: np.ndarray
    descriptor_quality: float
    upstream_proposal_id_valid: bool = False
    upstream_producer_epoch_id: int = 0
    upstream_proposal_id: int = 0
    mask_encoding: int = 0
    compressed_mask: bytes = b''
    labels: Tuple[CandidateLabel, ...] = ()


class ProducerIdentity:
    def __init__(self, configured_seed: int = 0):
        if not isinstance(configured_seed, int) or isinstance(
                configured_seed, bool) or configured_seed < 0:
            raise ValueError('producer epoch seed must be non-negative')
        self.epoch_id = configured_seed or secrets.randbits(64) or 1
        self._next_candidate = 1

    def next_candidate_id(self) -> int:
        value = self._next_candidate
        self._next_candidate += 1
        return value

    def advance_epoch(self) -> int:
        self.epoch_id = 1 if self.epoch_id >= (1 << 64) - 1 else (
            self.epoch_id + 1)
        self._next_candidate = 1
        return self.epoch_id


def pool_roi_descriptor(
        image_embeddings: np.ndarray,
        valid_patch_mask: np.ndarray,
        geometry,
        roi: Tuple[int, int, int, int]):
    embeddings = np.asarray(image_embeddings, dtype=np.float32)
    valid = np.asarray(valid_patch_mask)
    if embeddings.ndim != 3 or valid.shape != embeddings.shape[:2] or (
            valid.dtype != np.bool_):
        raise ValueError('embedding grid and valid mask are incompatible')
    if not np.isfinite(embeddings).all():
        raise ValueError('embedding grid must be finite')
    if len(roi) != 4 or roi[2] <= 0 or roi[3] <= 0:
        raise ValueError('roi width and height must be positive')

    source_left = max(0.0, min(float(geometry.source_width), float(roi[0])))
    source_top = max(0.0, min(float(geometry.source_height), float(roi[1])))
    source_right = max(
        source_left,
        min(float(geometry.source_width), float(roi[0] + roi[2])))
    source_bottom = max(
        source_top,
        min(float(geometry.source_height), float(roi[1] + roi[3])))
    if source_right <= source_left or source_bottom <= source_top:
        raise ValueError('roi does not intersect the source image')

    left = float(geometry.padding_left) + source_left * geometry.scale
    top = float(geometry.padding_top) + source_top * geometry.scale
    right = float(geometry.padding_left) + source_right * geometry.scale
    bottom = float(geometry.padding_top) + source_bottom * geometry.scale
    total_area = max(1e-12, (right - left) * (bottom - top))

    weighted = np.zeros(embeddings.shape[2], dtype=np.float64)
    used_area = 0.0
    patch = float(geometry.patch_size)
    for row in range(embeddings.shape[0]):
        for column in range(embeddings.shape[1]):
            if not valid[row, column]:
                continue
            cell_left = column * patch
            cell_top = row * patch
            cell_right = cell_left + patch
            cell_bottom = cell_top + patch
            overlap_width = max(
                0.0, min(right, cell_right) - max(left, cell_left))
            overlap_height = max(
                0.0, min(bottom, cell_bottom) - max(top, cell_top))
            overlap = overlap_width * overlap_height
            if overlap <= 0.0:
                continue
            weighted += embeddings[row, column].astype(np.float64) * overlap
            used_area += overlap
    if used_area <= 0.0:
        raise ValueError('roi contains no valid visual grid cells')
    descriptor = normalize_descriptor(weighted / used_area)
    quality = float(max(0.0, min(1.0, used_area / total_area)))
    return descriptor, quality
