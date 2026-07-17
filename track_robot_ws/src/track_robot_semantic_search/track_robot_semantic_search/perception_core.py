from dataclasses import dataclass
import math
from typing import Any, List, Optional, Sequence

import numpy as np

from .query import CachedTextEncoder
from .region_scoring import RegionCandidate, score_regions
from .visual_candidates import (
    CandidateProposal,
    VisualCandidateResult,
    normalize_descriptor,
    pool_roi_descriptor,
)


@dataclass(frozen=True)
class PerceptionResult:
    stamp_ns: int
    query_id: int
    query_version: int
    observation_id: int
    image_width: int
    image_height: int
    image_encoder_id: str
    text_encoder_id: str
    checkpoint_id: str
    preprocessing_scale: float
    padding_left: int
    padding_top: int
    model_input_width: int
    model_input_height: int
    inference_ms: float
    regions: List[RegionCandidate]
    visual_candidates: List[VisualCandidateResult]
    normalized_query_text: str
    task_descriptor: np.ndarray


class PassivePerceptionCore:
    def __init__(
            self,
            aligned_encoder: Any,
            threshold: float = 0.25,
            threshold_mode: str = 'absolute',
            quantile: float = 0.90,
            min_area: int = 1,
            max_regions: int = 10,
            max_visual_candidates: int = 64):
        for attribute in (
                'encoder_id', 'checkpoint_id', 'encode_text',
                'encode_image_grid'):
            if not hasattr(aligned_encoder, attribute):
                raise TypeError(
                    'aligned encoder is missing {}'.format(attribute))
        self._encoder = aligned_encoder
        self._text_cache = CachedTextEncoder(
            aligned_encoder.encoder_id, aligned_encoder.encode_text)
        self._threshold = threshold
        self._threshold_mode = threshold_mode
        self._quantile = quantile
        self._min_area = min_area
        self._max_regions = max_regions
        self._max_visual_candidates = max(1, min(64, int(
            max_visual_candidates)))
        self._observation_id = 0
        self._visual_candidate_id = 0

    def _next_candidate_id(self) -> int:
        self._visual_candidate_id += 1
        return self._visual_candidate_id

    def start_producer_epoch(self) -> None:
        self._visual_candidate_id = 0

    def process(
            self,
            image_bgr: np.ndarray,
            query_text: str,
            query_id: int,
            query_version: int,
            stamp_ns: int) -> PerceptionResult:
        return self.process_frame(
            image_bgr=image_bgr,
            stamp_ns=stamp_ns,
            query_text=query_text,
            query_id=query_id,
            query_version=query_version,
        )

    def process_frame(
            self,
            image_bgr: np.ndarray,
            stamp_ns: int,
            query_text: Optional[str] = None,
            query_id: int = 0,
            query_version: int = 0,
            proposals: Sequence[CandidateProposal] = ()) -> PerceptionResult:
        image = np.asarray(image_bgr)
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError('image_bgr must be shaped H,W,3')
        for name, value in (
                ('query_id', query_id),
                ('query_version', query_version),
                ('stamp_ns', stamp_ns)):
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                raise ValueError('{} must be a non-negative integer'.format(name))
        image_encoding = self._encoder.encode_image_grid(image)
        query = None
        regions = []
        if query_text is not None:
            query = self._text_cache.encode(query_text, query_version)
            regions = score_regions(
                image_encoding.embeddings,
                query.vector,
                image_encoding.valid_patch_mask,
                image_encoding.geometry,
                threshold=self._threshold,
                threshold_mode=self._threshold_mode,
                quantile=self._quantile,
                min_area=self._min_area,
                max_regions=self._max_regions)

        visual_candidates = []
        for proposal in proposals:
            if len(visual_candidates) >= self._max_visual_candidates:
                break
            try:
                descriptor, quality = pool_roi_descriptor(
                    image_encoding.embeddings,
                    image_encoding.valid_patch_mask,
                    image_encoding.geometry,
                    proposal.roi)
            except ValueError:
                continue
            visual_candidates.append(VisualCandidateResult(
                visual_candidate_id=self._next_candidate_id(),
                roi=proposal.roi,
                proposal_source=proposal.proposal_source,
                detector_confidence=proposal.detector_confidence,
                language_score=0.0,
                localization_score=quality,
                descriptor=descriptor,
                descriptor_quality=quality,
                upstream_proposal_id_valid=True,
                upstream_producer_epoch_id=proposal.producer_epoch_id,
                upstream_proposal_id=proposal.proposal_id,
                mask_encoding=proposal.mask_encoding,
                compressed_mask=proposal.compressed_mask,
                labels=proposal.labels,
            ))
        for region in regions:
            if len(visual_candidates) >= self._max_visual_candidates:
                break
            visual_candidates.append(VisualCandidateResult(
                visual_candidate_id=self._next_candidate_id(),
                roi=region.roi,
                proposal_source=2,
                detector_confidence=0.0,
                language_score=region.score,
                localization_score=region.peak_score,
                descriptor=region.descriptor,
                descriptor_quality=region.descriptor_quality,
            ))
        self._observation_id += 1
        geometry = image_encoding.geometry
        empty_descriptor = np.asarray([], dtype=np.float32)
        empty_descriptor.setflags(write=False)
        return PerceptionResult(
            stamp_ns=stamp_ns,
            query_id=query_id,
            query_version=query_version,
            observation_id=self._observation_id,
            image_width=int(image.shape[1]),
            image_height=int(image.shape[0]),
            image_encoder_id=self._encoder.encoder_id,
            text_encoder_id=self._encoder.encoder_id,
            checkpoint_id=self._encoder.checkpoint_id,
            preprocessing_scale=geometry.scale,
            padding_left=geometry.padding_left,
            padding_top=geometry.padding_top,
            model_input_width=geometry.model_width,
            model_input_height=geometry.model_height,
            inference_ms=float(image_encoding.inference_ms),
            regions=regions,
            visual_candidates=visual_candidates,
            normalized_query_text=(
                query.normalized_text if query is not None else ''),
            task_descriptor=(
                normalize_descriptor(query.vector)
                if query is not None else empty_descriptor),
        )


class SourceTimestampScheduler:
    def __init__(self, target_rate_hz: float):
        if not isinstance(target_rate_hz, (int, float)) or (
                not math.isfinite(float(target_rate_hz)) or
                float(target_rate_hz) <= 0.0):
            raise ValueError('target_rate_hz must be finite and positive')
        self.period_ns = int(round(1_000_000_000.0 / float(target_rate_hz)))
        self.last_processed_ns = None
        self.rollback_count = 0

    def should_process(self, stamp_ns: int) -> bool:
        if not isinstance(stamp_ns, int) or isinstance(stamp_ns, bool) or (
                stamp_ns < 0):
            raise ValueError('stamp_ns must be a non-negative integer')
        if self.last_processed_ns is None:
            self.last_processed_ns = stamp_ns
            return True
        if stamp_ns < self.last_processed_ns:
            self.rollback_count += 1
            self.last_processed_ns = stamp_ns
            return False
        if stamp_ns - self.last_processed_ns < self.period_ns:
            return False
        self.last_processed_ns = stamp_ns
        return True
