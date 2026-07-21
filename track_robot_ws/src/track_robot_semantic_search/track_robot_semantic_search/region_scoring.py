from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np

from .multiscale_windows import WindowEncoding
from .visual_candidates import normalize_descriptor


@dataclass(frozen=True)
class ImageGeometry:
    source_width: int
    source_height: int
    model_width: int
    model_height: int
    resized_width: int
    resized_height: int
    scale: float
    padding_left: int
    padding_top: int
    patch_size: int

    def __post_init__(self):
        integer_fields = (
            self.source_width, self.source_height,
            self.model_width, self.model_height,
            self.resized_width, self.resized_height,
            self.patch_size)
        if any(value <= 0 for value in integer_fields):
            raise ValueError('image geometry dimensions must be positive')
        if not np.isfinite(self.scale) or self.scale <= 0.0:
            raise ValueError('image geometry scale must be finite and positive')
        if self.padding_left < 0 or self.padding_top < 0:
            raise ValueError('image geometry padding must be non-negative')
        if self.padding_left + self.resized_width > self.model_width or (
                self.padding_top + self.resized_height > self.model_height):
            raise ValueError('resized image must fit inside model canvas')

    def map_roi(
            self, x: float, y: float, width: float,
            height: float) -> Tuple[int, int, int, int]:
        valid_left = float(self.padding_left)
        valid_top = float(self.padding_top)
        valid_right = valid_left + float(self.resized_width)
        valid_bottom = valid_top + float(self.resized_height)
        left = max(valid_left, min(valid_right, float(x)))
        top = max(valid_top, min(valid_bottom, float(y)))
        right = max(valid_left, min(valid_right, float(x) + float(width)))
        bottom = max(valid_top, min(valid_bottom, float(y) + float(height)))
        if right <= left or bottom <= top:
            return 0, 0, 0, 0
        source_left = int(np.floor((left - valid_left) / self.scale))
        source_top = int(np.floor((top - valid_top) / self.scale))
        source_right = int(np.ceil((right - valid_left) / self.scale))
        source_bottom = int(np.ceil((bottom - valid_top) / self.scale))
        source_left = max(0, min(self.source_width, source_left))
        source_top = max(0, min(self.source_height, source_top))
        source_right = max(source_left, min(self.source_width, source_right))
        source_bottom = max(source_top, min(self.source_height, source_bottom))
        return (
            source_left,
            source_top,
            source_right - source_left,
            source_bottom - source_top)


@dataclass(frozen=True)
class RegionCandidate:
    x: int
    y: int
    width: int
    height: int
    token_area: int
    score: float
    peak_score: float
    descriptor: np.ndarray
    descriptor_quality: float

    @property
    def roi(self) -> Tuple[int, int, int, int]:
        return self.x, self.y, self.width, self.height


def _validate_inputs(
        image_embeddings: np.ndarray,
        text_embedding: np.ndarray,
        valid_patch_mask: np.ndarray,
        geometry: ImageGeometry) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    image = np.asarray(image_embeddings, dtype=np.float32)
    text = np.asarray(text_embedding, dtype=np.float32)
    valid = np.asarray(valid_patch_mask)
    if image.ndim != 3:
        raise ValueError('image embeddings must be shaped H,W,C')
    if text.ndim != 1 or image.shape[2] != text.shape[0]:
        raise ValueError('image and text must have the same embedding dimension')
    if text.size == 0:
        raise ValueError('embedding dimension must not be empty')
    if valid.shape != image.shape[:2] or valid.dtype != np.bool_:
        raise ValueError('valid patch mask must be boolean and shaped H,W')
    if not np.isfinite(image).all() or not np.isfinite(text).all():
        raise ValueError('image and text embeddings must be finite')
    expected_width = image.shape[1] * geometry.patch_size
    expected_height = image.shape[0] * geometry.patch_size
    if expected_width != geometry.model_width or (
            expected_height != geometry.model_height):
        raise ValueError('embedding grid does not match image geometry')
    return image, text, valid


def score_regions(
        image_embeddings: np.ndarray,
        text_embedding: np.ndarray,
        valid_patch_mask: np.ndarray,
        geometry: ImageGeometry,
        threshold: float = 0.25,
        threshold_mode: str = 'absolute',
        quantile: float = 0.90,
        min_area: int = 1,
        max_regions: int = 10) -> List[RegionCandidate]:
    image, text, valid = _validate_inputs(
        image_embeddings, text_embedding, valid_patch_mask, geometry)
    _validate_scoring_options(
        threshold, threshold_mode, quantile, min_area, max_regions)
    if not valid.any():
        return []

    scores = _cosine_scores(image, text)
    cutoff = _score_cutoff(
        scores[valid], threshold, threshold_mode, quantile)
    return _regions_from_scores(
        image, valid, scores, cutoff, geometry, min_area, max_regions)


def _validate_scoring_options(
        threshold: float,
        threshold_mode: str,
        quantile: float,
        min_area: int,
        max_regions: int) -> None:
    if threshold_mode not in ('absolute', 'quantile'):
        raise ValueError('threshold_mode must be absolute or quantile')
    if not np.isfinite(threshold):
        raise ValueError('threshold must be finite')
    if not np.isfinite(quantile) or not 0.0 <= quantile <= 1.0:
        raise ValueError('quantile must be between 0 and 1')
    if min_area <= 0 or max_regions <= 0:
        raise ValueError('min_area and max_regions must be positive')


def _cosine_scores(image: np.ndarray, text: np.ndarray) -> np.ndarray:
    text_norm = float(np.linalg.norm(text))
    if text_norm <= 1e-12:
        raise ValueError('text embedding norm must be positive')
    image_norms = np.linalg.norm(image, axis=2)
    denominator = np.maximum(image_norms * text_norm, 1e-12)
    return np.sum(image * text.reshape(1, 1, -1), axis=2) / denominator


def _score_cutoff(
        score_values: np.ndarray,
        threshold: float,
        threshold_mode: str,
        quantile: float) -> float:
    if threshold_mode == 'quantile':
        return float(np.quantile(score_values, quantile))
    return float(threshold)


def _regions_from_scores(
        image: np.ndarray,
        valid: np.ndarray,
        scores: np.ndarray,
        cutoff: float,
        geometry: ImageGeometry,
        min_area: int,
        max_regions: int) -> List[RegionCandidate]:
    active = np.logical_and(valid, scores >= cutoff).astype(np.uint8)
    if not active.any():
        return []

    count, labels, stats, _ = cv2.connectedComponentsWithStats(
        active, connectivity=4)
    regions = []
    for label in range(1, count):
        token_area = int(stats[label, cv2.CC_STAT_AREA])
        if token_area < min_area:
            continue
        x = int(stats[label, cv2.CC_STAT_LEFT])
        y = int(stats[label, cv2.CC_STAT_TOP])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        component_scores = scores[labels == label]
        component_mask = labels == label
        descriptor = normalize_descriptor(np.mean(
            image[component_mask], axis=0))
        descriptor_quality = float(token_area) / float(width * height)
        source_roi = geometry.map_roi(
            x * geometry.patch_size,
            y * geometry.patch_size,
            width * geometry.patch_size,
            height * geometry.patch_size)
        if source_roi[2] <= 0 or source_roi[3] <= 0:
            continue
        regions.append(RegionCandidate(
            x=source_roi[0],
            y=source_roi[1],
            width=source_roi[2],
            height=source_roi[3],
            token_area=token_area,
            score=float(np.mean(component_scores)),
            peak_score=float(np.max(component_scores)),
            descriptor=descriptor,
            descriptor_quality=descriptor_quality))
    regions.sort(key=lambda item: (
        -item.score,
        -item.peak_score,
        item.y,
        item.x,
        item.height,
        item.width))
    return regions[:max_regions]


def _region_sort_key(item: RegionCandidate):
    return (
        -item.score,
        -item.peak_score,
        item.y,
        item.x,
        item.height,
        item.width,
    )


def _overlap_ratios(first: RegionCandidate, second: RegionCandidate):
    left = max(first.x, second.x)
    top = max(first.y, second.y)
    right = min(first.x + first.width, second.x + second.width)
    bottom = min(first.y + first.height, second.y + second.height)
    intersection = max(0, right - left) * max(0, bottom - top)
    first_area = first.width * first.height
    second_area = second.width * second.height
    union = first_area + second_area - intersection
    iou = float(intersection) / float(union) if union > 0 else 0.0
    containment = float(intersection) / float(min(first_area, second_area))
    return iou, containment


def suppress_duplicate_regions(
        regions: List[RegionCandidate],
        duplicate_iou_threshold: float = 0.50,
        duplicate_containment_threshold: float = 0.80,
        max_regions: int = 10) -> List[RegionCandidate]:
    for name, value in (
            ('duplicate_iou_threshold', duplicate_iou_threshold),
            ('duplicate_containment_threshold',
             duplicate_containment_threshold)):
        if not np.isfinite(value) or not 0.0 <= value <= 1.0:
            raise ValueError('{} must be finite and in [0, 1]'.format(name))
    if max_regions <= 0:
        raise ValueError('max_regions must be positive')
    kept = []
    for candidate in sorted(regions, key=_region_sort_key):
        duplicate = False
        for previous in kept:
            iou, containment = _overlap_ratios(candidate, previous)
            if iou >= duplicate_iou_threshold or (
                    containment >= duplicate_containment_threshold):
                duplicate = True
                break
        if not duplicate:
            kept.append(candidate)
            if len(kept) >= max_regions:
                break
    return kept


def _window_score(window: WindowEncoding, text: np.ndarray) -> float:
    if window.embedding.shape[0] != text.shape[0]:
        raise ValueError('image and text must have the same embedding dimension')
    text_norm = float(np.linalg.norm(text))
    if text_norm <= 1e-12:
        raise ValueError('text embedding norm must be positive')
    return float(np.dot(window.embedding, text) / text_norm)


def _window_region(
        window: WindowEncoding,
        score: float) -> RegionCandidate:
    return RegionCandidate(
        x=window.roi[0],
        y=window.roi[1],
        width=window.roi[2],
        height=window.roi[3],
        token_area=1,
        score=score,
        peak_score=score,
        descriptor=window.embedding,
        descriptor_quality=1.0,
    )


def score_multiscale_regions(
        image_embeddings: np.ndarray,
        text_embedding: np.ndarray,
        valid_patch_mask: np.ndarray,
        geometry: ImageGeometry,
        extra_windows: Tuple[WindowEncoding, ...] = (),
        threshold: float = 0.25,
        threshold_mode: str = 'absolute',
        quantile: float = 0.90,
        min_area: int = 1,
        max_regions: int = 10,
        duplicate_iou_threshold: float = 0.50,
        duplicate_containment_threshold: float = 0.80,
        ) -> List[RegionCandidate]:
    image, text, valid = _validate_inputs(
        image_embeddings, text_embedding, valid_patch_mask, geometry)
    _validate_scoring_options(
        threshold, threshold_mode, quantile, min_area, max_regions)
    extras = tuple(extra_windows)
    if any(not isinstance(window, WindowEncoding) for window in extras):
        raise ValueError('extra_windows must contain WindowEncoding values')
    scores = _cosine_scores(image, text)
    extra_scores = np.asarray(
        [_window_score(window, text) for window in extras],
        dtype=np.float32)
    cutoff_values = [scores[valid]]
    if extra_scores.size:
        cutoff_values.append(extra_scores)
    combined = np.concatenate(cutoff_values)
    if combined.size == 0:
        return []
    cutoff = _score_cutoff(
        combined, threshold, threshold_mode, quantile)

    local_regions = _regions_from_scores(
        image, valid, scores, cutoff, geometry, min_area, max_regions)
    global_regions = []
    for window, score in zip(extras, extra_scores):
        if float(score) < cutoff:
            continue
        region = _window_region(window, float(score))
        if window.kind == 'global':
            global_regions.append(region)
        else:
            local_regions.append(region)
    local_regions = suppress_duplicate_regions(
        local_regions,
        duplicate_iou_threshold,
        duplicate_containment_threshold,
        max_regions=max_regions)
    if local_regions:
        return local_regions
    return sorted(global_regions, key=_region_sort_key)[:max_regions]
