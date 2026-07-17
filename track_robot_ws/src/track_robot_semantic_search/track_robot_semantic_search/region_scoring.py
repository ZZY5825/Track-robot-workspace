from dataclasses import dataclass
from typing import List, Tuple

import cv2
import numpy as np

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
    if threshold_mode not in ('absolute', 'quantile'):
        raise ValueError('threshold_mode must be absolute or quantile')
    if not np.isfinite(threshold):
        raise ValueError('threshold must be finite')
    if not np.isfinite(quantile) or not 0.0 <= quantile <= 1.0:
        raise ValueError('quantile must be between 0 and 1')
    if min_area <= 0 or max_regions <= 0:
        raise ValueError('min_area and max_regions must be positive')
    if not valid.any():
        return []

    text_norm = float(np.linalg.norm(text))
    if text_norm <= 1e-12:
        raise ValueError('text embedding norm must be positive')
    image_norms = np.linalg.norm(image, axis=2)
    denominator = np.maximum(image_norms * text_norm, 1e-12)
    scores = np.sum(image * text.reshape(1, 1, -1), axis=2) / denominator
    cutoff = float(threshold)
    if threshold_mode == 'quantile':
        cutoff = float(np.quantile(scores[valid], quantile))
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
