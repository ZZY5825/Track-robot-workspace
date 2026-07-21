from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class WindowEncoding:
    kind: str
    roi: Tuple[int, int, int, int]
    embedding: np.ndarray

    def __post_init__(self):
        if self.kind not in ('global', 'center'):
            raise ValueError('window encoding kind must be global or center')
        if len(self.roi) != 4 or any(
                not isinstance(value, int) or isinstance(value, bool)
                for value in self.roi):
            raise ValueError('window encoding roi must contain four integers')
        x, y, width, height = self.roi
        if x < 0 or y < 0 or width <= 0 or height <= 0:
            raise ValueError('window encoding roi must have positive area')
        vector = np.asarray(self.embedding, dtype=np.float32)
        if vector.ndim != 1 or vector.size == 0:
            raise ValueError('window embedding must be a non-empty vector')
        if not np.isfinite(vector).all():
            raise ValueError('window embedding must be finite')
        norm = float(np.linalg.norm(vector))
        if norm <= 1e-12:
            raise ValueError('window embedding norm must be positive')
        normalized = np.array(vector / norm, dtype=np.float32, copy=True)
        normalized.setflags(write=False)
        object.__setattr__(self, 'embedding', normalized)


def validate_window_strategy(
        strategy: str,
        grid_size: int,
        center_window_scale: float) -> str:
    selected = str(strategy).strip().lower()
    if selected not in ('grid_only', 'multiscale_v1'):
        raise ValueError(
            'window_strategy must be grid_only or multiscale_v1')
    if not isinstance(grid_size, int) or isinstance(grid_size, bool) or (
            grid_size <= 0):
        raise ValueError('grid_size must be a positive integer')
    scale = float(center_window_scale)
    if not math.isfinite(scale) or not 0.25 <= scale <= 1.0:
        raise ValueError(
            'center_window_scale must be finite and in [0.25, 1.0]')
    if selected == 'multiscale_v1' and grid_size != 2:
        raise ValueError('multiscale_v1 requires grid_size=2')
    return selected


def center_window_roi(
        width: int,
        height: int,
        scale: float) -> Tuple[int, int, int, int]:
    if not isinstance(width, int) or isinstance(width, bool) or width <= 0 or (
            not isinstance(height, int) or isinstance(height, bool) or
            height <= 0):
        raise ValueError('image dimensions must be positive integers')
    value = float(scale)
    if not math.isfinite(value) or not 0.25 <= value <= 1.0:
        raise ValueError('center window scale must be finite and in [0.25, 1.0]')
    crop_width = max(1, int(round(width * value)))
    crop_height = max(1, int(round(height * value)))
    return (
        (width - crop_width) // 2,
        (height - crop_height) // 2,
        crop_width,
        crop_height,
    )


def letterbox_to_square(image_rgb: np.ndarray) -> np.ndarray:
    source = np.asarray(image_rgb)
    if source.ndim != 3 or source.shape[2] != 3:
        raise ValueError('image must be shaped H,W,3')
    height, width = source.shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError('image dimensions must be positive')
    side = max(height, width)
    output = np.zeros((side, side, 3), dtype=source.dtype)
    left = (side - width) // 2
    top = (side - height) // 2
    output[top:top + height, left:left + width] = source
    return output
