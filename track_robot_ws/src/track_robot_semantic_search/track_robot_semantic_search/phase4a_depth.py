"""Robust fixed-camera depth geometry for Phase 4A."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self):
        values = (self.fx, self.fy, self.cx, self.cy)
        if (
                not all(math.isfinite(value) for value in values)
                or self.fx <= 0.0
                or self.fy <= 0.0):
            raise ValueError('camera intrinsics are invalid')


class DepthEstimationError(ValueError):
    def __init__(self, reason, valid_samples=0):
        super().__init__(reason)
        self.reason = str(reason)
        self.valid_samples = int(valid_samples)


@dataclass(frozen=True)
class DepthEstimate:
    x: float
    y: float
    z: float
    depth_m: float
    quality: float
    valid_samples: int
    total_samples: int


def estimate_depth_point(
        depth,
        roi,
        intrinsics,
        minimum_samples=20,
        minimum_depth_m=0.2,
        maximum_depth_m=10.0,
        inner_fraction=0.5):
    """Estimate one optical-frame point from the inner part of a target ROI."""
    intrinsics.validate()
    image = np.asarray(depth)
    if image.ndim != 2:
        raise ValueError('depth image must be two-dimensional')
    if (
            minimum_samples <= 0
            or not 0.0 < inner_fraction <= 1.0
            or not 0.0 < minimum_depth_m < maximum_depth_m):
        raise ValueError('depth sampling configuration is invalid')
    x, y, width, height = (int(value) for value in roi)
    if width <= 0 or height <= 0:
        raise ValueError('ROI is empty')
    center_u = x + 0.5 * width
    center_v = y + 0.5 * height
    inner_width = max(1, int(round(width * inner_fraction)))
    inner_height = max(1, int(round(height * inner_fraction)))
    left = max(0, int(math.floor(center_u - 0.5 * inner_width)))
    top = max(0, int(math.floor(center_v - 0.5 * inner_height)))
    right = min(image.shape[1], left + inner_width)
    bottom = min(image.shape[0], top + inner_height)
    if right <= left or bottom <= top:
        raise ValueError('ROI lies outside depth image')
    sample = image[top:bottom, left:right].astype(np.float64, copy=False)
    finite_values = sample[np.isfinite(sample)]
    if finite_values.size == 0:
        raise DepthEstimationError('insufficient_depth_samples', 0)
    in_range = finite_values[
        (finite_values >= minimum_depth_m)
        & (finite_values <= maximum_depth_m)
    ]
    if in_range.size == 0:
        raise DepthEstimationError('depth_out_of_range', 0)
    if in_range.size < minimum_samples:
        raise DepthEstimationError(
            'insufficient_depth_samples', in_range.size)
    depth_m = float(np.median(in_range))
    # The robust median determines range; the target ROI center determines
    # the viewing ray.
    pixel_u = x + 0.5 * (width - 1)
    pixel_v = y + 0.5 * (height - 1)
    return DepthEstimate(
        x=(pixel_u - intrinsics.cx) * depth_m / intrinsics.fx,
        y=(pixel_v - intrinsics.cy) * depth_m / intrinsics.fy,
        z=depth_m,
        depth_m=depth_m,
        quality=float(in_range.size) / float(sample.size),
        valid_samples=int(in_range.size),
        total_samples=int(sample.size),
    )


def transform_point(point, translation, quaternion):
    """Apply a rigid transform represented by xyzw quaternion."""
    px, py, pz = (float(value) for value in point)
    tx, ty, tz = (float(value) for value in translation)
    qx, qy, qz, qw = (float(value) for value in quaternion)
    values = (px, py, pz, tx, ty, tz, qx, qy, qz, qw)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not all(math.isfinite(value) for value in values) or norm < 1e-12:
        raise ValueError('point transform or quaternion is invalid')
    qx, qy, qz, qw = (
        qx / norm, qy / norm, qz / norm, qw / norm)
    # Quaternion-vector rotation without constructing an intermediate matrix.
    uv_x = qy * pz - qz * py
    uv_y = qz * px - qx * pz
    uv_z = qx * py - qy * px
    uuv_x = qy * uv_z - qz * uv_y
    uuv_y = qz * uv_x - qx * uv_z
    uuv_z = qx * uv_y - qy * uv_x
    scale = 2.0 * qw
    return (
        px + scale * uv_x + 2.0 * uuv_x + tx,
        py + scale * uv_y + 2.0 * uuv_y + ty,
        pz + scale * uv_z + 2.0 * uuv_z + tz,
    )
