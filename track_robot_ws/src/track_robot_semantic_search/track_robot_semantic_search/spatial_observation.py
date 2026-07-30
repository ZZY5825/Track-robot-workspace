"""Pure helpers for adding registered stereo depth to semantic observations."""

import copy
from dataclasses import dataclass
import math

from .phase4a_depth import estimate_depth_point, transform_point


@dataclass(frozen=True)
class SpatialObservationConfig:
    frame_id: str = 'base_link'
    minimum_samples: int = 20
    minimum_depth_m: float = 0.3
    maximum_depth_m: float = 8.0
    inner_fraction: float = 0.5
    covariance_xy: float = 0.04
    covariance_z: float = 0.09

    def validate(self):
        if not self.frame_id:
            raise ValueError('frame_id is required')
        if self.minimum_samples <= 0:
            raise ValueError('minimum_samples must be positive')
        if not 0.0 < self.minimum_depth_m < self.maximum_depth_m:
            raise ValueError('depth range is invalid')
        if not 0.0 < self.inner_fraction <= 1.0:
            raise ValueError('inner_fraction must be in (0, 1]')
        if (
                not math.isfinite(self.covariance_xy)
                or not math.isfinite(self.covariance_z)
                or self.covariance_xy < 0.0
                or self.covariance_z < 0.0):
            raise ValueError('position covariance must be finite and nonnegative')


def _set_stamp(stamp, stamp_ns):
    stamp.sec = int(stamp_ns // 1_000_000_000)
    stamp.nanosec = int(stamp_ns % 1_000_000_000)


def spatialize_observation(
        observation,
        *,
        depth,
        intrinsics,
        translation,
        quaternion,
        localization_epoch_id,
        depth_stamp_ns,
        config=SpatialObservationConfig()):
    """Return a copied observation with metric stereo geometry when valid."""
    output = copy.deepcopy(observation)
    try:
        config.validate()
        if int(localization_epoch_id) <= 0 or int(depth_stamp_ns) <= 0:
            raise ValueError('localization epoch and depth stamp are required')
        roi = observation.roi
        estimate = estimate_depth_point(
            depth,
            roi=(
                roi.x_offset,
                roi.y_offset,
                roi.width,
                roi.height,
            ),
            intrinsics=intrinsics,
            minimum_samples=config.minimum_samples,
            minimum_depth_m=config.minimum_depth_m,
            maximum_depth_m=config.maximum_depth_m,
            inner_fraction=config.inner_fraction,
        )
        x, y, z = transform_point(
            (estimate.x, estimate.y, estimate.z),
            translation=translation,
            quaternion=quaternion,
        )
        if not all(math.isfinite(value) for value in (x, y, z)):
            raise ValueError('transformed position is non-finite')
    except (AttributeError, TypeError, ValueError):
        return output, False

    output.position_valid = True
    output.position_frame_id = config.frame_id
    output.position.x = x
    output.position.y = y
    output.position.z = z
    output.position_covariance = [
        config.covariance_xy, 0.0, 0.0,
        0.0, config.covariance_xy, 0.0,
        0.0, 0.0, config.covariance_z,
    ]
    output.localization_epoch_id = int(localization_epoch_id)
    output.pose_stamp_valid = True
    output.tf_stamp_valid = True
    _set_stamp(output.pose_stamp, int(depth_stamp_ns))
    _set_stamp(output.tf_stamp, int(depth_stamp_ns))
    output.geometry_confidence = float(estimate.quality)
    output.evidence_flags = int(output.evidence_flags) | 1 | 4
    return output, True
