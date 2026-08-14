import math

import numpy as np
import pytest

from track_robot_semantic_search.phase4a_depth import (
    CameraIntrinsics,
    DepthEstimationError,
    estimate_depth_point,
    transform_point,
)


def test_estimate_depth_point_uses_inner_roi_median_and_camera_intrinsics():
    depth = np.full((8, 10), np.nan, dtype=np.float32)
    depth[3:5, 4:6] = [[2.0, 2.2], [1.8, 2.0]]

    estimate = estimate_depth_point(
        depth,
        roi=(2, 1, 6, 6),
        intrinsics=CameraIntrinsics(fx=100.0, fy=100.0, cx=5.0, cy=4.0),
        minimum_samples=4,
    )

    assert estimate.depth_m == pytest.approx(2.0)
    assert estimate.x == pytest.approx(-0.01)
    assert estimate.y == pytest.approx(-0.01)
    assert estimate.z == pytest.approx(2.0)
    assert estimate.quality == pytest.approx(4.0 / 9.0)
    assert estimate.valid_samples == 4
    assert estimate.total_samples == 9


def test_estimate_depth_point_rejects_sparse_or_out_of_range_depth():
    depth = np.full((6, 6), np.nan, dtype=np.float32)
    depth[2, 2] = 20.0

    with pytest.raises(DepthEstimationError) as caught:
        estimate_depth_point(
            depth,
            roi=(1, 1, 4, 4),
            intrinsics=CameraIntrinsics(
                fx=100.0, fy=100.0, cx=3.0, cy=3.0),
            minimum_samples=2,
            maximum_depth_m=10.0,
        )
    assert caught.value.reason == 'depth_out_of_range'


def test_estimate_depth_point_reports_insufficient_depth_samples():
    with pytest.raises(DepthEstimationError) as caught:
        estimate_depth_point(
            np.full((4, 4), np.nan),
            roi=(0, 0, 4, 4),
            intrinsics=CameraIntrinsics(
                fx=100.0, fy=100.0, cx=1.5, cy=1.5),
            minimum_samples=4,
        )

    assert caught.value.reason == 'insufficient_depth_samples'
    assert caught.value.valid_samples == 0


def test_transform_point_applies_normalized_quaternion_and_translation():
    half_turn_z = (0.0, 0.0, 1.0, 0.0)
    point = transform_point(
        (1.0, 2.0, 3.0),
        translation=(0.5, -0.5, 1.0),
        quaternion=half_turn_z,
    )

    assert point == pytest.approx((-0.5, -2.5, 4.0))


def test_transform_point_rejects_invalid_quaternion():
    with pytest.raises(ValueError, match='quaternion'):
        transform_point(
            (1.0, 2.0, 3.0),
            translation=(0.0, 0.0, 0.0),
            quaternion=(0.0, 0.0, 0.0, math.nan),
        )
