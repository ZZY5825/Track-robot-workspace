from types import SimpleNamespace

import numpy as np

from track_robot_semantic_search.phase4a_depth import CameraIntrinsics
from track_robot_semantic_search.spatial_observation import (
    SpatialObservationConfig,
    spatialize_observation,
)


def observation():
    return SimpleNamespace(
        roi=SimpleNamespace(x_offset=0, y_offset=0, width=4, height=4),
        position_valid=False,
        position_frame_id='',
        position=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        position_covariance=[0.0] * 9,
        localization_epoch_id=0,
        pose_stamp_valid=False,
        pose_stamp=SimpleNamespace(sec=0, nanosec=0),
        tf_stamp_valid=False,
        tf_stamp=SimpleNamespace(sec=0, nanosec=0),
        geometry_confidence=0.0,
        evidence_flags=0,
    )


def test_spatialize_observation_sets_geometry_without_mutating_input():
    source = observation()
    depth = np.full((4, 4), 2.0, dtype=np.float32)
    result = spatialize_observation(
        source,
        depth=depth,
        intrinsics=CameraIntrinsics(
            fx=100.0, fy=100.0, cx=1.5, cy=1.5),
        translation=(0.0, 0.0, 0.0),
        quaternion=(0.0, 0.0, 0.0, 1.0),
        localization_epoch_id=7,
        depth_stamp_ns=2_500_000_123,
        config=SpatialObservationConfig(
            minimum_samples=4,
            inner_fraction=1.0,
        ),
    )

    assert result.accepted is True
    assert result.reason == 'matched_depth'
    assert result.valid_depth_samples == 16
    assert result.depth_quality == 1.0
    output = result.observation
    assert source.position_valid is False
    assert output.position_valid is True
    assert output.position_frame_id == 'base_link'
    assert output.position.x == 0.0
    assert output.position.y == 0.0
    assert output.position.z == 2.0
    assert output.position_covariance == [
        0.04, 0.0, 0.0,
        0.0, 0.04, 0.0,
        0.0, 0.0, 0.09,
    ]
    assert output.localization_epoch_id == 7
    assert output.pose_stamp_valid is True
    assert output.tf_stamp_valid is True
    assert output.pose_stamp.sec == 2
    assert output.pose_stamp.nanosec == 500_000_123
    assert output.tf_stamp.sec == 2
    assert output.tf_stamp.nanosec == 500_000_123
    assert output.geometry_confidence == 1.0
    assert output.evidence_flags == 5


def test_invalid_depth_preserves_the_original_observation():
    source = observation()
    result = spatialize_observation(
        source,
        depth=np.full((4, 4), np.nan, dtype=np.float32),
        intrinsics=CameraIntrinsics(
            fx=100.0, fy=100.0, cx=1.5, cy=1.5),
        translation=(0.0, 0.0, 0.0),
        quaternion=(0.0, 0.0, 0.0, 1.0),
        localization_epoch_id=7,
        depth_stamp_ns=2_500_000_123,
        config=SpatialObservationConfig(
            minimum_samples=4,
            inner_fraction=1.0,
        ),
    )

    assert result.accepted is False
    assert result.reason == 'insufficient_depth_samples'
    assert result.valid_depth_samples == 0
    assert result.depth_quality == 0.0
    output = result.observation
    assert output is not source
    assert output.position_valid is False
    assert output.position_frame_id == ''
    assert output.localization_epoch_id == 0
    assert output.evidence_flags == 0


def test_invalid_transformed_position_preserves_2d_observation():
    source = observation()
    result = spatialize_observation(
        source,
        depth=np.full((4, 4), 2.0, dtype=np.float32),
        intrinsics=CameraIntrinsics(
            fx=100.0, fy=100.0, cx=1.5, cy=1.5),
        translation=(0.0, 0.0, -2.0),
        quaternion=(0.0, 0.0, 0.0, 1.0),
        localization_epoch_id=7,
        depth_stamp_ns=2_500_000_123,
        config=SpatialObservationConfig(
            minimum_samples=4,
            inner_fraction=1.0,
        ),
    )

    assert result.accepted is False
    assert result.reason == 'invalid_transformed_position'
    assert result.valid_depth_samples == 16
    assert result.depth_quality == 1.0
    assert result.observation.position_valid is False


def test_invalid_intrinsics_are_not_reported_as_transform_failure():
    result = spatialize_observation(
        observation(),
        depth=np.full((4, 4), 2.0, dtype=np.float32),
        intrinsics=CameraIntrinsics(
            fx=0.0, fy=100.0, cx=1.5, cy=1.5),
        translation=(0.0, 0.0, 0.0),
        quaternion=(0.0, 0.0, 0.0, 1.0),
        localization_epoch_id=7,
        depth_stamp_ns=2_500_000_123,
        config=SpatialObservationConfig(
            minimum_samples=4,
            inner_fraction=1.0,
        ),
    )

    assert result.accepted is False
    assert result.reason == 'insufficient_depth_samples'
    assert result.observation.position_valid is False
