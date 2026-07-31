import numpy as np

from track_robot_semantic_search.camera_tracking import (
    AppearanceDescriptor,
    CameraTrackManager,
    CameraTrackingConfig,
    DetectionInput,
)
from track_robot_semantic_search.yolo_world_backend import GroundedDetection


def _config():
    return CameraTrackingConfig(
        minimum_iou=0.20,
        maximum_normalized_center_distance=0.30,
        ambiguity_margin=0.05,
        maximum_missed_frames=1,
        maximum_tracks=8,
    )


def _input(
        x1, y1, x2, y2, score=0.8, descriptor=None,
        label='blue bottle'):
    return DetectionInput(
        detection=GroundedDetection(
            float(x1), float(y1), float(x2), float(y2),
            float(score), label),
        descriptor=descriptor,
    )


def _descriptor(values, encoder='dinov3', checkpoint='dino-a', version=1):
    array = np.asarray(values, dtype=np.float32)
    array /= np.linalg.norm(array)
    return AppearanceDescriptor(
        values=array,
        quality=0.9,
        encoder_id=encoder,
        checkpoint_id=checkpoint,
        version=version,
    )


def test_stable_track_ids_do_not_depend_on_detection_order():
    manager = CameraTrackManager(_config())
    first = manager.update(
        1_000, (10, 1), (
            _input(0, 0, 20, 20),
            _input(80, 0, 100, 20),
        ))

    second = manager.update(
        2_000, (10, 1), (
            _input(81, 0, 101, 20),
            _input(1, 0, 21, 20),
        ))

    first_by_left = {
        value.detection.x1: value.camera_track_id
        for value in first.candidates
    }
    second_by_side = {
        'left' if value.detection.x1 < 40 else 'right':
            value.camera_track_id
        for value in second.candidates
    }
    assert second_by_side['left'] == first_by_left[0.0]
    assert second_by_side['right'] == first_by_left[80.0]


def test_ambiguous_match_creates_a_new_track():
    manager = CameraTrackManager(_config())
    first = manager.update(
        1_000, (10, 1), (
            _input(0, 0, 20, 20),
            _input(4, 0, 24, 20),
        ))

    second = manager.update(
        2_000, (10, 1), (_input(2, 0, 22, 20),))

    old_ids = {value.camera_track_id for value in first.candidates}
    assert second.candidates[0].camera_track_id not in old_ids


def test_query_change_advances_epoch_and_clears_tracks():
    manager = CameraTrackManager(_config())
    first = manager.update(
        1_000, (10, 1), (_input(0, 0, 20, 20),))

    second = manager.update(
        2_000, (11, 1), (_input(0, 0, 20, 20),))

    assert second.producer_epoch_id == first.producer_epoch_id + 1
    assert second.candidates[0].camera_track_id == 1


def test_timestamp_rollback_advances_epoch_and_resets_tracks():
    manager = CameraTrackManager(_config())
    first = manager.update(
        2_000, (10, 1), (_input(0, 0, 20, 20),))

    second = manager.update(
        1_000, (10, 1), (_input(0, 0, 20, 20),))

    assert second.producer_epoch_id == first.producer_epoch_id + 1
    assert second.rollback_count == 1
    assert second.candidates[0].camera_track_id == 1


def test_track_expires_after_bounded_missed_frames():
    manager = CameraTrackManager(_config())
    first = manager.update(
        1_000, (10, 1), (_input(0, 0, 20, 20),))
    manager.update(2_000, (10, 1), ())
    manager.update(3_000, (10, 1), ())

    fourth = manager.update(
        4_000, (10, 1), (_input(0, 0, 20, 20),))

    assert fourth.candidates[0].camera_track_id != (
        first.candidates[0].camera_track_id)


def test_incompatible_descriptors_are_not_compared_or_coerced():
    manager = CameraTrackManager(_config())
    first = manager.update(
        1_000, (10, 1), (
            _input(
                0, 0, 20, 20,
                descriptor=_descriptor([1.0, 0.0], encoder='dinov3-a')),
        ))

    second = manager.update(
        2_000, (10, 1), (
            _input(
                18, 0, 38, 20,
                descriptor=_descriptor([1.0, 0.0], encoder='dinov3-b')),
        ))

    assert second.candidates[0].camera_track_id != (
        first.candidates[0].camera_track_id)


def test_candidate_ids_are_positive_and_monotonic_per_epoch():
    manager = CameraTrackManager(_config())

    first = manager.update(
        1_000, (10, 1), (_input(0, 0, 20, 20),))
    second = manager.update(
        2_000, (10, 1), (_input(1, 0, 21, 20),))

    assert first.candidates[0].candidate_id == 1
    assert second.candidates[0].candidate_id == 2


def test_constant_velocity_prediction_preserves_track_during_camera_motion():
    manager = CameraTrackManager(_config())
    first = manager.update(
        1_000_000_000, (10, 1), (_input(0, 0, 20, 20),))
    second = manager.update(
        2_000_000_000, (10, 1), (_input(10, 0, 30, 20),))
    third = manager.update(
        3_000_000_000, (10, 1), (_input(30, 0, 50, 20),))

    assert second.candidates[0].camera_track_id == (
        first.candidates[0].camera_track_id)
    assert third.candidates[0].camera_track_id == (
        first.candidates[0].camera_track_id)


def test_camera_track_never_crosses_detector_labels():
    manager = CameraTrackManager(_config())
    first = manager.update(
        1_000, (10, 1), (_input(0, 0, 20, 20),))
    second = manager.update(
        2_000, (10, 1), (
            _input(0, 0, 20, 20, label='green bag'),))

    assert second.candidates[0].camera_track_id != (
        first.candidates[0].camera_track_id)
