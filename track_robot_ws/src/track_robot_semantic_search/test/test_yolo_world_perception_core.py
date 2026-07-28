import numpy as np

from track_robot_semantic_search.camera_tracking import (
    AppearanceDescriptor,
    CameraTrackManager,
    CameraTrackingConfig,
)
from track_robot_semantic_search.query_transport import ActiveQuery
from track_robot_semantic_search.yolo_world_backend import GroundedDetection
from track_robot_semantic_search.yolo_world_perception_core import (
    YoloWorldPerceptionCore,
)


class FakeWorld:
    def __init__(self, detections):
        self.detections = tuple(detections)
        self.calls = []

    def predict(self, image, query):
        self.calls.append((image.shape, query))
        return self.detections

    @staticmethod
    def active_text_descriptor():
        value = np.zeros((512,), dtype=np.float32)
        value[0] = 1.0
        return value


class FakeDino:
    available = True
    unavailable_reason = ''

    def encode(self, _image, detections):
        result = []
        for index, _ in enumerate(detections):
            values = np.zeros((4,), dtype=np.float32)
            values[index] = 1.0
            result.append(AppearanceDescriptor(
                values=values,
                quality=0.9,
                encoder_id='dinov3',
                checkpoint_id='dino.pth',
                version=1,
            ))
        return tuple(result)


def _detection(x1, score):
    return GroundedDetection(
        float(x1), 0.0, float(x1 + 20), 20.0,
        float(score), 'blue bottle')


def _core(detections):
    return YoloWorldPerceptionCore(
        backend=FakeWorld(detections),
        dino_backend=FakeDino(),
        tracker=CameraTrackManager(CameraTrackingConfig()),
    )


def test_core_requires_an_accepted_query_before_processing():
    core = _core((_detection(0, 0.9),))

    assert core.process(
        np.zeros((100, 200, 3), dtype=np.uint8), 1_000) is None


def test_core_correlates_query_boxes_tracks_and_text_descriptor():
    core = _core((_detection(0, 0.9), _detection(50, 0.8)))
    core.accept_query(ActiveQuery('blue bottle', 10, 1))

    result = core.process(
        np.zeros((100, 200, 3), dtype=np.uint8), 1_000)

    assert result.query_id == 10
    assert result.query_version == 1
    assert result.query_text == 'blue bottle'
    assert result.source_stamp_ns == 1_000
    assert len(result.candidates) == 2
    assert result.candidates[0].camera_track_id > 0
    assert result.candidates[0].descriptor.encoder_id == 'dinov3'
    assert result.task_descriptor.shape == (512,)
    assert np.linalg.norm(result.task_descriptor) == 1.0


def test_core_limits_dino_to_top_three_but_keeps_all_boxes():
    core = _core(tuple(_detection(index * 25, 0.9 - index * 0.1)
                       for index in range(4)))
    core.accept_query(ActiveQuery('blue bottle', 10, 1))

    result = core.process(
        np.zeros((100, 200, 3), dtype=np.uint8), 1_000)

    assert len(result.candidates) == 4
    assert sum(value.descriptor is not None
               for value in result.candidates) == 3


def test_query_change_advances_visual_producer_epoch():
    core = _core((_detection(0, 0.9),))
    core.accept_query(ActiveQuery('blue bottle', 10, 1))
    first = core.process(
        np.zeros((100, 200, 3), dtype=np.uint8), 1_000)
    core.accept_query(ActiveQuery('green cup', 11, 1))
    second = core.process(
        np.zeros((100, 200, 3), dtype=np.uint8), 2_000)

    assert second.producer_epoch_id == first.producer_epoch_id + 1


def test_empty_detection_is_a_valid_correlated_result():
    core = _core(())
    core.accept_query(ActiveQuery('blue bottle', 10, 1))

    result = core.process(
        np.zeros((100, 200, 3), dtype=np.uint8), 1_000)

    assert result.candidates == ()
    assert result.query_id == 10
