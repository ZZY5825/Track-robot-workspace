from types import SimpleNamespace

import numpy as np
import pytest

from track_robot_semantic_search.dino_crop_descriptors import (
    DinoCropConfig,
    DinoCropDescriptorBackend,
    extract_context_crops,
)
from track_robot_semantic_search.yolo_world_backend import GroundedDetection


def _detection(x1, y1, x2, y2, score):
    return GroundedDetection(
        float(x1), float(y1), float(x2), float(y2),
        float(score), 'blue bottle')


def test_extract_context_crops_clips_and_keeps_only_top_three():
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    detections = (
        _detection(0, 0, 20, 20, 0.9),
        _detection(50, 20, 80, 60, 0.8),
        _detection(100, 30, 140, 80, 0.7),
        _detection(150, 40, 190, 90, 0.6),
    )

    crops = extract_context_crops(
        image, detections, context_margin=0.25, maximum_crops=3)

    assert len(crops) == 3
    assert crops[0].shape == (25, 25, 3)
    assert crops[1].shape == (60, 46, 3)
    assert crops[2].shape == (76, 60, 3)


def test_backend_batches_crops_and_returns_unit_descriptors():
    calls = []

    def preprocess(crop, _input_size):
        calls.append(crop.shape)
        return np.asarray([crop.shape[0], crop.shape[1]], dtype=np.float32)

    def stack(values):
        return np.stack(values)

    def extract(_model, batch, _backend):
        assert batch.shape == (2, 2)
        return np.asarray([[3.0, 4.0], [0.0, 2.0]], dtype=np.float32)

    backend = DinoCropDescriptorBackend(
        model=object(),
        backend='fake',
        config=DinoCropConfig(
            input_size=224,
            context_margin=0.0,
            maximum_crops=3,
            encoder_id='dinov3:vits16plus',
            checkpoint_id='dino.pth',
            descriptor_version=1,
        ),
        preprocess_fn=preprocess,
        stack_fn=stack,
        extract_fn=extract,
    )
    image = np.zeros((100, 200, 3), dtype=np.uint8)

    descriptors = backend.encode(
        image,
        (
            _detection(0, 0, 20, 20, 0.9),
            _detection(50, 20, 80, 60, 0.8),
        ),
    )

    assert calls == [(20, 20, 3), (40, 30, 3)]
    assert len(descriptors) == 2
    assert descriptors[0].values == pytest.approx([0.6, 0.8])
    assert descriptors[1].values == pytest.approx([0.0, 1.0])
    assert all(value.encoder_id == 'dinov3:vits16plus'
               for value in descriptors)


def test_backend_moves_batched_crops_to_model_device_before_extracting():
    class FakeBatch:
        def __init__(self):
            self.device = 'cpu'

        def to(self, device):
            self.device = device
            return self

    batch = FakeBatch()

    def extract(_model, received, _backend):
        assert received is batch
        assert received.device == 'cuda:0'
        return np.asarray([[1.0, 0.0]], dtype=np.float32)

    backend = DinoCropDescriptorBackend(
        model=object(),
        backend='fake',
        config=DinoCropConfig(),
        preprocess_fn=lambda _crop, _size: object(),
        stack_fn=lambda _values: batch,
        extract_fn=extract,
        device='cuda:0',
    )

    descriptors = backend.encode(
        np.zeros((10, 10, 3), dtype=np.uint8),
        (_detection(0, 0, 5, 5, 0.9),),
    )

    assert len(descriptors) == 1
    assert batch.device == 'cuda:0'


@pytest.mark.parametrize('features', [
    [[float('nan'), 0.0]],
    [[0.0, 0.0]],
    [[[1.0, 0.0]]],
])
def test_backend_rejects_invalid_feature_output(features):
    backend = DinoCropDescriptorBackend(
        model=object(),
        backend='fake',
        config=DinoCropConfig(
            input_size=224,
            context_margin=0.0,
            maximum_crops=3,
            encoder_id='dinov3:vits16plus',
            checkpoint_id='dino.pth',
            descriptor_version=1,
        ),
        preprocess_fn=lambda _crop, _size: np.ones((2,), dtype=np.float32),
        stack_fn=lambda values: np.stack(values),
        extract_fn=lambda _model, _batch, _backend: np.asarray(
            features, dtype=np.float32),
    )

    with pytest.raises(ValueError, match='DINO descriptor'):
        backend.encode(
            np.zeros((10, 10, 3), dtype=np.uint8),
            (_detection(0, 0, 5, 5, 0.9),),
        )


def test_disabled_backend_returns_explicit_empty_evidence():
    backend = DinoCropDescriptorBackend.disabled('checkpoint unavailable')

    assert backend.available is False
    assert backend.unavailable_reason == 'checkpoint unavailable'
    assert backend.encode(
        np.zeros((10, 10, 3), dtype=np.uint8),
        (_detection(0, 0, 5, 5, 0.9),),
    ) == ()
