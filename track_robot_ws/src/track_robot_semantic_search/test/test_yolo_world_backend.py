from types import SimpleNamespace

import numpy as np
import pytest

from track_robot_semantic_search.yolo_world_backend import (
    GroundedDetection,
    YoloWorldBackend,
    load_yolo_world_dependencies,
    normalize_yolo_world_result,
)


class FakeValue:
    def __init__(self, value):
        self._value = value

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return self._value


class FakeCuda:
    def init(self):
        pass

    def memory_reserved(self, _device):
        return 0

    def reset_peak_memory_stats(self, _device):
        pass


def test_dependency_error_preserves_underlying_import_failure(
        tmp_path, monkeypatch):
    def fail_import(_name):
        raise ImportError('missing runtime dependency: pandas')

    monkeypatch.setattr(
        'track_robot_semantic_search.yolo_world_backend.'
        'importlib.import_module',
        fail_import,
    )

    with pytest.raises(
            RuntimeError,
            match='ImportError: missing runtime dependency: pandas'):
        load_yolo_world_dependencies(
            tmp_path / 'yolo',
            tmp_path / 'clip',
        )


def test_model_initialization_restores_original_torch_loader(tmp_path):
    runtime_path = tmp_path / 'yolo-runtime'
    clip_runtime_path = tmp_path / 'clip-runtime'
    runtime_path.mkdir()
    clip_runtime_path.mkdir()
    world_checkpoint = tmp_path / 'world.pt'
    clip_checkpoint = tmp_path / 'clip.pt'
    world_checkpoint.write_bytes(b'world')
    clip_checkpoint.write_bytes(b'clip')

    def original_torch_load(*_args, **_kwargs):
        return None

    def incompatible_ultralytics_load(*_args, **_kwargs):
        return None

    torch = SimpleNamespace(
        load=incompatible_ultralytics_load,
        cuda=FakeCuda(),
    )
    dependencies = SimpleNamespace(
        torch=torch,
        torch_load_original=original_torch_load,
        yolo_world_class=lambda *_args, **_kwargs: SimpleNamespace(),
    )

    YoloWorldBackend.from_local_model(
        runtime_path=runtime_path,
        clip_runtime_path=clip_runtime_path,
        world_checkpoint=world_checkpoint,
        clip_checkpoint=clip_checkpoint,
        dependencies=dependencies,
    )

    assert torch.load is original_torch_load


def test_normalize_yolo_world_result_returns_generic_detections():
    result = SimpleNamespace(
        boxes=SimpleNamespace(
            xyxy=FakeValue([[-10.0, 4.0, 30.0, 50.0]]),
            conf=FakeValue([0.75]),
            cls=FakeValue([0.0]),
        ),
        names={0: 'blue bottle'},
    )

    detections = normalize_yolo_world_result(
        result,
        query='blue bottle',
        width=100,
        height=60,
        max_detections=4,
    )

    assert detections == (
        GroundedDetection(
            x1=0.0,
            y1=4.0,
            x2=30.0,
            y2=50.0,
            score=0.75,
            label='blue bottle',
        ),
    )


def test_active_text_descriptor_is_flat_finite_unit_vector():
    values = [[[3.0, 4.0] + [0.0] * 510]]
    backend = object.__new__(YoloWorldBackend)
    backend.model = SimpleNamespace(
        model=SimpleNamespace(txt_feats=FakeValue(values)))

    descriptor = backend.active_text_descriptor()

    assert descriptor.shape == (512,)
    assert descriptor.dtype == np.float32
    assert np.linalg.norm(descriptor) == pytest.approx(1.0)
    assert descriptor[0] == pytest.approx(0.6)
    assert descriptor[1] == pytest.approx(0.8)


@pytest.mark.parametrize('values', [
    [[[1.0, 0.0]]],
    [[[float('nan')] + [0.0] * 511]],
    [[[0.0] * 512]],
])
def test_active_text_descriptor_rejects_invalid_model_features(values):
    backend = object.__new__(YoloWorldBackend)
    backend.model = SimpleNamespace(
        model=SimpleNamespace(txt_feats=FakeValue(values)))

    with pytest.raises(ValueError, match='text descriptor'):
        backend.active_text_descriptor()


def test_vocabulary_update_error_exposes_underlying_failure():
    def fail_set_classes(_classes):
        raise RuntimeError('CLIP text encoder is on the wrong device')

    backend = object.__new__(YoloWorldBackend)
    backend.model = SimpleNamespace(set_classes=fail_set_classes)
    backend._dependencies = SimpleNamespace(
        clip=SimpleNamespace(load=lambda *_args, **_kwargs: None),
    )
    backend._clip_checkpoint = 'clip.pt'
    backend._cuda_device = 'cuda:0'
    backend._active_query = None

    with pytest.raises(
            RuntimeError,
            match='CLIP text encoder is on the wrong device'):
        backend._set_query('green bottle')


def test_query_switch_restores_cached_clip_precision_before_encoding():
    class CachedClip:
        def __init__(self):
            self.dtype = 'float16'

        def float(self):
            self.dtype = 'float32'
            return self

    cached_clip = CachedClip()

    def set_classes(_classes):
        if cached_clip.dtype != 'float32':
            raise RuntimeError('expected scalar type Float but found Half')

    backend = object.__new__(YoloWorldBackend)
    backend.model = SimpleNamespace(
        model=SimpleNamespace(clip_model=cached_clip),
        set_classes=set_classes,
    )
    backend._dependencies = SimpleNamespace(
        clip=SimpleNamespace(load=lambda *_args, **_kwargs: None),
    )
    backend._clip_checkpoint = 'clip.pt'
    backend._cuda_device = 'cuda:0'
    backend._active_query = 'green bottle'

    backend._set_query('yellow cylinder')

    assert cached_clip.dtype == 'float32'
    assert backend._active_query == 'yellow cylinder'
