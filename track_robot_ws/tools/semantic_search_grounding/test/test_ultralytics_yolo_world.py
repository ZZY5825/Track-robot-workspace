from pathlib import Path

import pytest

from tools.semantic_search_grounding.contracts import TeacherDetection
from tools.semantic_search_grounding.ultralytics_yolo_world import (
    UltralyticsYoloWorld,
    YoloWorldDependencies,
    normalize_yolo_world_result,
)


class FakeTensor:
    def __init__(self, value):
        self.value = value

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return self.value


class FakeBoxes:
    def __init__(self, boxes=None, scores=None, classes=None):
        self.xyxy = FakeTensor(
            [[-2.0, 5.0, 650.0, 490.0]] if boxes is None else boxes)
        self.conf = FakeTensor([0.75] if scores is None else scores)
        self.cls = FakeTensor([0.0] if classes is None else classes)


class FakeResult:
    def __init__(self, boxes=None, scores=None, classes=None):
        self.orig_shape = (480, 640)
        self.boxes = FakeBoxes(boxes, scores, classes)
        self.names = {0: 'blue container'}


class FakeClip:
    def __init__(self):
        self.calls = []

    def load(self, name, **kwargs):
        self.calls.append((name, kwargs))
        return ('local-clip-model', 'preprocess')


class FakeWorldModel:
    load_calls = []
    clip_module = None
    torch_module = None
    result = None

    def __init__(self, checkpoint, verbose=False):
        self.__class__.torch_module.load(checkpoint, map_location='cpu')
        self.__class__.load_calls.append((checkpoint, verbose))
        self.set_class_calls = []
        self.predict_calls = []

    def set_classes(self, classes):
        self.set_class_calls.append(list(classes))
        self.__class__.clip_module.load('ViT-B/32')

    def predict(self, **kwargs):
        self.predict_calls.append(kwargs)
        return [self.__class__.result or FakeResult()]


class FakeCuda:
    def __init__(self):
        self.synchronize_calls = []
        self.reset_peak_calls = []
        self.init_calls = 0
        self.initialized = False

    def init(self):
        self.init_calls += 1
        self.initialized = True

    def memory_reserved(self, _device):
        if not self.initialized:
            raise RuntimeError('did you call init?')
        return 100 * 1024 * 1024

    @staticmethod
    def max_memory_reserved(_device):
        return 164 * 1024 * 1024

    def reset_peak_memory_stats(self, device):
        self.reset_peak_calls.append(device)

    def synchronize(self, device):
        self.synchronize_calls.append(device)


class FakeTorch:
    def __init__(self):
        self.cuda = FakeCuda()
        self.load = self.patched_load
        self.original_load_calls = []

    @staticmethod
    def patched_load(*_args, **_kwargs):
        raise TypeError("'weights_only' is an invalid keyword argument")

    def original_load(self, *args, **kwargs):
        self.original_load_calls.append((args, kwargs))
        return {'model': 'loaded'}


def _dependencies():
    clip = FakeClip()
    torch = FakeTorch()
    FakeWorldModel.load_calls = []
    FakeWorldModel.clip_module = clip
    FakeWorldModel.torch_module = torch
    FakeWorldModel.result = None
    return YoloWorldDependencies(
        torch=torch,
        clip=clip,
        yolo_world_class=FakeWorldModel,
        torch_load_original=torch.original_load,
    )


def _paths(tmp_path):
    runtime = tmp_path / 'runtime'
    clip_runtime = tmp_path / 'clip-runtime'
    runtime.mkdir()
    clip_runtime.mkdir()
    world = tmp_path / 'yolov8s-worldv2.pt'
    clip = tmp_path / 'ViT-B-32.pt'
    world.write_bytes(b'world')
    clip.write_bytes(b'clip')
    return runtime, clip_runtime, world, clip


def test_normalizes_clamps_sorts_and_truncates():
    result = FakeResult(
        boxes=[
            [20.0, 20.0, 120.0, 100.0],
            [-4.0, 5.0, 90.0, 80.0],
            [30.0, 30.0, 40.0, 40.0],
        ],
        scores=[0.8, 0.9, 0.7],
        classes=[0.0, 0.0, 0.0],
    )

    detections = normalize_yolo_world_result(
        result, 'blue container', width=100, height=90, max_detections=2)

    assert detections == (
        TeacherDetection(0.0, 5.0, 90.0, 80.0, 0.9, 'blue container'),
        TeacherDetection(20.0, 20.0, 100.0, 90.0, 0.8,
                         'blue container'),
    )


def test_discards_collapsed_box_after_clamping():
    result = FakeResult(
        boxes=[[110.0, 5.0, 120.0, 20.0]],
        scores=[0.8],
        classes=[0.0],
    )
    assert normalize_yolo_world_result(
        result, 'blue container', 100, 90, 5) == ()


@pytest.mark.parametrize('boxes,scores,classes,names', [
    ([[0, 0, 1, 1]], [], [0.0], {0: 'blue container'}),
    ([[0, 0, float('nan'), 1]], [0.5], [0.0],
     {0: 'blue container'}),
    ([[0, 0, 1, 1]], [float('inf')], [0.0],
     {0: 'blue container'}),
    ([[0, 0, 1, 1]], [1.1], [0.0], {0: 'blue container'}),
    ([[0, 0, 1, 1]], [0.5], [0.5], {0: 'blue container'}),
    ([[0, 0, 1, 1]], [0.5], [0.0], {0: 'wrong label'}),
])
def test_rejects_malformed_model_result(boxes, scores, classes, names):
    result = FakeResult(boxes=boxes, scores=scores, classes=classes)
    result.names = names
    with pytest.raises(ValueError):
        normalize_yolo_world_result(
            result, 'blue container', 100, 90, 5)


def test_loads_only_local_models_and_forwards_exact_fp16_parameters(tmp_path):
    dependencies = _dependencies()
    runtime, clip_runtime, world, clip = _paths(tmp_path)
    original_clip_load = dependencies.clip.load

    backend = UltralyticsYoloWorld.from_local_model(
        runtime_path=runtime,
        clip_runtime_path=clip_runtime,
        world_checkpoint=world,
        clip_checkpoint=clip,
        confidence_floor=0.05,
        iou_threshold=0.70,
        input_size=640,
        max_detections=17,
        dependencies=dependencies,
    )
    detections = backend.predict(
        Path('/data/image.png'), 'blue container')

    assert FakeWorldModel.load_calls == [(str(world), False)]
    assert dependencies.torch.original_load_calls == [(
        (str(world),), {'map_location': 'cpu'})]
    assert dependencies.torch.load == dependencies.torch.patched_load
    assert backend.model.set_class_calls == [['blue container']]
    assert dependencies.clip.calls == [(
        str(clip),
        {'device': 'cuda:0', 'jit': False},
    )]
    assert dependencies.clip.load == original_clip_load
    assert backend.model.predict_calls == [{
        'source': str(Path('/data/image.png')),
        'imgsz': 640,
        'conf': 0.05,
        'iou': 0.70,
        'max_det': 17,
        'device': 0,
        'half': True,
        'augment': False,
        'verbose': False,
        'save': False,
        'stream': False,
    }]
    assert detections == (
        TeacherDetection(0.0, 5.0, 640.0, 480.0, 0.75,
                         'blue container'),
    )


def test_caches_vocabulary_until_query_changes(tmp_path):
    dependencies = _dependencies()
    runtime, clip_runtime, world, clip = _paths(tmp_path)
    backend = UltralyticsYoloWorld.from_local_model(
        runtime, clip_runtime, world, clip, dependencies=dependencies)

    backend.predict(Path('/data/1.png'), 'blue container')
    backend.predict(Path('/data/2.png'), 'blue container')
    FakeWorldModel.result = FakeResult()
    FakeWorldModel.result.names = {0: 'green cup'}
    backend.predict(Path('/data/3.png'), 'green cup')

    assert backend.model.set_class_calls == [
        ['blue container'], ['green cup']]
    assert len(dependencies.clip.calls) == 2


def test_restores_clip_loader_when_vocabulary_update_fails(tmp_path):
    dependencies = _dependencies()
    runtime, clip_runtime, world, clip = _paths(tmp_path)
    backend = UltralyticsYoloWorld.from_local_model(
        runtime, clip_runtime, world, clip, dependencies=dependencies)
    original_clip_load = dependencies.clip.load

    def fail(_classes):
        raise RuntimeError('text encoding failed')

    backend.model.set_classes = fail
    with pytest.raises(RuntimeError, match='vocabulary'):
        backend.predict(Path('/data/image.png'), 'blue container')

    assert dependencies.clip.load == original_clip_load


def test_synchronization_and_peak_memory_accounting(tmp_path):
    dependencies = _dependencies()
    runtime, clip_runtime, world, clip = _paths(tmp_path)
    backend = UltralyticsYoloWorld.from_local_model(
        runtime, clip_runtime, world, clip, dependencies=dependencies)

    backend.synchronize()

    assert dependencies.torch.cuda.reset_peak_calls == ['cuda:0']
    assert dependencies.torch.cuda.init_calls == 1
    assert dependencies.torch.cuda.synchronize_calls == ['cuda:0']
    assert backend.incremental_cuda_reserved_mib() == 64.0


@pytest.mark.parametrize(
    'confidence_floor,iou_threshold,input_size,max_detections,half', [
        (-0.1, 0.7, 640, 10, True),
        (0.05, 1.1, 640, 10, True),
        (0.05, 0.7, 0, 10, True),
        (0.05, 0.7, 640, 257, True),
        (0.05, 0.7, 640, 10, 1),
    ])
def test_rejects_invalid_configuration(
        tmp_path, confidence_floor, iou_threshold, input_size,
        max_detections, half):
    runtime, clip_runtime, world, clip = _paths(tmp_path)
    with pytest.raises(ValueError):
        UltralyticsYoloWorld.from_local_model(
            runtime,
            clip_runtime,
            world,
            clip,
            confidence_floor=confidence_floor,
            iou_threshold=iou_threshold,
            input_size=input_size,
            max_detections=max_detections,
            half=half,
            dependencies=_dependencies(),
        )
