from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.semantic_search_grounding.contracts import TeacherDetection
from tools.semantic_search_grounding.huggingface_grounding_dino import (
    HuggingFaceDependencies,
    HuggingFaceGroundingDino,
    load_huggingface_dependencies,
    normalize_detection_result,
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


class FakeInputs(dict):
    input_ids = 'input-ids'

    def __init__(self):
        super().__init__(pixel_values='pixels')
        self.device = None

    def to(self, device):
        self.device = device
        return self


class FakeImage:
    size = (640, 480)

    def __init__(self):
        self.convert_mode = None

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False

    def convert(self, mode):
        self.convert_mode = mode
        return self


class FakeImageModule:
    def __init__(self):
        self.opened = []
        self.image = FakeImage()

    def open(self, path):
        self.opened.append(path)
        return self.image


class FakeProcessor:
    load_calls = []

    @classmethod
    def from_pretrained(cls, model_dir, **kwargs):
        cls.load_calls.append((model_dir, kwargs))
        return cls()

    def __init__(self):
        self.calls = []
        self.post_calls = []
        self.inputs = FakeInputs()

    def __call__(self, **kwargs):
        self.calls.append(kwargs)
        return self.inputs

    def post_process_grounded_object_detection(self, outputs, input_ids,
                                               **kwargs):
        self.post_calls.append((outputs, input_ids, kwargs))
        return [{
            'boxes': FakeTensor([[-5.0, 20.0, 650.0, 500.0]]),
            'scores': FakeTensor([0.75]),
            'labels': ['blue container'],
        }]


class FakeModel:
    load_calls = []

    @classmethod
    def from_pretrained(cls, model_dir, **kwargs):
        cls.load_calls.append((model_dir, kwargs))
        return cls()

    def __init__(self):
        self.device = None
        self.eval_called = False
        self.calls = []

    def to(self, device):
        self.device = device
        return self

    def eval(self):
        self.eval_called = True
        return self

    def __call__(self, **kwargs):
        self.calls.append(kwargs)
        return 'outputs'


class FakeCuda:
    def __init__(self):
        self.reserved = [100 * 1024 * 1024, 164 * 1024 * 1024]
        self.synchronize_calls = []
        self.reset_peak_calls = []

    def memory_reserved(self, device):
        return self.reserved.pop(0) if len(self.reserved) > 1 else self.reserved[0]

    def reset_peak_memory_stats(self, device):
        self.reset_peak_calls.append(device)

    def max_memory_reserved(self, device):
        return self.reserved[-1]

    def synchronize(self, device):
        self.synchronize_calls.append(device)


class FakeNoGrad:
    def __enter__(self):
        return None

    def __exit__(self, *_args):
        return False


class FakeTorch:
    def __init__(self):
        self.cuda = FakeCuda()

    @staticmethod
    def no_grad():
        return FakeNoGrad()


def _dependencies():
    FakeProcessor.load_calls = []
    FakeModel.load_calls = []
    return HuggingFaceDependencies(
        torch=FakeTorch(),
        image_module=FakeImageModule(),
        processor_class=FakeProcessor,
        model_class=FakeModel,
    )


def test_normalizes_clamps_sorts_and_truncates_detections():
    result = {
        'boxes': FakeTensor([
            [20.0, 20.0, 120.0, 100.0],
            [-4.0, 5.0, 90.0, 80.0],
            [30.0, 30.0, 40.0, 40.0],
        ]),
        'scores': FakeTensor([0.8, 0.9, 0.7]),
        'labels': ['second', 'first', 'discarded'],
    }

    detections = normalize_detection_result(
        result, width=100, height=90, max_detections=2)

    assert detections == (
        TeacherDetection(0.0, 5.0, 90.0, 80.0, 0.9, 'first'),
        TeacherDetection(20.0, 20.0, 100.0, 90.0, 0.8, 'second'),
    )


def test_normalization_discards_collapsed_boxes_after_clamping():
    result = {
        'boxes': [[110.0, 5.0, 120.0, 20.0]],
        'scores': [0.8],
        'labels': ['outside'],
    }
    assert normalize_detection_result(
        result, width=100, height=90, max_detections=2) == ()


@pytest.mark.parametrize('result', [
    {'boxes': [[0, 0, 1, 1]], 'scores': [], 'labels': ['x']},
    {'boxes': [[0, 0, float('nan'), 1]], 'scores': [0.5], 'labels': ['x']},
    {'boxes': [[0, 0, 1, 1]], 'scores': [float('inf')], 'labels': ['x']},
    {'boxes': [[0, 0, 1, 1]], 'scores': [1.1], 'labels': ['x']},
    {'boxes': [[0, 0, 1, 1]], 'scores': [0.5], 'labels': ['']},
])
def test_normalization_rejects_malformed_results(result):
    with pytest.raises(ValueError):
        normalize_detection_result(
            result, width=100, height=90, max_detections=2)


def test_local_model_loading_and_prediction_are_strict(tmp_path):
    dependencies = _dependencies()
    model_dir = tmp_path / 'model'
    model_dir.mkdir()

    backend = HuggingFaceGroundingDino.from_local_model(
        model_dir=model_dir,
        box_threshold=0.05,
        text_threshold=0.06,
        max_detections=17,
        dependencies=dependencies,
    )
    detections = backend.predict(Path('/data/image.png'), 'blue container')

    expected_load = [(str(model_dir), {'local_files_only': True})]
    assert FakeProcessor.load_calls == expected_load
    assert FakeModel.load_calls == expected_load
    assert dependencies.image_module.opened == [Path('/data/image.png')]
    assert dependencies.image_module.image.convert_mode == 'RGB'
    assert backend.processor.calls == [{
        'images': dependencies.image_module.image,
        'text': [['blue container']],
        'return_tensors': 'pt',
    }]
    assert backend.processor.inputs.device == 'cuda'
    assert backend.model.device == 'cuda'
    assert backend.model.eval_called is True
    assert backend.model.calls == [{'pixel_values': 'pixels'}]
    assert backend.processor.post_calls == [(
        'outputs',
        'input-ids',
        {
            'threshold': 0.05,
            'text_threshold': 0.06,
            'target_sizes': [(480, 640)],
        },
    )]
    assert detections == (
        TeacherDetection(0.0, 20.0, 640.0, 480.0, 0.75,
                         'blue container'),
    )


def test_synchronization_and_incremental_memory_accounting(tmp_path):
    dependencies = _dependencies()
    model_dir = tmp_path / 'model'
    model_dir.mkdir()
    backend = HuggingFaceGroundingDino.from_local_model(
        model_dir=model_dir,
        dependencies=dependencies,
    )

    backend.synchronize()

    assert dependencies.torch.cuda.synchronize_calls == ['cuda']
    assert dependencies.torch.cuda.reset_peak_calls == ['cuda']
    assert backend.incremental_cuda_reserved_mib() == 64.0


def test_missing_dependencies_have_bounded_runtime_error(monkeypatch):
    def missing(_name):
        raise ImportError('secret path and a very long diagnostic')

    monkeypatch.setattr(
        'tools.semantic_search_grounding.huggingface_grounding_dino.'
        'importlib.import_module',
        missing,
    )
    with pytest.raises(
            RuntimeError,
            match='Grounding DINO runtime dependencies are unavailable'):
        load_huggingface_dependencies()


@pytest.mark.parametrize('box_threshold,text_threshold,max_detections', [
    (-0.1, 0.05, 10),
    (0.05, 1.1, 10),
    (0.05, 0.05, 0),
    (0.05, 0.05, 257),
])
def test_adapter_rejects_invalid_configuration(
        tmp_path, box_threshold, text_threshold, max_detections):
    model_dir = tmp_path / 'model'
    model_dir.mkdir()
    with pytest.raises(ValueError):
        HuggingFaceGroundingDino.from_local_model(
            model_dir=model_dir,
            box_threshold=box_threshold,
            text_threshold=text_threshold,
            max_detections=max_detections,
            dependencies=_dependencies(),
        )
