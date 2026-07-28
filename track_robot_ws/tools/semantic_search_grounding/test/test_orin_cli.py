import io
import json
import os
from pathlib import Path
import subprocess
import sys
from types import MappingProxyType

from track_robot_semantic_search.grounding_dataset import (
    GroundingCase,
    GroundingDataset,
)
from track_robot_semantic_search.grounding_predictions import (
    load_grounding_predictions,
)
from track_robot_semantic_search.grounding_query import (
    normalize_grounding_query,
)

from tools.semantic_search_grounding.contracts import TeacherDetection
from tools.semantic_search_grounding.orin_environment import (
    OrinEnvironmentProbe,
)


def _probe(ready):
    report = {
        'schema_version': '1.0.0',
        'host_role': 'r0c_orin_candidate',
        'architecture': 'aarch64',
        'l4t_release': '35',
        'os': 'Linux-5.10.104-tegra-aarch64',
        'python': '3.8.10',
        'paths': {
            'runtime_present': ready,
            'clip_runtime_present': ready,
        },
        'runtime': {
            'torch': {
                'available': ready,
                'version': '1.13.0a0+936e9305.nv22.11' if ready else '',
                'cuda_available': ready,
                'device_name': 'Orin' if ready else '',
                'origin': '/runtime/torch/__init__.py',
            },
            'ultralytics': {
                'available': ready,
                'version': '8.2.103' if ready else '',
                'yolo_world_available': ready,
                'origin': '/runtime/ultralytics/__init__.py',
            },
            'clip': {
                'available': ready,
                'version': '1.0' if ready else '',
                'origin': '/clip-runtime/clip/__init__.py',
            },
        },
        'models': {
            'world': {
                'present': ready,
                'sha256': 'a' * 64 if ready else '',
                'filename': 'yolov8s-worldv2.pt',
            },
            'clip': {
                'present': ready,
                'sha256': 'b' * 64 if ready else '',
                'filename': 'ViT-B-32.pt',
            },
            'composite_sha256': 'c' * 64 if ready else '',
        },
        'host_eligible': True,
        'runtime_ready': ready,
        'reasons': [] if ready else ['runtime_path_unavailable'],
    }
    return OrinEnvironmentProbe(MappingProxyType(report))


class FakeBackend:
    def __init__(self, fail=False):
        self.fail = fail
        self.calls = []
        self.synchronizations = 0

    def predict(self, image_path, query):
        self.calls.append((image_path, query))
        if self.fail:
            raise RuntimeError('inference failed')
        return (TeacherDetection(1, 2, 11, 22, 0.8, query),)

    def synchronize(self):
        self.synchronizations += 1

    @staticmethod
    def incremental_cuda_reserved_mib():
        return 64.0


def _dataset():
    case = GroundingCase(
        case_id='case-1',
        split='test',
        image_path=Path('/data/image.png'),
        image_sha256='d' * 64,
        session_id='session-1',
        physical_object_id='object-1',
        query=normalize_grounding_query('blue container'),
        target_present=True,
        boxes=(),
        scenario_tags=(),
        label_review_status='human_verified',
    )
    return GroundingDataset('dataset-1', (case,))


def _common():
    return [
        '--runtime-path', '/runtime',
        '--clip-runtime-path', '/clip-runtime',
        '--world-checkpoint', '/models/yolov8s-worldv2.pt',
        '--clip-checkpoint', '/models/ViT-B-32.pt',
    ]


def _predict_arguments(output):
    return [
        'predict',
        *_common(),
        '--dataset', '/data/dataset.json',
        '--candidate-id', 'yolov8s-worldv2-fp16-640-c005-i070',
        '--licence-approved',
        '--output', str(output),
    ]


def test_cli_import_has_no_eager_heavy_dependencies():
    script = """
import builtins
original_import = builtins.__import__
def guarded_import(name, *args, **kwargs):
    if name.split('.')[0] in {
        'cv2', 'PIL', 'torch', 'ultralytics', 'clip'
    }:
        raise RuntimeError('eager heavy import: ' + name)
    return original_import(name, *args, **kwargs)
builtins.__import__ = guarded_import
import tools.semantic_search_grounding.orin_cli
"""
    environment = dict(os.environ)
    environment['PYTHONPATH'] = 'src/track_robot_semantic_search:.'
    result = subprocess.run(
        [sys.executable, '-c', script],
        cwd=str(Path(__file__).parents[3]),
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_probe_writes_valid_unavailable_report(tmp_path):
    from tools.semantic_search_grounding.orin_cli import run

    output = tmp_path / 'probe.json'
    code = run(
        ['probe', *_common(), '--output', str(output)],
        probe_fn=lambda **_kwargs: _probe(False),
        stdout=io.StringIO(),
        stderr=io.StringIO(),
    )

    assert code == 2
    assert json.loads(output.read_text())['runtime_ready'] is False


def test_unavailable_predict_does_not_load_dataset_or_backend(tmp_path):
    from tools.semantic_search_grounding.orin_cli import run

    calls = []
    stderr = io.StringIO()
    code = run(
        _predict_arguments(tmp_path / 'predictions.json'),
        probe_fn=lambda **_kwargs: _probe(False),
        dataset_loader=lambda _path: calls.append('dataset'),
        backend_factory=lambda **_kwargs: calls.append('backend'),
        stdout=io.StringIO(),
        stderr=stderr,
    )

    assert code == 2
    assert calls == []
    assert len(stderr.getvalue().splitlines()) == 1


def test_smoke_writes_bounded_structured_result(tmp_path):
    from tools.semantic_search_grounding.orin_cli import run

    output = tmp_path / 'smoke.json'
    backend = FakeBackend()
    code = run(
        [
            'smoke',
            *_common(),
            '--image', '/data/image.png',
            '--query', 'Blue   Container',
            '--output', str(output),
        ],
        probe_fn=lambda **_kwargs: _probe(True),
        backend_factory=lambda **_kwargs: backend,
        image_probe=lambda _path: (640, 480),
        clock_ns=iter([0, 12_000_000]).__next__,
        stdout=io.StringIO(),
        stderr=io.StringIO(),
    )
    document = json.loads(output.read_text())

    assert code == 0
    assert document['schema_version'] == 'r0c_smoke/1.0.0'
    assert document['candidate_id'] == (
        'yolov8s-worldv2-fp16-640-c005-i070')
    assert document['query'] == 'blue container'
    assert document['image_width'] == 640
    assert document['image_height'] == 480
    assert document['complete_path_ms'] == 12.0
    assert document['incremental_cuda_reserved_mib'] == 64.0
    assert document['detections'] == [{
        'box_xywh': [1.0, 2.0, 10.0, 20.0],
        'score': 0.8,
        'label': 'blue container',
    }]
    assert backend.synchronizations == 2


def test_smoke_failure_preserves_existing_output(tmp_path):
    from tools.semantic_search_grounding.orin_cli import run

    output = tmp_path / 'smoke.json'
    output.write_text('existing', encoding='utf-8')
    code = run(
        [
            'smoke',
            *_common(),
            '--image', '/data/image.png',
            '--query', 'blue container',
            '--output', str(output),
        ],
        probe_fn=lambda **_kwargs: _probe(True),
        backend_factory=lambda **_kwargs: FakeBackend(fail=True),
        image_probe=lambda _path: (640, 480),
        stdout=io.StringIO(),
        stderr=io.StringIO(),
    )

    assert code == 2
    assert output.read_text() == 'existing'
    assert list(tmp_path.glob('*.tmp')) == []


def test_predict_writes_platform_compatible_r0a_artifact(tmp_path):
    from tools.semantic_search_grounding.orin_cli import run

    output = tmp_path / 'predictions.json'
    calls = []

    def backend_factory(**kwargs):
        calls.append(kwargs)
        return FakeBackend()

    code = run(
        _predict_arguments(output),
        probe_fn=lambda **_kwargs: _probe(True),
        dataset_loader=lambda _path: _dataset(),
        backend_factory=backend_factory,
        stdout=io.StringIO(),
        stderr=io.StringIO(),
    )
    parsed = load_grounding_predictions(output)

    assert code == 0
    assert set(parsed.predictions) == {'case-1'}
    assert parsed.release_evidence == {
        'runtime_available': True,
        'platform_compatible': True,
        'licence_approved': True,
    }
    assert parsed.model_identity['checkpoint_sha256'] == 'c' * 64
    assert parsed.model_identity['licence'] == 'AGPL-3.0'
    assert calls == [{
        'runtime_path': Path('/runtime'),
        'clip_runtime_path': Path('/clip-runtime'),
        'world_checkpoint': Path('/models/yolov8s-worldv2.pt'),
        'clip_checkpoint': Path('/models/ViT-B-32.pt'),
        'confidence_floor': 0.05,
        'iou_threshold': 0.70,
        'input_size': 640,
        'max_detections': 256,
        'device': 0,
        'half': True,
    }]
