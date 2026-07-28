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
from tools.semantic_search_grounding.environment import EnvironmentProbe


def _probe(ready):
    report = {
        'schema_version': '1.0.0',
        'host_role': 'r0b_desktop_teacher',
        'architecture': 'x86_64' if ready else 'aarch64',
        'os': 'Ubuntu 22.04',
        'python': '3.11.9',
        'gpu': {
            'nvidia_smi_available': ready,
            'devices': ([{
                'name': 'NVIDIA RTX 4090',
                'memory_total_mib': 24564,
                'driver_version': '550.90',
            }] if ready else []),
        },
        'runtime': {
            'torch': {
                'available': ready,
                'version': '2.4.1' if ready else '',
                'cuda_available': ready,
            },
            'transformers': {
                'available': ready,
                'version': '4.57.0' if ready else '',
            },
            'pillow': {
                'available': ready,
                'version': '10.4.0' if ready else '',
            },
        },
        'model': {
            'model_dir_present': ready,
            'checkpoint_present': ready,
            'checkpoint_sha256': 'a' * 64 if ready else '',
        },
        'host_eligible': ready,
        'runtime_ready': ready,
        'reasons': [] if ready else ['architecture_not_x86_64'],
    }
    return EnvironmentProbe(MappingProxyType(report))


def _dataset():
    case = GroundingCase(
        case_id='case-1',
        split='test',
        image_path=Path('/data/image.png'),
        image_sha256='b' * 64,
        session_id='session-1',
        physical_object_id='object-1',
        query=normalize_grounding_query('blue container'),
        target_present=True,
        boxes=(),
        scenario_tags=(),
        label_review_status='human_verified',
    )
    return GroundingDataset(dataset_id='dataset-1', cases=(case,))


class FakeBackend:
    def predict(self, image_path, query):
        assert image_path == Path('/data/image.png')
        assert query == 'blue container'
        return (TeacherDetection(1, 2, 11, 22, 0.8, query),)

    @staticmethod
    def synchronize():
        return None

    @staticmethod
    def incremental_cuda_reserved_mib():
        return 64.0


def _predict_arguments(output):
    return [
        'predict',
        '--dataset', '/data/dataset.json',
        '--model-dir', '/models/grounding-dino-tiny',
        '--checkpoint-file', 'model.safetensors',
        '--model-revision', '0123456789abcdef',
        '--candidate-id', 'grounding-dino-tiny-box005-text005',
        '--licence', 'Apache-2.0',
        '--licence-approved',
        '--output', str(output),
    ]


def test_cli_import_does_not_require_model_or_dataset_runtime():
    script = """
import builtins
original_import = builtins.__import__
def guarded_import(name, *args, **kwargs):
    if name.split('.')[0] in {'cv2', 'PIL', 'torch', 'transformers'}:
        raise RuntimeError('eager heavy import: ' + name)
    return original_import(name, *args, **kwargs)
builtins.__import__ = guarded_import
import tools.semantic_search_grounding.cli
"""
    environment = dict(os.environ)
    environment['PYTHONPATH'] = (
        'src/track_robot_semantic_search:.')
    result = subprocess.run(
        [sys.executable, '-c', script],
        cwd=str(Path(__file__).parents[3]),
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr


def test_probe_writes_report_and_returns_two_when_not_ready(tmp_path):
    from tools.semantic_search_grounding.cli import run

    output = tmp_path / 'probe.json'
    code = run(
        ['probe', '--output', str(output)],
        probe_fn=lambda **_kwargs: _probe(False),
        stdout=io.StringIO(),
        stderr=io.StringIO(),
    )

    assert code == 2
    assert json.loads(output.read_text())['runtime_ready'] is False


def test_predict_rejects_jetson_before_backend_import(tmp_path):
    from tools.semantic_search_grounding.cli import run

    calls = []
    output = tmp_path / 'predictions.json'
    stderr = io.StringIO()
    code = run(
        _predict_arguments(output),
        probe_fn=lambda **_kwargs: _probe(False),
        dataset_loader=lambda _path: calls.append('dataset'),
        backend_factory=lambda **_kwargs: calls.append('backend'),
        stdout=io.StringIO(),
        stderr=stderr,
    )

    assert code == 2
    assert calls == []
    assert not output.exists()
    assert len(stderr.getvalue().splitlines()) == 1


def test_invalid_dataset_has_one_bounded_error_and_no_output(tmp_path):
    from tools.semantic_search_grounding.cli import run

    output = tmp_path / 'predictions.json'
    stderr = io.StringIO()

    def invalid(_path):
        raise ValueError(('bad dataset\n' * 200))

    code = run(
        _predict_arguments(output),
        probe_fn=lambda **_kwargs: _probe(True),
        dataset_loader=invalid,
        backend_factory=lambda **_kwargs: FakeBackend(),
        stdout=io.StringIO(),
        stderr=stderr,
    )

    assert code == 2
    assert not output.exists()
    assert len(stderr.getvalue().splitlines()) == 1
    assert len(stderr.getvalue()) <= 520


def test_backend_failure_preserves_existing_output(tmp_path):
    from tools.semantic_search_grounding.cli import run

    output = tmp_path / 'predictions.json'
    output.write_text('existing', encoding='utf-8')

    def fail(**_kwargs):
        raise RuntimeError('model failed')

    code = run(
        _predict_arguments(output),
        probe_fn=lambda **_kwargs: _probe(True),
        dataset_loader=lambda _path: _dataset(),
        backend_factory=fail,
        stdout=io.StringIO(),
        stderr=io.StringIO(),
    )

    assert code == 2
    assert output.read_text(encoding='utf-8') == 'existing'
    assert list(tmp_path.glob('*.tmp')) == []


def test_complete_fake_desktop_run_writes_valid_artifact_atomically(tmp_path):
    from tools.semantic_search_grounding.cli import run

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
    assert parsed.candidate_id == 'grounding-dino-tiny-box005-text005'
    assert parsed.model_identity['checkpoint_sha256'] == 'a' * 64
    assert parsed.release_evidence['licence_approved'] is True
    assert calls == [{
        'model_dir': Path('/models/grounding-dino-tiny'),
        'box_threshold': 0.05,
        'text_threshold': 0.05,
        'max_detections': 256,
    }]
    assert list(tmp_path.glob('*.tmp')) == []
