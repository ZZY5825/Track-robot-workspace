import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest


SETUP_PATH = Path(__file__).resolve().parents[1] / 'setup.py'


def _sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _write_evaluation_inputs(tmp_path, release_evidence=None):
    cases = [
        ('validation-positive', 'validation', True),
        ('validation-absent', 'validation', False),
        ('test-positive', 'test', True),
        ('test-absent', 'test', False),
    ]
    dataset_cases = []
    predictions = []
    for index, (case_id, split, target_present) in enumerate(cases):
        image_path = tmp_path / '{}.png'.format(case_id)
        assert cv2.imwrite(
            str(image_path),
            np.full((8, 8, 3), index, dtype=np.uint8))
        dataset_cases.append({
            'case_id': case_id,
            'split': split,
            'image_relative_path': image_path.name,
            'image_sha256': _sha256(image_path),
            'image_width': 8,
            'image_height': 8,
            'session_id': 'session-{}'.format(case_id),
            'physical_object_id': (
                'object-{}'.format(case_id) if target_present else ''),
            'query_text': 'tall blue container',
            'target_present': target_present,
            'ground_truth_boxes_xywh': (
                [[0.0, 0.0, 8.0, 8.0]] if target_present else []),
            'scenario_tags': ['cluttered'],
            'label_review_status': 'human_verified',
        })
        predictions.append({
            'case_id': case_id,
            'complete_path_ms': 100.0,
            'detections': ([{
                'box_xywh': [0.0, 0.0, 8.0, 8.0],
                'score': 0.8,
                'label': 'container',
            }] if target_present else []),
        })

    dataset_path = tmp_path / 'dataset.json'
    dataset_path.write_text(json.dumps({
        'schema_version': '1.0.0',
        'dataset_id': 'grounding-r0',
        'cases': dataset_cases,
    }), encoding='utf-8')
    predictions_path = tmp_path / 'predictions.json'
    predictions_path.write_text(json.dumps({
        'schema_version': '1.0.0',
        'dataset_id': 'grounding-r0',
        'candidate_id': 'candidate-a',
        'model_identity': {
            'implementation': 'fixture',
            'code_revision': 'revision',
            'checkpoint_id': 'model.pt',
            'checkpoint_sha256': 'b' * 64,
            'licence': 'Apache-2.0',
        },
        'platform': {
            'role': 'jetson_candidate',
            'hardware': 'Jetson AGX Orin',
            'os': 'Ubuntu 20.04',
            'python': '3.8.10',
            'pytorch': '1.13.0',
            'device': 'cuda',
        },
        'input_size': [1280, 1280],
        'incremental_cuda_reserved_mib': 1024.0,
        'release_evidence': release_evidence or {
            'runtime_available': True,
            'platform_compatible': True,
            'licence_approved': True,
        },
        'predictions': predictions,
    }), encoding='utf-8')
    return dataset_path, predictions_path


def _report(candidate_id='candidate-a', runtime_available=True):
    return {
        'dataset_id': 'grounding-r0',
        'dataset_checksum': 'a' * 64,
        'candidate_id': candidate_id,
        'validation_selection': {
            'status': 'selected',
            'threshold': 0.70,
        },
        'runtime_available': runtime_available,
        'platform_compatible': True,
        'licence_approved': True,
        'human_reviewed_test_labels': True,
        'test_metrics': {
            'top1_recall_iou_50': 0.90,
            'target_absent_false_accept_rate': 0.02,
            'median_accepted_positive_iou': 0.60,
        },
        'resources': {
            'p95_complete_path_ms': 100.0,
            'semantic_rate_hz': 10.0,
            'incremental_cuda_reserved_mib': 1024.0,
        },
    }


def _write_report(tmp_path, report, name='report.json'):
    path = tmp_path / name
    path.write_text(json.dumps(report), encoding='utf-8')
    return path


def test_evaluation_cli_writes_report_atomically_even_when_gates_fail(tmp_path):
    from track_robot_semantic_search.grounding_evaluation_cli import run

    dataset_path, predictions_path = _write_evaluation_inputs(
        tmp_path,
        release_evidence={
            'runtime_available': False,
            'platform_compatible': False,
            'licence_approved': False,
        })
    output_path = tmp_path / 'evaluation.json'

    exit_code = run(dataset_path, predictions_path, output_path)

    assert exit_code == 0
    report = json.loads(output_path.read_text(encoding='utf-8'))
    assert report['candidate_id'] == 'candidate-a'
    assert report['release_gates']['all_passed'] is False
    assert not list(tmp_path.glob('*.tmp'))


def test_selection_cli_writes_plain_unavailable_payload_atomically(
        tmp_path, monkeypatch):
    import track_robot_semantic_search.grounding_selection_cli as cli
    from track_robot_semantic_search.manifest import write_json_atomic

    report_path = _write_report(
        tmp_path, _report(runtime_available=False))
    output_path = tmp_path / 'selection.json'
    captured = {}

    def capture_write(path, payload):
        captured['payload'] = payload
        write_json_atomic(path, payload)

    monkeypatch.setattr(cli, 'write_json_atomic', capture_write)

    exit_code = cli.run([report_path], output_path)

    assert exit_code == 2
    assert type(captured['payload']) is dict
    assert type(captured['payload']['rejected']) is dict
    assert type(captured['payload']['rejected']['candidate-a']) is list
    assert type(captured['payload']['ranking']) is list
    assert json.loads(output_path.read_text(encoding='utf-8')) == {
        'ranking': [],
        'rejected': {'candidate-a': ['runtime_available']},
        'selected_candidate_id': None,
        'status': 'unavailable',
    }
    assert not list(tmp_path.glob('*.tmp'))


def test_selection_cli_returns_zero_only_when_candidate_is_selected(tmp_path):
    from track_robot_semantic_search.grounding_selection_cli import run

    report_path = _write_report(tmp_path, _report())
    output_path = tmp_path / 'selection.json'

    exit_code = run([report_path], output_path)

    assert exit_code == 0
    payload = json.loads(output_path.read_text(encoding='utf-8'))
    assert payload['status'] == 'selected'
    assert payload['selected_candidate_id'] == 'candidate-a'
    assert payload['ranking'] == ['candidate-a']


@pytest.mark.parametrize('module_name,input_kind', [
    ('grounding_evaluation_cli', 'evaluation'),
    ('grounding_selection_cli', 'selection'),
])
def test_malformed_input_returns_two_with_one_bounded_error_and_no_output(
        tmp_path, capsys, module_name, input_kind):
    module = __import__(
        'track_robot_semantic_search.{}'.format(module_name),
        fromlist=['run'])
    malformed = tmp_path / 'malformed.json'
    malformed.write_text('{invalid', encoding='utf-8')
    output_path = tmp_path / 'output.json'

    if input_kind == 'evaluation':
        exit_code = module.run(malformed, malformed, output_path)
    else:
        exit_code = module.run([malformed], output_path)

    captured = capsys.readouterr()
    assert exit_code == 2
    assert captured.out == ''
    assert len(captured.err.splitlines()) == 1
    assert 0 < len(captured.err.rstrip('\n')) <= 512
    assert not output_path.exists()
    assert not list(tmp_path.glob('*.tmp'))


def test_expected_error_text_is_sanitized_and_bounded(
        tmp_path, monkeypatch, capsys):
    import track_robot_semantic_search.grounding_evaluation_cli as cli

    def fail_to_load(_path):
        raise ValueError('first line\nsecond line ' + ('x' * 2000))

    monkeypatch.setattr(cli, 'load_grounding_dataset', fail_to_load)
    output_path = tmp_path / 'output.json'

    exit_code = cli.run(
        tmp_path / 'dataset.json',
        tmp_path / 'predictions.json',
        output_path)

    captured = capsys.readouterr()
    assert exit_code == 2
    assert len(captured.err.splitlines()) == 1
    assert len(captured.err.rstrip('\n')) <= 512
    assert not output_path.exists()


def test_programming_errors_are_not_swallowed(tmp_path, monkeypatch, capsys):
    import track_robot_semantic_search.grounding_evaluation_cli as cli

    def programming_error(_path):
        raise RuntimeError('bug')

    monkeypatch.setattr(cli, 'load_grounding_dataset', programming_error)

    with pytest.raises(RuntimeError, match='bug'):
        cli.run(
            tmp_path / 'dataset.json',
            tmp_path / 'predictions.json',
            tmp_path / 'output.json')

    captured = capsys.readouterr()
    assert captured.err == ''
    assert not (tmp_path / 'output.json').exists()


def test_parsers_expose_the_documented_arguments(tmp_path):
    from track_robot_semantic_search.grounding_evaluation_cli import (
        parser as evaluation_parser,
    )
    from track_robot_semantic_search.grounding_selection_cli import (
        parser as selection_parser,
    )

    evaluation = evaluation_parser().parse_args([
        '--dataset', str(tmp_path / 'dataset.json'),
        '--predictions', str(tmp_path / 'predictions.json'),
        '--output', str(tmp_path / 'report.json'),
    ])
    selection = selection_parser().parse_args([
        '--report', str(tmp_path / 'a.json'),
        '--report', str(tmp_path / 'b.json'),
        '--output', str(tmp_path / 'selection.json'),
    ])

    assert evaluation.dataset == tmp_path / 'dataset.json'
    assert evaluation.predictions == tmp_path / 'predictions.json'
    assert evaluation.output == tmp_path / 'report.json'
    assert selection.report == [
        tmp_path / 'a.json',
        tmp_path / 'b.json',
    ]
    assert selection.output == tmp_path / 'selection.json'


@pytest.mark.parametrize('module_name,arguments', [
    (
        'grounding_evaluation_cli',
        ['--dataset', 'dataset.json', '--predictions', 'predictions.json',
         '--output', 'report.json'],
    ),
    (
        'grounding_selection_cli',
        ['--report', 'report.json', '--output', 'selection.json'],
    ),
])
def test_main_exits_with_run_result(monkeypatch, module_name, arguments):
    module = __import__(
        'track_robot_semantic_search.{}'.format(module_name),
        fromlist=['main'])
    monkeypatch.setattr(module, 'run', lambda *unused: 2)

    with pytest.raises(SystemExit) as caught:
        module.main(arguments)

    assert caught.value.code == 2


def test_grounding_commands_are_packaged():
    source = SETUP_PATH.read_text(encoding='utf-8')

    assert (
        "'semantic_search_grounding_evaluate = '\n"
        "            'track_robot_semantic_search."
        "grounding_evaluation_cli:main'"
    ) in source
    assert (
        "'semantic_search_grounding_select = '\n"
        "            'track_robot_semantic_search."
        "grounding_selection_cli:main'"
    ) in source
