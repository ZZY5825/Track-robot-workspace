import math
import json
from pathlib import Path

import pytest

from track_robot_semantic_search.model_selection import (
    select_candidate,
    validate_benchmark,
)
from track_robot_semantic_search.model_selection_cli import run


def candidate(candidate_id, recall, latency, **overrides):
    value = {
        'candidate_id': candidate_id,
        'available': True,
        'python_compatible': True,
        'licence_approved': True,
        'memory_pass': True,
        'latency_pass': True,
        'phrase_region_recall': recall,
        'p95_latency_ms': latency,
    }
    value.update(overrides)
    return value


def test_selection_prefers_highest_recall_that_passes_all_gates():
    result = select_candidate([
        candidate('fast', 0.80, 40.0),
        candidate('accurate', 0.91, 100.0),
        candidate('invalid-best', 0.99, 30.0, licence_approved=False),
    ])

    assert result.status == 'selected'
    assert result.selected_candidate_id == 'accurate'
    assert result.rejected['invalid-best'] == ['licence_approved']


def test_selection_uses_latency_then_id_for_deterministic_ties():
    result = select_candidate([
        candidate('z-model', 0.9, 50.0),
        candidate('b-model', 0.9, 40.0),
        candidate('a-model', 0.9, 40.0),
    ])

    assert result.selected_candidate_id == 'a-model'


def test_unlabelled_compatible_candidate_is_selected_provisionally():
    result = select_candidate([
        candidate('slow', None, 50.0),
        candidate('fast', None, 40.0),
    ])

    assert result.status == 'provisional_selected'
    assert result.selected_candidate_id == 'fast'
    assert result.accuracy_status == 'not_evaluated'


def test_evaluated_candidate_outranks_provisional_candidate():
    result = select_candidate([
        candidate('unlabelled', None, 20.0),
        candidate('evaluated', 0.70, 80.0),
    ])

    assert result.status == 'selected'
    assert result.selected_candidate_id == 'evaluated'
    assert result.accuracy_status == 'evaluated'


def test_no_passing_candidate_is_explicitly_unavailable():
    result = select_candidate([
        candidate('missing', 0.9, 20.0, available=False),
        candidate('slow', 0.9, 200.0, latency_pass=False),
    ])

    assert result.status == 'unavailable'
    assert result.selected_candidate_id is None
    assert result.rejected == {
        'missing': ['available'],
        'slow': ['latency_pass'],
    }


@pytest.mark.parametrize('field,value', [
    ('phrase_region_recall', math.nan),
    ('phrase_region_recall', 1.01),
    ('p95_latency_ms', math.inf),
    ('p95_latency_ms', -0.1),
])
def test_selection_rejects_invalid_metrics(field, value):
    item = candidate('bad', 0.9, 40.0)
    item[field] = value

    with pytest.raises(ValueError, match=field):
        select_candidate([item])


def test_null_recall_is_the_only_unlabelled_sentinel():
    item = candidate('bad', None, 40.0)
    item['phrase_region_recall'] = 'not_evaluated'

    with pytest.raises(ValueError, match='phrase_region_recall'):
        select_candidate([item])


def test_selection_rejects_duplicate_ids():
    with pytest.raises(ValueError, match='duplicate candidate_id'):
        select_candidate([
            candidate('same', 0.8, 40.0),
            candidate('same', 0.9, 30.0),
        ])


def test_gate_fields_must_be_boolean():
    with pytest.raises(ValueError, match='available must be boolean'):
        select_candidate([candidate('bad', 0.8, 30.0, available='yes')])


def benchmark_payload(items):
    return {
        'schema_version': '1.0.0',
        'run_id': 'phase1-test',
        'platform': {
            'python': '3.8.10',
            'pytorch': '1.13.0',
            'device': 'Jetson AGX Orin',
        },
        'candidates': items,
    }


def test_model_benchmark_schema_accepts_selection_input():
    schema_path = (
        Path(__file__).resolve().parents[1] /
        'schemas' /
        'model_benchmark.schema.json')
    schema = json.loads(schema_path.read_text(encoding='utf-8'))

    payload = benchmark_payload([candidate('clip', 0.8, 40.0)])

    validate_benchmark(payload)
    assert schema['properties']['schema_version']['const'] == '1.0.0'
    assert set(schema['required']) == {
        'schema_version', 'run_id', 'platform', 'candidates'}


def test_cli_atomically_writes_selected_result(tmp_path):
    input_path = tmp_path / 'benchmark.json'
    output_path = tmp_path / 'selection.json'
    input_path.write_text(json.dumps(benchmark_payload([
        candidate('clip', 0.8, 40.0),
    ])), encoding='utf-8')

    exit_code = run(input_path, output_path)

    assert exit_code == 0
    output = json.loads(output_path.read_text(encoding='utf-8'))
    assert output['selection'] == {
        'status': 'selected',
        'selected_candidate_id': 'clip',
        'rejected': {},
        'accuracy_status': 'evaluated',
    }
    assert not list(tmp_path.glob('*.tmp'))


def test_cli_writes_unavailable_result_and_returns_nonzero(tmp_path):
    input_path = tmp_path / 'benchmark.json'
    output_path = tmp_path / 'selection.json'
    input_path.write_text(json.dumps(benchmark_payload([
        candidate('clip', 0.8, 40.0, available=False),
    ])), encoding='utf-8')

    exit_code = run(input_path, output_path)

    assert exit_code == 2
    output = json.loads(output_path.read_text(encoding='utf-8'))
    assert output['selection']['status'] == 'unavailable'
    assert output['selection']['selected_candidate_id'] is None


def test_model_selection_cli_is_packaged():
    setup_source = (
        Path(__file__).resolve().parents[1] / 'setup.py'
    ).read_text(encoding='utf-8')

    assert 'semantic_search_select_text_model' in setup_source
    assert 'track_robot_semantic_search.model_selection_cli:main' in setup_source
