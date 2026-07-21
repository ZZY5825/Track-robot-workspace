import json

import pytest

from track_robot_semantic_search.task_threshold_calibration import (
    calibrate_task_threshold,
)
from track_robot_semantic_search.task_threshold_calibration_cli import run


def sample(index, relevant, score, **extra):
    value = {
        'dataset_id': 'calibration-1',
        'split': 'calibration',
        'query_id': index // 4 + 1,
        'candidate_id': 'candidate-{}'.format(index),
        'task_relevant': relevant,
        'relevance_score': score,
    }
    value.update(extra)
    return value


def sufficient_samples():
    positives = [
        sample(index, True, 0.90 - index * 0.005)
        for index in range(30)
    ]
    negatives = [
        sample(100 + index, False, 0.20 + index * 0.005)
        for index in range(30)
    ]
    return positives + negatives


def test_selects_highest_threshold_meeting_recall_and_false_confirmation_gates():
    report = calibrate_task_threshold('calibration-1', sufficient_samples())

    assert report['status'] == 'calibrated'
    assert report['selected_threshold'] == pytest.approx(0.77)
    assert report['selected_metrics']['recall'] == pytest.approx(0.90)
    assert report['selected_metrics']['hard_negative_false_confirmation_rate'] == 0.0
    assert report['gates'] == {
        'minimum_30_positive_samples': True,
        'minimum_30_hard_negative_samples': True,
        'candidate_recall_at_least_0_90': True,
        'hard_negative_false_confirmation_at_most_0_05': True,
    }


def test_insufficient_sample_counts_never_calibrate_a_threshold():
    report = calibrate_task_threshold(
        'calibration-1', sufficient_samples()[:20])

    assert report['status'] == 'insufficient_evidence'
    assert report['selected_threshold'] is None
    assert 'positive_samples_below_30' in report['reasons']
    assert 'hard_negative_samples_below_30' in report['reasons']


def test_no_threshold_meeting_hard_negative_gate_is_insufficient():
    samples = [sample(index, True, 0.7) for index in range(30)]
    samples += [sample(100 + index, False, 0.8) for index in range(30)]

    report = calibrate_task_threshold('calibration-1', samples)

    assert report['status'] == 'insufficient_evidence'
    assert report['selected_threshold'] is None
    assert 'no_threshold_meets_quality_gates' in report['reasons']


@pytest.mark.parametrize(
    'mutation, message',
    [
        (lambda rows: rows.append(dict(rows[0])), 'duplicate'),
        (lambda rows: rows[0].update(split='final_test'), 'calibration split'),
        (lambda rows: rows[0].update(relevance_score=float('nan')), 'score'),
        (lambda rows: rows[0].update(relevance_score=1.1), 'score'),
        (lambda rows: rows[0].update(task_relevant=1), 'task_relevant'),
        (lambda rows: rows[0].update(dataset_id='other'), 'dataset_id'),
    ],
)
def test_rejects_invalid_or_leaky_calibration_samples(mutation, message):
    rows = sufficient_samples()
    mutation(rows)
    with pytest.raises(ValueError, match=message):
        calibrate_task_threshold('calibration-1', rows)


def test_rejects_unbounded_sample_input():
    repeated = [
        sample(index, index % 2 == 0, 0.5)
        for index in range(100_001)
    ]
    with pytest.raises(ValueError, match='100000'):
        calibrate_task_threshold('calibration-1', repeated)


def test_sample_order_does_not_change_hash_or_report():
    rows = sufficient_samples()
    first = calibrate_task_threshold('calibration-1', rows)
    second = calibrate_task_threshold('calibration-1', list(reversed(rows)))
    assert first == second
    assert len(first['samples_sha256']) == 64


def test_cli_writes_byte_identical_report_and_returns_two_when_insufficient(
        tmp_path):
    samples_path = tmp_path / 'samples.jsonl'
    output_path = tmp_path / 'report.json'
    samples_path.write_text(
        ''.join(json.dumps(row) + '\n' for row in sufficient_samples()),
        encoding='utf-8')

    assert run(samples_path, output_path, 'calibration-1') == 0
    first = output_path.read_bytes()
    assert run(samples_path, output_path, 'calibration-1') == 0
    assert output_path.read_bytes() == first

    insufficient = tmp_path / 'insufficient.jsonl'
    insufficient.write_text(
        json.dumps(sufficient_samples()[0]) + '\n', encoding='utf-8')
    assert run(insufficient, output_path, 'calibration-1') == 2


def test_report_schema_declares_strict_calibration_contract():
    root = __import__('pathlib').Path(__file__).resolve().parents[1]
    schema = json.loads((
        root / 'schemas' / 'phase2_task_threshold_calibration.schema.json'
    ).read_text(encoding='utf-8'))
    report = calibrate_task_threshold('calibration-1', sufficient_samples())

    assert schema['properties']['schema_version']['const'] == '1.0.0'
    assert set(schema['required']) == set(report)
    assert schema['additionalProperties'] is False
