import json
import math
from pathlib import Path

import pytest

from track_robot_semantic_search.grounding_dataset import (
    GroundingBox,
    GroundingCase,
    GroundingDataset,
)
from track_robot_semantic_search.grounding_predictions import (
    GroundingDetection,
    GroundingPrediction,
    GroundingPredictionSet,
)
from track_robot_semantic_search.grounding_query import GroundingQuery


SCHEMA_PATH = (
    Path(__file__).resolve().parents[1] /
    'schemas' / 'grounding_evaluation_report.schema.json')


def case(case_id, split='validation', target_present=True, boxes=None,
         tags=('cluttered',), review_status='human_verified',
         image_path=None, image_sha256=None):
    if boxes is None:
        boxes = (GroundingBox(0.0, 0.0, 10.0, 10.0),) \
            if target_present else ()
    return GroundingCase(
        case_id=case_id,
        split=split,
        image_path=Path(image_path or '/fixture/{}.png'.format(case_id)),
        image_sha256=image_sha256 or ('a' * 64),
        session_id='session-{}'.format(case_id),
        physical_object_id='object-{}'.format(case_id)
        if target_present else '',
        query=GroundingQuery(
            raw_text='Tall Blue Container',
            normalized_text='tall blue container'),
        target_present=target_present,
        boxes=tuple(boxes),
        scenario_tags=tuple(tags),
        label_review_status=review_status,
    )


def detection(score, box=None, label='container'):
    return GroundingDetection(
        box=box or GroundingBox(0.0, 0.0, 10.0, 10.0),
        score=score,
        label=label,
    )


def prediction(case_id, detections=(), latency=100.0):
    return GroundingPrediction(
        case_id=case_id,
        complete_path_ms=latency,
        detections=tuple(detections),
    )


def prediction_set(cases, predictions, dataset_id='grounding-r0'):
    return GroundingPredictionSet(
        dataset_id=dataset_id,
        candidate_id='candidate-a',
        model_identity={
            'implementation': 'fixture',
            'code_revision': 'revision',
            'checkpoint_id': 'model.pt',
            'checkpoint_sha256': 'b' * 64,
            'licence': 'Apache-2.0',
        },
        platform={
            'role': 'jetson_candidate',
            'hardware': 'Jetson AGX Orin',
            'os': 'Ubuntu 20.04',
            'python': '3.8.10',
            'pytorch': '1.13.0',
            'device': 'cuda',
        },
        input_size=(1280, 1280),
        incremental_cuda_reserved_mib=1024.0,
        release_evidence={
            'runtime_available': True,
            'platform_compatible': True,
            'licence_approved': True,
        },
        predictions={item.case_id: item for item in predictions},
    )


def validation_fixture():
    cases = (
        case('positive'),
        case('absent', target_present=False),
    )
    predictions = {
        'positive': prediction('positive', (detection(0.7),)),
        'absent': prediction('absent', (detection(0.6),)),
    }
    return cases, predictions


def test_iou_is_exact_for_known_overlap():
    from track_robot_semantic_search.grounding_evaluation import (
        intersection_over_union,
    )

    first = GroundingBox(0.0, 0.0, 10.0, 10.0)
    second = GroundingBox(5.0, 0.0, 10.0, 10.0)

    assert intersection_over_union(first, second) == pytest.approx(1.0 / 3.0)
    assert intersection_over_union(
        first, GroundingBox(20.0, 20.0, 5.0, 5.0)) == 0.0


def test_top1_uses_highest_score_then_geometry_tie_break():
    from track_robot_semantic_search.grounding_evaluation import (
        metrics_at_threshold,
    )

    cases = (case('positive'),)
    predictions = {
        'positive': prediction('positive', (
            detection(0.8, GroundingBox(20.0, 20.0, 5.0, 5.0), 'z'),
            detection(0.8, GroundingBox(0.0, 0.0, 10.0, 10.0), 'a'),
        )),
    }

    metrics = metrics_at_threshold(cases, predictions, 0.5)

    assert metrics['top1_recall_iou_50'] == 1.0
    assert metrics['median_accepted_positive_iou'] == 1.0


def test_positive_uses_best_overlap_across_multiple_ground_truth_boxes():
    from track_robot_semantic_search.grounding_evaluation import (
        metrics_at_threshold,
    )

    cases = (case('positive', boxes=(
        GroundingBox(20.0, 20.0, 5.0, 5.0),
        GroundingBox(0.0, 0.0, 10.0, 10.0),
    )),)
    predictions = {'positive': prediction(
        'positive', (detection(0.8),))}

    assert metrics_at_threshold(
        cases, predictions, 0.5)['top1_recall_iou_50'] == 1.0


def test_absent_prediction_counts_as_false_accept():
    from track_robot_semantic_search.grounding_evaluation import (
        metrics_at_threshold,
    )

    cases = (case('absent', target_present=False),)
    predictions = {'absent': prediction(
        'absent', (detection(0.5),))}

    metrics = metrics_at_threshold(cases, predictions, 0.5)

    assert metrics['target_absent_false_accept_rate'] == 1.0
    assert metrics['empty_output_rate'] == 0.0


def test_empty_detections_are_empty_outputs_and_not_accepted_iou():
    from track_robot_semantic_search.grounding_evaluation import (
        metrics_at_threshold,
    )

    cases = (
        case('positive'),
        case('absent', target_present=False),
    )
    predictions = {
        item.case_id: prediction(item.case_id)
        for item in cases
    }

    metrics = metrics_at_threshold(cases, predictions, 0.5)

    assert metrics == {
        'positive_case_count': 1,
        'absent_case_count': 1,
        'top1_recall_iou_50': 0.0,
        'target_absent_false_accept_rate': 0.0,
        'median_accepted_positive_iou': None,
        'empty_output_rate': 1.0,
    }


def test_metrics_are_none_when_required_case_class_is_missing():
    from track_robot_semantic_search.grounding_evaluation import (
        metrics_at_threshold,
    )

    positive = case('positive')
    metrics = metrics_at_threshold(
        (positive,), {'positive': prediction('positive', (detection(0.8),))},
        0.5)

    assert metrics['absent_case_count'] == 0
    assert metrics['target_absent_false_accept_rate'] is None

    absent = case('absent', target_present=False)
    metrics = metrics_at_threshold(
        (absent,), {'absent': prediction('absent')}, 0.5)
    assert metrics['positive_case_count'] == 0
    assert metrics['top1_recall_iou_50'] is None
    assert metrics['median_accepted_positive_iou'] is None


def test_validation_threshold_selects_highest_eligible_score():
    from track_robot_semantic_search.grounding_evaluation import (
        select_validation_threshold,
    )

    cases, predictions = validation_fixture()

    selected = select_validation_threshold(cases, predictions)

    assert selected['threshold'] == pytest.approx(0.7)
    assert selected['status'] == 'selected'
    assert 'reason' not in selected
    assert selected['metrics']['top1_recall_iou_50'] == 1.0


def test_validation_threshold_never_reads_test_cases():
    from track_robot_semantic_search.grounding_evaluation import (
        select_validation_threshold,
    )

    validation_cases, validation_predictions = validation_fixture()
    poisoned_test_case = case('test-only', split='test')
    poisoned_test_prediction = prediction(
        'test-only', (detection(0.99),))

    selected = select_validation_threshold(
        validation_cases,
        dict(validation_predictions, **{
            poisoned_test_case.case_id: poisoned_test_prediction,
        }))

    assert selected['threshold'] == pytest.approx(0.7)


@pytest.mark.parametrize('cases,predictions,reason', [
    (
        (case('absent', target_present=False),),
        {'absent': prediction('absent')},
        'no_positive_validation_cases',
    ),
    (
        (case('positive'),),
        {'positive': prediction('positive', (detection(0.8),))},
        'no_target_absent_validation_cases',
    ),
])
def test_validation_threshold_requires_both_case_classes(
        cases, predictions, reason):
    from track_robot_semantic_search.grounding_evaluation import (
        select_validation_threshold,
    )

    assert select_validation_threshold(cases, predictions) == {
        'status': 'unavailable',
        'threshold': None,
        'reason': reason,
    }


def test_validation_threshold_is_unavailable_when_no_candidate_meets_gates():
    from track_robot_semantic_search.grounding_evaluation import (
        select_validation_threshold,
    )

    cases = (
        case('positive'),
        case('absent', target_present=False),
    )
    predictions = {
        'positive': prediction(
            'positive', (detection(
                0.8, GroundingBox(20.0, 20.0, 5.0, 5.0)),)),
        'absent': prediction('absent', (detection(0.1),)),
    }

    assert select_validation_threshold(cases, predictions) == {
        'status': 'unavailable',
        'threshold': None,
        'reason': 'no_validation_threshold_meets_quality_gates',
    }


def evaluation_fixture(test_review_status='human_verified'):
    cases = (
        case('validation-positive'),
        case('validation-absent', target_present=False),
        case(
            'test-positive', split='test', tags=('cluttered', 'small_target'),
            review_status=test_review_status),
        case(
            'test-absent', split='test', target_present=False,
            tags=('cluttered',), review_status=test_review_status),
    )
    predictions = (
        prediction(
            'validation-positive', (detection(0.7),), latency=10.0),
        prediction('validation-absent', (), latency=20.0),
        prediction('test-positive', (detection(0.8),), latency=30.0),
        prediction('test-absent', (), latency=40.0),
    )
    dataset = GroundingDataset(dataset_id='grounding-r0', cases=cases)
    return dataset, prediction_set(cases, predictions)


def test_evaluation_freezes_validation_threshold_and_reports_test_evidence():
    from track_robot_semantic_search.grounding_evaluation import (
        evaluate_grounding_candidate,
    )

    dataset, predictions = evaluation_fixture()

    report = evaluate_grounding_candidate(dataset, predictions)

    assert report['schema_version'] == '1.0.0'
    assert report['validation_selection']['threshold'] == pytest.approx(0.7)
    assert report['test_metrics']['top1_recall_iou_50'] == 1.0
    assert report['test_metrics']['target_absent_false_accept_rate'] == 0.0
    assert report['per_scenario_test_metrics']['cluttered'] == \
        report['test_metrics']
    assert report['per_scenario_test_metrics']['small_target'][
        'absent_case_count'] == 0
    assert report['human_reviewed_test_labels'] is True
    assert report['resources'] == {
        'complete_path_case_count': 2,
        'p50_complete_path_ms': 35.0,
        'p95_complete_path_ms': pytest.approx(39.5),
        'maximum_complete_path_ms': 40.0,
        'semantic_rate_hz': pytest.approx(1000.0 / 35.0),
        'incremental_cuda_reserved_mib': 1024.0,
    }
    assert report['runtime_available'] is True
    assert report['platform_compatible'] is True
    assert report['licence_approved'] is True
    assert report['release_gates']['all_passed'] is True
    assert report['reasons'] == []


def test_evaluation_reports_unreviewed_test_labels_as_failed_gate():
    from track_robot_semantic_search.grounding_evaluation import (
        evaluate_grounding_candidate,
    )

    dataset, predictions = evaluation_fixture('human_authored')

    report = evaluate_grounding_candidate(dataset, predictions)

    assert report['human_reviewed_test_labels'] is False
    assert report['release_gates']['human_reviewed_test_labels'] is False
    assert 'human_reviewed_test_labels' in report['reasons']


@pytest.mark.parametrize('mutation,match', [
    ('dataset_id', 'dataset_id'),
    ('missing', 'case sets'),
    ('extra', 'case sets'),
])
def test_evaluation_rejects_identity_or_prediction_case_set_mismatch(
        mutation, match):
    from track_robot_semantic_search.grounding_evaluation import (
        evaluate_grounding_candidate,
    )

    dataset, predictions = evaluation_fixture()
    if mutation == 'dataset_id':
        predictions = prediction_set(
            dataset.cases, predictions.predictions.values(),
            dataset_id='other-dataset')
    elif mutation == 'missing':
        predictions = prediction_set(
            dataset.cases,
            tuple(predictions.predictions.values())[:-1])
    else:
        predictions = prediction_set(
            dataset.cases,
            tuple(predictions.predictions.values()) +
            (prediction('extra-case'),))

    with pytest.raises(ValueError, match=match):
        evaluate_grounding_candidate(dataset, predictions)


def test_dataset_checksum_is_canonical_and_covers_review_and_image_digest():
    from track_robot_semantic_search.grounding_evaluation import (
        evaluate_grounding_candidate,
    )

    dataset, predictions = evaluation_fixture()
    first = evaluate_grounding_candidate(dataset, predictions)
    relocated = GroundingDataset(
        dataset_id=dataset.dataset_id,
        cases=tuple(
            GroundingCase(
                **dict(
                    item.__dict__,
                    image_path=Path('/different-machine') / item.image_path.name,
                ))
            for item in reversed(dataset.cases)))
    relocated_predictions = prediction_set(
        relocated.cases, predictions.predictions.values())

    second = evaluate_grounding_candidate(relocated, relocated_predictions)

    assert first['dataset_checksum'] == second['dataset_checksum']
    changed_case = GroundingCase(
        **dict(dataset.cases[0].__dict__, image_sha256='c' * 64))
    changed_dataset = GroundingDataset(
        dataset.dataset_id, (changed_case,) + dataset.cases[1:])
    changed = evaluate_grounding_candidate(changed_dataset, predictions)
    assert changed['dataset_checksum'] != first['dataset_checksum']

    review_changed_case = GroundingCase(
        **dict(dataset.cases[-1].__dict__,
               label_review_status='human_authored'))
    review_changed_dataset = GroundingDataset(
        dataset.dataset_id, dataset.cases[:-1] + (review_changed_case,))
    review_changed = evaluate_grounding_candidate(
        review_changed_dataset, predictions)
    assert review_changed['dataset_checksum'] != first['dataset_checksum']


def test_p95_latency_is_finite_linearly_interpolated():
    from track_robot_semantic_search.grounding_evaluation import (
        evaluate_grounding_candidate,
    )

    dataset, predictions = evaluation_fixture()
    report = evaluate_grounding_candidate(dataset, predictions)

    value = report['resources']['p95_complete_path_ms']
    assert value == pytest.approx(39.5)
    assert math.isfinite(value)


def test_report_schema_is_versioned_closed_and_accepts_generated_report():
    from track_robot_semantic_search.grounding_evaluation import (
        evaluate_grounding_candidate,
    )

    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    dataset, predictions = evaluation_fixture()
    report = evaluate_grounding_candidate(dataset, predictions)

    assert schema['properties']['schema_version']['const'] == '1.0.0'
    assert schema['additionalProperties'] is False
    assert set(schema['required']) == {
        'schema_version', 'dataset_id', 'dataset_checksum', 'candidate_id',
        'model_identity', 'platform', 'input_size', 'validation_selection',
        'test_metrics', 'per_scenario_test_metrics', 'resources',
        'runtime_available', 'platform_compatible', 'licence_approved',
        'human_reviewed_test_labels', 'release_gates', 'reasons',
    }
    assert set(report) == set(schema['required'])

    def assert_objects_are_closed(node):
        if isinstance(node, dict):
            if node.get('type') == 'object':
                assert node.get('additionalProperties') is False
            for value in node.values():
                assert_objects_are_closed(value)
        elif isinstance(node, list):
            for value in node:
                assert_objects_are_closed(value)

    assert_objects_are_closed(schema)
