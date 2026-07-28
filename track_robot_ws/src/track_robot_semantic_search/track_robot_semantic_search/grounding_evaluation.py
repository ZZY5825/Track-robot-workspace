import hashlib
import json
import math
from statistics import median
from typing import Mapping, Sequence

from .evaluation import percentile
from .grounding_dataset import (
    GroundingBox,
    GroundingCase,
    GroundingDataset,
)
from .grounding_predictions import (
    GroundingPrediction,
    GroundingPredictionSet,
)


SCHEMA_VERSION = '1.0.0'
_RECALL_GATE = 0.85
_FALSE_ACCEPT_GATE = 0.05
_MEDIAN_IOU_GATE = 0.50


def intersection_over_union(
        first: GroundingBox, second: GroundingBox) -> float:
    """Return the intersection-over-union of two XYWH boxes."""
    intersection_width = max(
        0.0, min(first.x + first.width, second.x + second.width) -
        max(first.x, second.x))
    intersection_height = max(
        0.0, min(first.y + first.height, second.y + second.height) -
        max(first.y, second.y))
    intersection = intersection_width * intersection_height
    first_area = max(0.0, first.width) * max(0.0, first.height)
    second_area = max(0.0, second.width) * max(0.0, second.height)
    union = first_area + second_area - intersection
    return 0.0 if union <= 0.0 else intersection / union


def _top_detection(prediction: GroundingPrediction, threshold: float):
    retained = [
        detection for detection in prediction.detections
        if detection.score >= threshold
    ]
    retained.sort(key=lambda detection: (
        -detection.score,
        detection.box.x,
        detection.box.y,
        detection.box.width,
        detection.box.height,
        detection.label,
    ))
    return retained[0] if retained else None


def metrics_at_threshold(
        cases: Sequence[GroundingCase],
        predictions: Mapping[str, GroundingPrediction],
        threshold: float) -> Mapping[str, object]:
    """Compute deterministic top-1 localization metrics."""
    threshold = float(threshold)
    if not math.isfinite(threshold) or threshold < 0.0 or threshold > 1.0:
        raise ValueError('threshold must be finite and in [0, 1]')

    positive_count = 0
    absent_count = 0
    recall_successes = 0
    false_accepts = 0
    empty_outputs = 0
    accepted_positive_ious = []
    cases = tuple(cases)

    for case_value in cases:
        try:
            prediction = predictions[case_value.case_id]
        except KeyError as error:
            raise ValueError(
                'missing prediction for case_id {}'.format(
                    case_value.case_id)) from error
        top = _top_detection(prediction, threshold)
        if top is None:
            empty_outputs += 1

        if case_value.target_present:
            positive_count += 1
            if top is None:
                continue
            top_iou = max(
                intersection_over_union(top.box, box)
                for box in case_value.boxes)
            accepted_positive_ious.append(top_iou)
            if top_iou >= 0.50:
                recall_successes += 1
        else:
            absent_count += 1
            if top is not None:
                false_accepts += 1

    return {
        'positive_case_count': positive_count,
        'absent_case_count': absent_count,
        'top1_recall_iou_50': (
            recall_successes / positive_count if positive_count else None),
        'target_absent_false_accept_rate': (
            false_accepts / absent_count if absent_count else None),
        'median_accepted_positive_iou': (
            median(accepted_positive_ious)
            if accepted_positive_ious else None),
        'empty_output_rate': (
            empty_outputs / len(cases) if cases else None),
    }


def _unavailable_selection(reason: str) -> Mapping[str, object]:
    return {
        'status': 'unavailable',
        'threshold': None,
        'reason': reason,
    }


def select_validation_threshold(
        cases: Sequence[GroundingCase],
        predictions: Mapping[str, GroundingPrediction]) -> Mapping[str, object]:
    """Select the highest validation-only score threshold meeting all gates."""
    validation_cases = tuple(case for case in cases
                             if case.split == 'validation')
    positive_count = sum(
        1 for case in validation_cases if case.target_present)
    absent_count = len(validation_cases) - positive_count
    if not positive_count:
        return _unavailable_selection('no_positive_validation_cases')
    if not absent_count:
        return _unavailable_selection(
            'no_target_absent_validation_cases')

    thresholds = {0.0, 1.0}
    for case_value in validation_cases:
        try:
            prediction = predictions[case_value.case_id]
        except KeyError as error:
            raise ValueError(
                'missing prediction for case_id {}'.format(
                    case_value.case_id)) from error
        thresholds.update(
            detection.score for detection in prediction.detections
            if math.isfinite(detection.score))

    for threshold in sorted(thresholds, reverse=True):
        metrics = metrics_at_threshold(
            validation_cases, predictions, threshold)
        if (
                metrics['top1_recall_iou_50'] >= _RECALL_GATE and
                metrics['target_absent_false_accept_rate'] <=
                _FALSE_ACCEPT_GATE and
                metrics['median_accepted_positive_iou'] is not None and
                metrics['median_accepted_positive_iou'] >= _MEDIAN_IOU_GATE):
            return {
                'status': 'selected',
                'threshold': threshold,
                'metrics': metrics,
            }

    return _unavailable_selection(
        'no_validation_threshold_meets_quality_gates')


def _canonical_case(case: GroundingCase) -> Mapping[str, object]:
    return {
        'case_id': case.case_id,
        'split': case.split,
        'image_sha256': case.image_sha256,
        'session_id': case.session_id,
        'physical_object_id': case.physical_object_id,
        'query': {
            'raw_text': case.query.raw_text,
            'normalized_text': case.query.normalized_text,
        },
        'target_present': case.target_present,
        'ground_truth_boxes_xywh': [
            [box.x, box.y, box.width, box.height]
            for box in case.boxes
        ],
        'scenario_tags': list(case.scenario_tags),
        'label_review_status': case.label_review_status,
    }


def _dataset_checksum(dataset: GroundingDataset) -> str:
    payload = {
        'dataset_id': dataset.dataset_id,
        'cases': [
            _canonical_case(case)
            for case in sorted(dataset.cases, key=lambda item: item.case_id)
        ],
    }
    canonical_json = json.dumps(
        payload, sort_keys=True, separators=(',', ':'),
        ensure_ascii=True, allow_nan=False)
    return hashlib.sha256(canonical_json.encode('utf-8')).hexdigest()


def _unavailable_metrics(cases: Sequence[GroundingCase]):
    positive_count = sum(1 for case in cases if case.target_present)
    return {
        'positive_case_count': positive_count,
        'absent_case_count': len(cases) - positive_count,
        'top1_recall_iou_50': None,
        'target_absent_false_accept_rate': None,
        'median_accepted_positive_iou': None,
        'empty_output_rate': None,
    }


def _resource_metrics(
        test_cases: Sequence[GroundingCase],
        predictions: Mapping[str, GroundingPrediction],
        incremental_cuda_reserved_mib: float) -> Mapping[str, object]:
    latencies = [
        predictions[case.case_id].complete_path_ms for case in test_cases
    ]
    mean_latency = sum(latencies) / len(latencies) if latencies else None
    return {
        'complete_path_case_count': len(latencies),
        'p50_complete_path_ms': percentile(latencies, 0.50),
        'p95_complete_path_ms': percentile(latencies, 0.95),
        'maximum_complete_path_ms': max(latencies) if latencies else None,
        'semantic_rate_hz': (
            1000.0 / mean_latency
            if mean_latency is not None and mean_latency > 0.0 else None),
        'incremental_cuda_reserved_mib': incremental_cuda_reserved_mib,
    }


def _release_gates(report: Mapping[str, object]) -> Mapping[str, bool]:
    test_metrics = report['test_metrics']
    resources = report['resources']
    gates = {
        'validation_threshold_selected': (
            report['validation_selection']['status'] == 'selected'),
        'runtime_available': report['runtime_available'] is True,
        'platform_compatible': report['platform_compatible'] is True,
        'licence_approved': report['licence_approved'] is True,
        'human_reviewed_test_labels': (
            report['human_reviewed_test_labels'] is True),
        'top1_recall_iou_50_at_least_0_85': (
            test_metrics['top1_recall_iou_50'] is not None and
            test_metrics['top1_recall_iou_50'] >= 0.85),
        'false_accept_rate_at_most_0_05': (
            test_metrics['target_absent_false_accept_rate'] is not None and
            test_metrics['target_absent_false_accept_rate'] <= 0.05),
        'median_iou_at_least_0_50': (
            test_metrics['median_accepted_positive_iou'] is not None and
            test_metrics['median_accepted_positive_iou'] >= 0.50),
        'latency_p95_at_most_150_ms': (
            resources['p95_complete_path_ms'] is not None and
            resources['p95_complete_path_ms'] <= 150.0),
        'semantic_rate_at_least_5_hz': (
            resources['semantic_rate_hz'] is not None and
            resources['semantic_rate_hz'] >= 5.0),
        'incremental_cuda_at_most_1536_mib': (
            resources['incremental_cuda_reserved_mib'] <= 1536.0),
    }
    gates['all_passed'] = all(gates.values())
    return gates


def evaluate_grounding_candidate(
        dataset: GroundingDataset,
        prediction_set: GroundingPredictionSet) -> Mapping[str, object]:
    """Evaluate one prediction set without allowing held-out threshold tuning."""
    if dataset.dataset_id != prediction_set.dataset_id:
        raise ValueError('dataset_id does not match prediction dataset_id')
    dataset_case_ids = {case.case_id for case in dataset.cases}
    prediction_case_ids = set(prediction_set.predictions)
    if dataset_case_ids != prediction_case_ids:
        raise ValueError('dataset and prediction case sets do not match')

    validation_cases = tuple(
        case for case in dataset.cases if case.split == 'validation')
    test_cases = tuple(case for case in dataset.cases if case.split == 'test')
    validation_selection = select_validation_threshold(
        validation_cases, prediction_set.predictions)
    if validation_selection['threshold'] is None:
        test_metrics = _unavailable_metrics(test_cases)
        per_scenario = {}
    else:
        threshold = validation_selection['threshold']
        test_metrics = metrics_at_threshold(
            test_cases, prediction_set.predictions, threshold)
        tags = sorted({
            tag for case in test_cases for tag in case.scenario_tags
        })
        per_scenario = {
            tag: metrics_at_threshold(
                tuple(case for case in test_cases
                      if tag in case.scenario_tags),
                prediction_set.predictions,
                threshold,
            )
            for tag in tags
        }

    release_evidence = prediction_set.release_evidence
    report = {
        'schema_version': SCHEMA_VERSION,
        'dataset_id': dataset.dataset_id,
        'dataset_checksum': _dataset_checksum(dataset),
        'candidate_id': prediction_set.candidate_id,
        'model_identity': dict(prediction_set.model_identity),
        'platform': dict(prediction_set.platform),
        'input_size': list(prediction_set.input_size),
        'validation_selection': validation_selection,
        'test_metrics': test_metrics,
        'per_scenario_test_metrics': per_scenario,
        'resources': _resource_metrics(
            test_cases, prediction_set.predictions,
            prediction_set.incremental_cuda_reserved_mib),
        'runtime_available': release_evidence['runtime_available'],
        'platform_compatible': release_evidence['platform_compatible'],
        'licence_approved': release_evidence['licence_approved'],
        'human_reviewed_test_labels': bool(test_cases) and all(
            case.label_review_status == 'human_verified'
            for case in test_cases),
    }
    gates = _release_gates(report)
    report['release_gates'] = gates
    report['reasons'] = [
        name for name, passed in gates.items()
        if name != 'all_passed' and not passed
    ]
    return report
