import hashlib
import json
import math
import re
from collections import Counter, defaultdict


SCHEMA_VERSION = '2.0.0'
REQUIRED_SCENARIOS = (
    'static_multi_view',
    'similar_static_objects',
    'moving_human_crossing',
    'camera_occlusion',
    'camera_fov_exit_lidar_visible',
    'both_sensors_exit_reentry',
    'lidar_cluster_split',
    'lidar_cluster_merge',
    'camera_false_positive',
    'lidar_false_cluster',
    'robot_rotation_translation',
    'task_change_without_memory_clear',
)
_SHA256 = re.compile(r'^[0-9a-f]{64}$')


def _finite(value):
    return (
        isinstance(value, (int, float)) and
        not isinstance(value, bool) and math.isfinite(value))


def _percentile(values, quantile):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    position = quantile * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def _ratio(numerator, denominator):
    return None if denominator == 0 else numerator / denominator


def _record_hash(records):
    encoded = json.dumps(
        records, sort_keys=True, separators=(',', ':'),
        ensure_ascii=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _valid_vector(value):
    return (
        isinstance(value, list) and len(value) == 3 and
        all(_finite(item) for item in value))


def _public_key(record):
    epoch = record.get('memory_epoch_id')
    object_id = record.get('global_object_id')
    if (not isinstance(epoch, int) or isinstance(epoch, bool) or epoch <= 0 or
            not isinstance(object_id, int) or isinstance(object_id, bool) or
            object_id <= 0):
        return None
    return epoch, object_id


def _annotation_index(annotations):
    result = {}
    ignored = set()
    for item in annotations:
        if not isinstance(item, dict):
            raise ValueError('Phase 2 annotation must be an object')
        stamp = item.get('stamp_ns')
        object_id = item.get('object_id')
        if (not isinstance(stamp, int) or isinstance(stamp, bool) or stamp < 0 or
                not isinstance(object_id, str) or not object_id):
            raise ValueError('Phase 2 annotation identity is invalid')
        key = stamp, object_id
        if key in result:
            raise ValueError('duplicate Phase 2 annotation identity')
        if item.get('ignore', False) is True:
            ignored.add(key)
        elif item.get('ignore', False) is not False:
            raise ValueError('annotation ignore flag must be boolean')
        result[key] = item
    return result, ignored


def _identity_metrics(predictions):
    if not predictions:
        return {
            'available': False,
            'continuity_ratio': None,
            'id_switches': None,
            'duplicate_objects': None,
            'incorrect_merges': None,
            'fragmentation_count': None,
            'reason': 'matched_identity_samples_unavailable',
        }
    by_object_stamp = defaultdict(set)
    by_stamp_key = defaultdict(set)
    key_counts = Counter()
    for item in predictions:
        public_key = _public_key(item)
        if public_key is None:
            continue
        object_id = item['annotation_object_id']
        stamp = item['stamp_ns']
        by_object_stamp[(object_id, stamp)].add(public_key)
        by_stamp_key[(stamp, public_key)].add(object_id)
        key_counts[(object_id, public_key)] += 1
    if not by_object_stamp:
        return _identity_metrics([])

    duplicates = sum(max(0, len(keys) - 1) for keys in by_object_stamp.values())
    incorrect_merges = sum(
        max(0, len(objects) - 1) for objects in by_stamp_key.values())
    switches = 0
    fragmentations = 0
    dominant_total = 0
    sample_total = 0
    object_ids = sorted({key[0] for key in by_object_stamp})
    for object_id in object_ids:
        samples = sorted(
            (stamp, sorted(keys)[0])
            for (candidate_id, stamp), keys in by_object_stamp.items()
            if candidate_id == object_id)
        switches += sum(
            first[1] != second[1]
            for first, second in zip(samples, samples[1:]))
        unique_keys = {
            public_key for (candidate_id, public_key), count in key_counts.items()
            if candidate_id == object_id and count > 0}
        fragmentations += max(0, len(unique_keys) - 1)
        counts = [
            count for (candidate_id, _), count in key_counts.items()
            if candidate_id == object_id]
        dominant_total += max(counts)
        sample_total += sum(counts)
    return {
        'available': True,
        'continuity_ratio': _ratio(dominant_total, sample_total),
        'id_switches': switches,
        'duplicate_objects': duplicates,
        'incorrect_merges': incorrect_merges,
        'fragmentation_count': fragmentations,
        'reason': '',
    }


def _association_metrics(predictions):
    samples = [item for item in predictions if all(
        isinstance(item.get(name), bool) for name in (
            'association_expected', 'association_decision',
            'association_correct'))]
    if not samples:
        return {
            'available': False, 'true_positive': None,
            'false_positive': None, 'false_negative': None,
            'precision': None, 'recall': None,
            'reason': 'association_labels_unavailable'}
    true_positive = sum(
        item['association_expected'] and item['association_decision'] and
        item['association_correct'] for item in samples)
    false_positive = sum(
        item['association_decision'] and not (
            item['association_expected'] and item['association_correct'])
        for item in samples)
    false_negative = sum(
        item['association_expected'] and not (
            item['association_decision'] and item['association_correct'])
        for item in samples)
    return {
        'available': True,
        'true_positive': true_positive,
        'false_positive': false_positive,
        'false_negative': false_negative,
        'precision': _ratio(true_positive, true_positive + false_positive),
        'recall': _ratio(true_positive, true_positive + false_negative),
        'reason': '',
    }


def _position_metrics(annotation_by_key, predictions):
    errors = []
    for item in predictions:
        annotation = annotation_by_key.get(
            (item['stamp_ns'], item['annotation_object_id']))
        position = annotation.get('position_3d') if annotation else None
        expected = position.get('xyz_m') if isinstance(position, dict) else None
        actual = item.get('position_m')
        if _valid_vector(expected) and _valid_vector(actual):
            errors.append(math.sqrt(sum(
                (float(first) - float(second)) ** 2
                for first, second in zip(expected, actual))))
    return {
        'available': bool(errors),
        'sample_count': len(errors),
        'p50_error_m': _percentile(errors, 0.50),
        'p95_error_m': _percentile(errors, 0.95),
        'reason': '' if errors else 'paired_3d_annotations_unavailable',
    }


def _binary_event_metric(predictions, expected_name, success_name):
    samples = [item for item in predictions if item.get(expected_name) is True]
    if not samples:
        return {
            'available': False, 'trial_count': 0, 'success_count': 0,
            'rate': None, 'reason': '{}_unavailable'.format(expected_name)}
    if any(not isinstance(item.get(success_name), bool) for item in samples):
        raise ValueError('{} must be boolean'.format(success_name))
    successes = sum(item[success_name] for item in samples)
    return {
        'available': True, 'trial_count': len(samples),
        'success_count': successes, 'rate': successes / len(samples),
        'reason': ''}


def _task_threshold(calibration):
    if calibration is None:
        return None
    if not isinstance(calibration, dict):
        raise ValueError('task threshold calibration must be an object')
    required = {
        'schema_version', 'dataset_id', 'split', 'samples_sha256',
        'status', 'selected_threshold', 'gates',
    }
    if not required <= set(calibration):
        raise ValueError('task threshold calibration is incomplete')
    if (calibration['schema_version'] != '1.0.0' or
            not isinstance(calibration['dataset_id'], str) or
            not calibration['dataset_id'] or
            calibration['split'] != 'calibration' or
            not isinstance(calibration['samples_sha256'], str) or
            not _SHA256.fullmatch(calibration['samples_sha256']) or
            calibration['status'] not in (
                'calibrated', 'insufficient_evidence') or
            not isinstance(calibration['gates'], dict)):
        raise ValueError('task threshold calibration contract is invalid')
    if calibration['status'] != 'calibrated':
        return None
    threshold = calibration['selected_threshold']
    if (not _finite(threshold) or threshold < 0.0 or threshold > 1.0 or
            not calibration['gates'] or
            not all(value is True for value in calibration['gates'].values())):
        raise ValueError('calibrated task threshold evidence is invalid')
    return float(threshold)


def _task_metrics(annotation_by_key, predictions, frozen_threshold):
    task_annotations = {
        key: item for key, item in annotation_by_key.items()
        if isinstance(item.get('task_relevant'), bool)
    }
    positive_count = sum(
        item['task_relevant'] for item in task_annotations.values())
    negative_count = len(task_annotations) - positive_count
    unavailable = {
        'available': False,
        'threshold_calibrated': frozen_threshold is not None,
        'frozen_threshold': frozen_threshold,
        'positive_count': positive_count,
        'hard_negative_count': negative_count,
        'true_positive': None, 'false_positive': None,
        'false_negative': None, 'true_negative': None,
        'confirmed_precision': None, 'candidate_recall': None,
        'hard_negative_false_confirmation_rate': None,
        'query_count': 0, 'top1_accuracy': None,
        'mean_reciprocal_rank': None,
        'reason': 'task_relevance_annotations_unavailable',
    }
    if not task_annotations or positive_count == 0 or negative_count == 0:
        return unavailable

    predictions_by_key = defaultdict(list)
    ranks = defaultdict(list)
    for item in predictions:
        key = item['stamp_ns'], item['annotation_object_id']
        annotation = task_annotations.get(key)
        if annotation is None:
            continue
        names = (
            'query_id', 'task_relevant', 'task_rank', 'task_selected',
            'task_relevance', 'task_threshold')
        if any(name not in item for name in names):
            raise ValueError('task prediction evidence is incomplete')
        query_id = item['query_id']
        rank = item['task_rank']
        selected = item['task_selected']
        relevance = item['task_relevance']
        threshold = item['task_threshold']
        if (not isinstance(query_id, int) or isinstance(query_id, bool) or
                query_id <= 0 or not isinstance(rank, int) or
                isinstance(rank, bool) or rank <= 0 or
                not isinstance(selected, bool) or
                not isinstance(item['task_relevant'], bool) or
                not _finite(relevance) or relevance < 0.0 or relevance > 1.0 or
                not _finite(threshold) or threshold < 0.0 or threshold > 1.0):
            raise ValueError('task prediction evidence is invalid')
        if item['task_relevant'] != annotation['task_relevant']:
            raise ValueError('task prediction relevance label conflicts with annotation')
        annotation_query = annotation.get('query_id')
        if annotation_query is not None and annotation_query != query_id:
            raise ValueError('task prediction query conflicts with annotation')
        expected_selection = float(relevance) >= float(threshold)
        if selected != expected_selection:
            raise ValueError('task selection conflicts with score and threshold')
        if (frozen_threshold is not None and
                not math.isclose(
                    float(threshold), frozen_threshold,
                    rel_tol=0.0, abs_tol=1e-12)):
            raise ValueError('task prediction does not use frozen threshold')
        predictions_by_key[key].append(item)
        if annotation['task_relevant']:
            ranks[query_id].append(rank)

    true_positive = 0
    false_positive = 0
    for key, annotation in task_annotations.items():
        selected = any(
            item['task_selected'] for item in predictions_by_key.get(key, []))
        if annotation['task_relevant']:
            true_positive += selected
        else:
            false_positive += selected
    false_negative = positive_count - true_positive
    true_negative = negative_count - false_positive
    best_ranks = [min(values) for _, values in sorted(ranks.items())]
    return {
        'available': True,
        'threshold_calibrated': frozen_threshold is not None,
        'frozen_threshold': frozen_threshold,
        'positive_count': positive_count,
        'hard_negative_count': negative_count,
        'true_positive': true_positive,
        'false_positive': false_positive,
        'false_negative': false_negative,
        'true_negative': true_negative,
        'confirmed_precision': _ratio(
            true_positive, true_positive + false_positive),
        'candidate_recall': _ratio(true_positive, positive_count),
        'hard_negative_false_confirmation_rate': _ratio(
            false_positive, negative_count),
        'query_count': len(best_ranks),
        'top1_accuracy': _ratio(
            sum(rank == 1 for rank in best_ranks), len(best_ranks)),
        'mean_reciprocal_rank': _ratio(
            sum(1.0 / rank for rank in best_ranks), len(best_ranks)),
        'reason': '',
    }


def _memory_lifetime_metrics(predictions):
    stamps = defaultdict(list)
    for item in predictions:
        key = _public_key(item)
        if key is not None:
            stamps[key].append(item['stamp_ns'])
    lifetimes = [
        (max(values) - min(values)) / 1e9 for values in stamps.values()]
    return {
        'available': bool(lifetimes), 'object_count': len(lifetimes),
        'p50_sec': _percentile(lifetimes, 0.50),
        'p95_sec': _percentile(lifetimes, 0.95),
        'reason': '' if lifetimes else 'object_lifetime_samples_unavailable',
    }


def _runtime_metrics(runtime):
    unavailable = {
        'available': False, 'update_rate_hz': None,
        'core_latency_p50_ms': None, 'core_latency_p95_ms': None,
        'semantic_path_latency_p50_ms': None,
        'semantic_path_latency_p95_ms': None,
        'duration_sec': None, 'source_span_sec': None,
        'drops': None, 'bounded_growth_pass': None,
        'long_duration_pass': None,
        'reason': 'runtime_profile_unavailable'}
    if not isinstance(runtime, dict):
        return unavailable
    duration = runtime.get('duration_sec')
    stamps = runtime.get('update_stamps_ns')
    modules = runtime.get('module_latency_ms')
    core = modules.get('semantic_memory_core') if isinstance(modules, dict) else None
    semantic_path = runtime.get('semantic_path_latency_ms')
    drops = runtime.get('drops')
    bounded = runtime.get('bounded_growth_pass')
    if (not _finite(duration) or duration <= 0.0 or
            not isinstance(stamps, list) or len(stamps) < 2 or
            any(not isinstance(value, int) or isinstance(value, bool)
                for value in stamps) or
            not isinstance(core, list) or not core or
            any(not _finite(value) or value < 0.0 for value in core) or
            not isinstance(semantic_path, list) or not semantic_path or
            any(not _finite(value) or value < 0.0 for value in semantic_path) or
            not isinstance(drops, int) or isinstance(drops, bool) or drops < 0 or
            not isinstance(bounded, bool)):
        raise ValueError('runtime profile violates the Phase 2 contract')
    source_span = (max(stamps) - min(stamps)) / 1e9
    rate = 0.0 if source_span <= 0.0 else (len(stamps) - 1) / source_span
    return {
        'available': True, 'update_rate_hz': rate,
        'core_latency_p50_ms': _percentile(core, 0.50),
        'core_latency_p95_ms': _percentile(core, 0.95),
        'semantic_path_latency_p50_ms': _percentile(semantic_path, 0.50),
        'semantic_path_latency_p95_ms': _percentile(semantic_path, 0.95),
        'duration_sec': float(duration), 'source_span_sec': source_span,
        'drops': drops,
        'bounded_growth_pass': bounded,
        'long_duration_pass': min(float(duration), source_span) >= 1800.0,
        'reason': ''}


def _resource_metrics(resources):
    result = {'available': False, 'cpu_p95_percent': None,
              'gpu_p95_percent': None, 'resident_memory_p95_mb': None,
              'cuda_reserved_memory_p95_mib': None,
              'reason': 'resource_profile_unavailable'}
    if not isinstance(resources, dict):
        return result
    names = (
        'cpu_percent', 'gpu_percent', 'resident_memory_mb',
        'cuda_reserved_memory_mib')
    if any(not isinstance(resources.get(name), list) or
           not resources[name] or any(
               not _finite(value) or value < 0.0 for value in resources[name])
           for name in names):
        raise ValueError('resource profile violates the Phase 2 contract')
    return {
        'available': True,
        'cpu_p95_percent': _percentile(resources['cpu_percent'], 0.95),
        'gpu_p95_percent': _percentile(resources['gpu_percent'], 0.95),
        'resident_memory_p95_mb': _percentile(
            resources['resident_memory_mb'], 0.95),
        'cuda_reserved_memory_p95_mib': _percentile(
            resources['cuda_reserved_memory_mib'], 0.95),
        'reason': '',
    }


def build_phase2_report(
        dataset_id, manifest_sha256, annotations, predictions,
        runtime, resources, covered_scenarios,
        deterministic_replay_passed, human_tracking_regression_passed,
        software_revision, task_threshold_calibration=None):
    if (not isinstance(dataset_id, str) or not dataset_id or
            not isinstance(manifest_sha256, str) or
            not _SHA256.fullmatch(manifest_sha256) or
            not isinstance(software_revision, str) or not software_revision):
        raise ValueError('Phase 2 report provenance is invalid')
    if not isinstance(annotations, list) or not isinstance(predictions, list):
        raise ValueError('annotations and predictions must be arrays')
    if (not isinstance(covered_scenarios, list) or
            any(item not in REQUIRED_SCENARIOS for item in covered_scenarios) or
            len(set(covered_scenarios)) != len(covered_scenarios)):
        raise ValueError('covered scenarios are invalid or duplicated')
    if (not isinstance(deterministic_replay_passed, bool) or
            not isinstance(human_tracking_regression_passed, bool)):
        raise ValueError('external gate evidence must be boolean')

    annotation_by_key, ignored = _annotation_index(annotations)
    evaluated_annotations = {
        key: value for key, value in annotation_by_key.items()
        if key not in ignored}
    matched_predictions = []
    for item in predictions:
        if not isinstance(item, dict):
            raise ValueError('Phase 2 prediction must be an object')
        stamp = item.get('stamp_ns')
        object_id = item.get('annotation_object_id')
        if (not isinstance(stamp, int) or isinstance(stamp, bool) or stamp < 0 or
                not isinstance(object_id, str) or not object_id):
            raise ValueError('Phase 2 prediction match identity is invalid')
        if (stamp, object_id) in evaluated_annotations:
            matched_predictions.append(item)

    identity = _identity_metrics(matched_predictions)
    association = _association_metrics(matched_predictions)
    position = _position_metrics(evaluated_annotations, matched_predictions)
    reidentification = _binary_event_metric(
        matched_predictions, 'reidentification_expected',
        'reidentification_success')
    stale_reactivation = _binary_event_metric(
        matched_predictions, 'stale_reactivation_expected',
        'stale_reactivation_success')
    stale_reactivation['accuracy'] = stale_reactivation.pop('rate')
    frozen_task_threshold = _task_threshold(task_threshold_calibration)
    task_ranking = _task_metrics(
        evaluated_annotations, matched_predictions, frozen_task_threshold)
    memory_lifetime = _memory_lifetime_metrics(matched_predictions)
    runtime_metric = _runtime_metrics(runtime)
    resource_metric = _resource_metrics(resources)

    covered = sorted(covered_scenarios)
    missing = sorted(set(REQUIRED_SCENARIOS) - set(covered))
    gates = {
        'zero_identity_corruption': bool(
            identity['available'] and identity['id_switches'] == 0 and
            identity['duplicate_objects'] == 0 and
            identity['incorrect_merges'] == 0),
        'association_precision_at_least_0_95': bool(
            association['precision'] is not None and
            association['precision'] >= 0.95),
        'association_recall_at_least_0_80': bool(
            association['recall'] is not None and
            association['recall'] >= 0.80),
        'reidentification_evaluated_and_successful': bool(
            reidentification['available'] and reidentification['rate'] >= 0.80),
        'stale_reactivation_evaluated_and_accurate': bool(
            stale_reactivation['available'] and
            stale_reactivation['accuracy'] >= 0.80),
        'position_consistency_reported': position['available'],
        'task_threshold_calibrated_and_frozen': bool(
            task_ranking['threshold_calibrated']),
        'task_candidate_recall_at_least_0_90': bool(
            task_ranking['candidate_recall'] is not None and
            task_ranking['candidate_recall'] >= 0.90),
        'task_hard_negative_false_confirmation_at_most_0_05': bool(
            task_ranking['hard_negative_false_confirmation_rate'] is not None and
            task_ranking['hard_negative_false_confirmation_rate'] <= 0.05),
        'update_rate_at_least_5_hz': bool(
            runtime_metric['available'] and
            runtime_metric['update_rate_hz'] >= 5.0),
        'core_latency_p95_at_most_50_ms': bool(
            runtime_metric['available'] and
            runtime_metric['core_latency_p95_ms'] <= 50.0),
        'semantic_path_latency_p95_at_most_150_ms': bool(
            runtime_metric['available'] and
            runtime_metric['semantic_path_latency_p95_ms'] <= 150.0),
        'stable_for_at_least_30_minutes': bool(
            runtime_metric['available'] and
            runtime_metric['long_duration_pass']),
        'no_drops_or_unbounded_growth': bool(
            runtime_metric['available'] and runtime_metric['drops'] == 0 and
            runtime_metric['bounded_growth_pass']),
        'resource_profile_available': resource_metric['available'],
        'cuda_reserved_memory_p95_at_most_1536_mib': bool(
            resource_metric['available'] and
            resource_metric['cuda_reserved_memory_p95_mib'] <= 1536.0),
        'deterministic_replay': deterministic_replay_passed,
        'all_twelve_scenarios_covered': not missing,
        'human_tracking_regression': human_tracking_regression_passed,
    }
    passed = all(gates.values())
    availability = [
        identity['available'], association['available'], position['available'],
        reidentification['available'], stale_reactivation['available'],
        task_ranking['available'], memory_lifetime['available'],
        runtime_metric['available'], resource_metric['available'],
        task_ranking['threshold_calibrated'], task_ranking['available'],
        not missing, deterministic_replay_passed,
        human_tracking_regression_passed,
    ]
    reasons = []
    if not evaluated_annotations:
        reasons.append('annotations_unavailable')
    if not matched_predictions:
        reasons.append('matched_predictions_unavailable')
    if not runtime_metric['available']:
        reasons.append('runtime_profile_unavailable')
    if not resource_metric['available']:
        reasons.append('resource_profile_unavailable')
    if not task_ranking['available']:
        reasons.append('task_evaluation_evidence_unavailable')
    if not task_ranking['threshold_calibrated']:
        reasons.append('task_threshold_calibration_unavailable')
    if missing:
        reasons.append('scenario_evidence_incomplete')
    if not deterministic_replay_passed:
        reasons.append('deterministic_replay_not_proven')
    if not human_tracking_regression_passed:
        reasons.append('human_tracking_regression_not_proven')
    status = 'passed' if passed else ('unavailable' if not all(availability) else 'failed')
    return {
        'schema_version': SCHEMA_VERSION,
        'dataset_id': dataset_id,
        'manifest_sha256': manifest_sha256,
        'software_revision': software_revision,
        'status': status,
        'passed': passed,
        'evidence': {
            'annotation_count': len(annotations),
            'evaluated_annotation_count': len(evaluated_annotations),
            'prediction_count': len(predictions),
            'matched_prediction_count': len(matched_predictions),
            'annotations_sha256': _record_hash(annotations),
            'predictions_sha256': _record_hash(predictions),
            'task_threshold_calibration_dataset_id': (
                task_threshold_calibration.get('dataset_id')
                if isinstance(task_threshold_calibration, dict) else None),
            'task_threshold_calibration_samples_sha256': (
                task_threshold_calibration.get('samples_sha256')
                if isinstance(task_threshold_calibration, dict) else None),
        },
        'metrics': {
            'identity': identity,
            'association': association,
            'position': position,
            'reidentification': reidentification,
            'stale_reactivation': stale_reactivation,
            'memory_lifetime': memory_lifetime,
            'task_ranking': task_ranking,
            'runtime': runtime_metric,
            'resources': resource_metric,
        },
        'covered_scenarios': covered,
        'missing_scenarios': missing,
        'gates': gates,
        'reasons': reasons,
    }
