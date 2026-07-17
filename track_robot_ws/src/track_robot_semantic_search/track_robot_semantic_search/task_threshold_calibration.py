import hashlib
import json
import math


SCHEMA_VERSION = '1.0.0'
MAXIMUM_SAMPLES = 100_000
MINIMUM_POSITIVE_SAMPLES = 30
MINIMUM_HARD_NEGATIVE_SAMPLES = 30
MINIMUM_RECALL = 0.90
MAXIMUM_FALSE_CONFIRMATION_RATE = 0.05


def _ratio(numerator, denominator):
    return None if denominator == 0 else numerator / denominator


def _canonical_samples(samples):
    return sorted(
        samples,
        key=lambda row: (row['query_id'], row['candidate_id']))


def _samples_hash(samples):
    encoded = json.dumps(
        _canonical_samples(samples), sort_keys=True, separators=(',', ':'),
        ensure_ascii=False, allow_nan=False).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def _validate_samples(dataset_id, samples):
    if not isinstance(dataset_id, str) or not dataset_id:
        raise ValueError('dataset_id must be non-empty')
    if not isinstance(samples, list):
        raise ValueError('calibration samples must be an array')
    if len(samples) > MAXIMUM_SAMPLES:
        raise ValueError('calibration samples exceed bounded maximum 100000')
    identities = set()
    for row in samples:
        if not isinstance(row, dict):
            raise ValueError('calibration sample must be an object')
        if row.get('dataset_id') != dataset_id:
            raise ValueError('sample dataset_id does not match calibration')
        if row.get('split') != 'calibration':
            raise ValueError('task threshold requires calibration split')
        query_id = row.get('query_id')
        candidate_id = row.get('candidate_id')
        relevant = row.get('task_relevant')
        score = row.get('relevance_score')
        if (not isinstance(query_id, int) or isinstance(query_id, bool) or
                query_id <= 0 or not isinstance(candidate_id, str) or
                not candidate_id):
            raise ValueError('calibration candidate identity is invalid')
        if not isinstance(relevant, bool):
            raise ValueError('task_relevant must be boolean')
        if (not isinstance(score, (int, float)) or isinstance(score, bool) or
                not math.isfinite(score) or score < 0.0 or score > 1.0):
            raise ValueError('relevance score must be finite in [0, 1]')
        identity = query_id, candidate_id
        if identity in identities:
            raise ValueError('duplicate calibration candidate identity')
        identities.add(identity)


def _metrics(samples, threshold):
    positives = [row for row in samples if row['task_relevant']]
    negatives = [row for row in samples if not row['task_relevant']]
    true_positive = sum(
        row['relevance_score'] >= threshold for row in positives)
    false_negative = len(positives) - true_positive
    false_positive = sum(
        row['relevance_score'] >= threshold for row in negatives)
    true_negative = len(negatives) - false_positive
    return {
        'true_positive': true_positive,
        'false_positive': false_positive,
        'false_negative': false_negative,
        'true_negative': true_negative,
        'precision': _ratio(true_positive, true_positive + false_positive),
        'recall': _ratio(true_positive, len(positives)),
        'hard_negative_false_confirmation_rate': _ratio(
            false_positive, len(negatives)),
    }


def calibrate_task_threshold(dataset_id, samples):
    _validate_samples(dataset_id, samples)
    ordered = _canonical_samples(samples)
    positive_count = sum(row['task_relevant'] for row in ordered)
    negative_count = len(ordered) - positive_count
    enough_positive = positive_count >= MINIMUM_POSITIVE_SAMPLES
    enough_negative = negative_count >= MINIMUM_HARD_NEGATIVE_SAMPLES

    qualifying = []
    for threshold in sorted(
            {float(row['relevance_score']) for row in ordered}, reverse=True):
        metrics = _metrics(ordered, threshold)
        if (metrics['recall'] is not None and
                metrics['recall'] >= MINIMUM_RECALL and
                metrics['hard_negative_false_confirmation_rate'] is not None and
                metrics['hard_negative_false_confirmation_rate'] <=
                MAXIMUM_FALSE_CONFIRMATION_RATE):
            qualifying.append((threshold, metrics))
    quality_available = bool(qualifying)
    calibrated = enough_positive and enough_negative and quality_available
    selected_threshold = qualifying[0][0] if calibrated else None
    selected_metrics = qualifying[0][1] if calibrated else {
        'true_positive': None,
        'false_positive': None,
        'false_negative': None,
        'true_negative': None,
        'precision': None,
        'recall': None,
        'hard_negative_false_confirmation_rate': None,
    }
    reasons = []
    if not enough_positive:
        reasons.append('positive_samples_below_30')
    if not enough_negative:
        reasons.append('hard_negative_samples_below_30')
    if not quality_available:
        reasons.append('no_threshold_meets_quality_gates')

    return {
        'schema_version': SCHEMA_VERSION,
        'dataset_id': dataset_id,
        'split': 'calibration',
        'samples_sha256': _samples_hash(ordered),
        'sample_count': len(ordered),
        'positive_count': positive_count,
        'hard_negative_count': negative_count,
        'status': 'calibrated' if calibrated else 'insufficient_evidence',
        'selected_threshold': selected_threshold,
        'selected_metrics': selected_metrics,
        'gates': {
            'minimum_30_positive_samples': enough_positive,
            'minimum_30_hard_negative_samples': enough_negative,
            'candidate_recall_at_least_0_90': bool(
                calibrated and selected_metrics['recall'] >= MINIMUM_RECALL),
            'hard_negative_false_confirmation_at_most_0_05': bool(
                calibrated and
                selected_metrics['hard_negative_false_confirmation_rate'] <=
                MAXIMUM_FALSE_CONFIRMATION_RATE),
        },
        'reasons': reasons,
    }
