import math
from typing import Dict, List, Mapping, Optional, Sequence

from .evaluation import percentile


BASELINES = (
    ('baseline_1_fixed_detector', 'fixed_detector'),
    ('baseline_2_lidar_geometry', 'lidar_geometry'),
    ('baseline_3_language_camera', 'language_camera'),
)
STATUS_VALUES = {'passed', 'failed', 'unavailable', 'not_evaluated'}
CAPABILITY_KEYS = {
    'camera', 'lidar', 'imu', 'local_pose', 'world_pose',
    'query_events', 'annotations', 'active_motion',
}


def _finite(value, field: str, minimum: Optional[float] = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('{} must be finite'.format(field))
    number = float(value)
    if not math.isfinite(number) or (
            minimum is not None and number < minimum):
        raise ValueError('{} must be finite'.format(field))
    return number


def _validate_capabilities(capabilities: Mapping[str, object]) -> Dict[str, bool]:
    if not isinstance(capabilities, Mapping) or (
            set(capabilities) != CAPABILITY_KEYS):
        raise ValueError('manifest_capabilities have an invalid shape')
    if any(not isinstance(value, bool) for value in capabilities.values()):
        raise ValueError('manifest capabilities must be boolean')
    return dict(capabilities)


def build_baseline_report(
        baseline_id: str,
        dataset_id: str,
        manifest_sha256: str,
        manifest_capabilities: Mapping[str, object],
        records: Sequence[Mapping[str, object]],
        software_revision: str,
        model_evidence: Optional[Mapping[str, object]] = None) -> Dict[str, object]:
    baseline_map = dict(BASELINES)
    if baseline_id not in baseline_map:
        raise ValueError('unknown baseline_id: {}'.format(baseline_id))
    if not isinstance(dataset_id, str) or not dataset_id.strip():
        raise ValueError('dataset_id must not be empty')
    if not isinstance(manifest_sha256, str) or len(manifest_sha256) != 64 or (
            any(character not in '0123456789abcdef' for character in manifest_sha256)):
        raise ValueError('manifest_sha256 must be lowercase SHA-256')
    capabilities = _validate_capabilities(manifest_capabilities)
    if not isinstance(software_revision, str) or not software_revision.strip():
        raise ValueError('software_revision must not be empty')

    expected_kind = baseline_map[baseline_id]
    filtered = []
    latencies = []
    labelled = []
    rates = []
    for index, item in enumerate(records):
        if not isinstance(item, Mapping):
            raise ValueError('record {} must be an object'.format(index))
        if item.get('kind') != expected_kind:
            continue
        score = _finite(item.get('score'), 'score')
        latency = _finite(item.get('latency_ms'), 'latency_ms', minimum=0.0)
        correct = item.get('correct')
        if correct is not None and not isinstance(correct, bool):
            raise ValueError('correct must be boolean when present')
        output_rate = item.get('output_rate_hz')
        if output_rate is not None:
            rates.append(_finite(
                output_rate, 'output_rate_hz', minimum=0.0))
        filtered.append((score, correct))
        latencies.append(latency)
        if correct is not None:
            labelled.append(correct)

    evidence = dict(model_evidence or {})
    if baseline_id in (
            'baseline_1_fixed_detector', 'baseline_3_language_camera'):
        model_available = evidence.get('available') is True
        model = {
            'required': True,
            'available': model_available,
            'encoder_id': str(evidence.get('encoder_id', '')),
            'checkpoint_id': str(evidence.get('checkpoint_id', '')),
            'licence': str(evidence.get('licence', 'not reviewed')),
        }
    else:
        model_available = True
        model = {
            'required': False,
            'available': True,
            'encoder_id': 'not_applicable',
            'checkpoint_id': 'not_applicable',
            'licence': 'not_applicable',
        }

    reasons: List[str] = []
    if not filtered:
        reasons.append('observations_unavailable')
    if not model_available:
        reasons.append('model_unavailable')
    annotations_available = capabilities['annotations'] and bool(labelled)
    if filtered and not annotations_available:
        reasons.append('annotations_unavailable')

    precision = (
        float(sum(1 for value in labelled if value)) / len(labelled)
        if labelled else None)
    latency = {
        'mean': sum(latencies) / len(latencies) if latencies else None,
        'p50': percentile(latencies, 0.50) if latencies else None,
        'p95': percentile(latencies, 0.95) if latencies else None,
        'maximum': max(latencies) if latencies else None,
    }
    output_rate_hz = min(rates) if rates else None
    gates = {
        'runtime_available': bool(filtered) and model_available,
        'annotations_available': annotations_available,
        'precision_at_least_0_85': (
            precision is not None and precision >= 0.85),
        'latency_p95_at_most_150_ms': (
            latency['p95'] is not None and latency['p95'] <= 150.0),
        'semantic_output_at_least_5_hz': (
            output_rate_hz is not None and output_rate_hz >= 5.0),
        'no_motion_output': True,
    }
    gates['runtime_contract_passed'] = all((
        gates['runtime_available'],
        gates['latency_p95_at_most_150_ms'],
        gates['semantic_output_at_least_5_hz'],
        gates['no_motion_output'],
    ))
    if not gates['runtime_available']:
        status = 'unavailable'
    elif not annotations_available:
        status = 'not_evaluated'
    elif gates['precision_at_least_0_85'] and (
            gates['runtime_contract_passed']):
        status = 'passed'
    else:
        status = 'failed'
    report = {
        'schema_version': '1.0.0',
        'phase': 'phase1',
        'baseline_id': baseline_id,
        'status': status,
        'dataset': {
            'dataset_id': dataset_id,
            'manifest_sha256': manifest_sha256,
            'capabilities': capabilities,
        },
        'artifacts': {
            'software_revision': software_revision,
            'model': model,
        },
        'metrics': {
            'observation_count': len(filtered),
            'labelled_count': len(labelled),
            'precision': precision,
            'phrase_region_recall': None,
            'output_rate_hz': output_rate_hz,
            'latency_ms': latency,
        },
        'gates': gates,
        'reasons': reasons,
        'passed': status == 'passed',
    }
    validate_phase1_report(report)
    return report


def validate_phase1_report(report: Mapping[str, object]) -> None:
    if not isinstance(report, Mapping):
        raise ValueError('Phase 1 report must be an object')
    required = {
        'schema_version', 'phase', 'baseline_id', 'status', 'dataset',
        'artifacts', 'metrics', 'gates', 'reasons', 'passed',
    }
    if set(report) != required:
        raise ValueError('Phase 1 report has invalid top-level fields')
    if report['schema_version'] != '1.0.0' or report['phase'] != 'phase1':
        raise ValueError('Phase 1 report version or phase is invalid')
    if report['baseline_id'] not in dict(BASELINES):
        raise ValueError('Phase 1 report baseline_id is invalid')
    if report['status'] not in STATUS_VALUES:
        raise ValueError('Phase 1 report status is invalid')
    if report['passed'] is not (report['status'] == 'passed'):
        raise ValueError('Phase 1 report passed flag is inconsistent')
    dataset = report['dataset']
    if not isinstance(dataset, Mapping):
        raise ValueError('Phase 1 report dataset is invalid')
    _validate_capabilities(dataset.get('capabilities'))
    sha256 = dataset.get('manifest_sha256')
    if not isinstance(sha256, str) or len(sha256) != 64:
        raise ValueError('Phase 1 report manifest SHA is invalid')
    metrics = report['metrics']
    if not isinstance(metrics, Mapping):
        raise ValueError('Phase 1 report metrics are invalid')
    for field in ('observation_count', 'labelled_count'):
        if not isinstance(metrics.get(field), int) or metrics[field] < 0:
            raise ValueError('Phase 1 report {} is invalid'.format(field))
    latency = metrics.get('latency_ms')
    if not isinstance(latency, Mapping):
        raise ValueError('Phase 1 report latency metrics are invalid')
    for value in latency.values():
        if value is not None:
            _finite(value, 'latency metric', minimum=0.0)
    if not isinstance(report['reasons'], list) or any(
            not isinstance(reason, str) for reason in report['reasons']):
        raise ValueError('Phase 1 report reasons are invalid')
