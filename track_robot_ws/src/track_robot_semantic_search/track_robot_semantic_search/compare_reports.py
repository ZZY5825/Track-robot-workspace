import argparse
import json
import math
import re
import sys
from pathlib import Path

from .evaluation import compute_hard_gates
from .manifest import ManifestError, load_manifest, sha256_file


_REPORT_KEYS = {
    'schema_version', 'dataset_id', 'manifest_sha256',
    'manifest_capabilities', 'run', 'artifacts', 'coverage',
    'topic_metrics', 'synchronization', 'latency_metrics', 'localization',
    'semantic_counts', 'resources', 'safety', 'gates', 'passed',
}
_RUN_KEYS = {
    'run_id', 'phase', 'replay_rate', 'timing_policy',
    'wall_duration_sec', 'target_source_duration_sec',
    'minimum_source_coverage_ratio', 'freshness_time_base',
}
_CAPABILITY_KEYS = {
    'camera', 'lidar', 'imu', 'local_pose', 'world_pose',
    'query_events', 'annotations', 'active_motion',
}
_HARD_GATES = {
    'required_topic_window_complete',
    'sync_p95_at_most_80_ms',
    'manifest_localization_mode_respected',
    'replay_rate_consistent',
    'no_forward_permission',
}
_FORMAL_DURATIONS = {0.5: 90.0, 1.0: 45.0, 2.0: 22.5}
_SHA256_PATTERN = re.compile(r'[0-9a-f]{64}')
_MAX_NUMBER = 1e308
_MISSING = object()
_PROVENANCE_FIELDS = (
    ('dataset_id', ('dataset_id',)),
    ('artifacts.software_revision', ('artifacts', 'software_revision')),
    ('artifacts.config_sha256', ('artifacts', 'config_sha256')),
    ('artifacts.model_exports', ('artifacts', 'model_exports')),
    ('coverage.accuracy', ('coverage', 'accuracy')),
    ('coverage.identity', ('coverage', 'identity')),
    ('coverage.active_search', ('coverage', 'active_search')),
)


def _value_at(report, keys):
    value = report
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return _MISSING
        value = value[key]
    return value


def _finite_number(value, minimum=None, positive=False):
    if type(value) is int:
        finite = abs(value) <= _MAX_NUMBER
    elif type(value) is float:
        finite = math.isfinite(value) and abs(value) <= _MAX_NUMBER
    else:
        return False
    if not finite:
        return False
    if positive:
        return value > 0
    return minimum is None or value >= minimum


def _nonnegative_integer(value):
    return (
        type(value) is int and value >= 0 and
        abs(value) <= _MAX_NUMBER)


def _numbers_are_finite(value):
    if type(value) in (int, float):
        return _finite_number(value)
    if isinstance(value, dict):
        return all(_numbers_are_finite(item) for item in value.values())
    if isinstance(value, list):
        return all(_numbers_are_finite(item) for item in value)
    return True


def _exact_mapping(value, keys, name, errors):
    if not isinstance(value, dict):
        errors.append('{} must be an object'.format(name))
        return False
    if set(value) != set(keys):
        errors.append('{} has an invalid field set'.format(name))
        return False
    return True


def _expected_topic_rate(count, span):
    if count < 2 or span == 0.0:
        return 0.0
    try:
        rate = round((count - 1) / span, 3)
    except (ArithmeticError, OverflowError):
        return None
    return rate if _finite_number(rate, minimum=0.0) else None


def _validate_topic_metrics(metrics, errors):
    if not isinstance(metrics, dict):
        errors.append('topic_metrics must be an object')
        return
    fields = {
        'count', 'source_start_ns', 'source_end_ns', 'source_span_sec',
        'receive_span_sec', 'source_rate_hz', 'receive_rate_hz',
        'source_sequence_sha256',
    }
    for name, values in metrics.items():
        label = 'topic_metrics.{}'.format(name)
        if not isinstance(name, str) or not name:
            errors.append('topic metric names must be non-empty strings')
            continue
        if not _exact_mapping(values, fields, label, errors):
            continue
        count_valid = _nonnegative_integer(values['count'])
        if not count_valid:
            errors.append('{}.count must be a non-negative integer'.format(
                label))
        stamps_valid = True
        for field in ('source_start_ns', 'source_end_ns'):
            if not _nonnegative_integer(values[field]):
                stamps_valid = False
                errors.append('{}.{} must be a non-negative integer'.format(
                    label, field))
        measurements_valid = True
        for field in (
                'source_span_sec', 'receive_span_sec', 'source_rate_hz',
                'receive_rate_hz'):
            if not _finite_number(values[field], minimum=0.0):
                measurements_valid = False
                errors.append('{}.{} must be finite and non-negative'.format(
                    label, field))
        digest = values['source_sequence_sha256']
        if not isinstance(digest, str) or not _SHA256_PATTERN.fullmatch(digest):
            errors.append('{}.source_sequence_sha256 is invalid'.format(
                label))
        if not (count_valid and stamps_valid and measurements_valid):
            continue

        start = values['source_start_ns']
        end = values['source_end_ns']
        if end < start:
            errors.append(
                '{}.source_end_ns precedes source_start_ns'.format(label))
            continue
        expected_span = round((end - start) / 1000000000.0, 9)
        if values['source_span_sec'] != expected_span:
            errors.append(
                '{}.source_span_sec is inconsistent with timestamps'.format(
                    label))
        if values['count'] < 2 and any(
                values[field] != 0.0 for field in (
                    'source_span_sec', 'receive_span_sec', 'source_rate_hz',
                    'receive_rate_hz')):
            errors.append(
                '{} spans and rates must be zero below two samples'.format(
                    label))
        for span_field, rate_field in (
                ('source_span_sec', 'source_rate_hz'),
                ('receive_span_sec', 'receive_rate_hz')):
            expected_rate = _expected_topic_rate(
                values['count'], values[span_field])
            if expected_rate is None or values[rate_field] != expected_rate:
                errors.append(
                    '{}.{} is inconsistent with count and {}'.format(
                        label, rate_field, span_field))


def _validate_series_map(values, name, fields, errors, count=False):
    if not isinstance(values, dict):
        errors.append('{} must be an object'.format(name))
        return
    for series_name, series in values.items():
        label = '{}.{}'.format(name, series_name)
        if not isinstance(series_name, str) or not series_name:
            errors.append('{} names must be non-empty strings'.format(name))
            continue
        if not _exact_mapping(series, fields, label, errors):
            continue
        if count and (
                not _nonnegative_integer(series['count']) or
                series['count'] == 0):
            errors.append('{}.count must be a positive integer'.format(label))
        for field in fields - {'count'}:
            if not _finite_number(series[field], minimum=0.0):
                errors.append('{}.{} must be finite and non-negative'.format(
                    label, field))


def _validate_report(report):
    if not isinstance(report, dict):
        return ['top level must be an object']
    errors = []
    if set(report) != _REPORT_KEYS:
        errors.append('top level has an invalid field set')
    if not _numbers_are_finite(report):
        errors.append('all numeric values must be finite and bounded')
    if report.get('schema_version') != '1.1.0':
        errors.append('schema_version must be 1.1.0')
    if not isinstance(report.get('dataset_id'), str) or not \
            report.get('dataset_id'):
        errors.append('dataset_id must be a non-empty string')
    manifest_sha256 = report.get('manifest_sha256')
    if not isinstance(manifest_sha256, str) or not \
            _SHA256_PATTERN.fullmatch(manifest_sha256):
        errors.append('manifest_sha256 must be lowercase SHA-256')

    capabilities = report.get('manifest_capabilities')
    if _exact_mapping(
            capabilities, _CAPABILITY_KEYS, 'manifest_capabilities', errors):
        if any(type(value) is not bool for value in capabilities.values()):
            errors.append('manifest capabilities must be boolean')

    run = report.get('run')
    if _exact_mapping(run, _RUN_KEYS, 'run', errors):
        if not isinstance(run['run_id'], str) or not run['run_id']:
            errors.append('run.run_id must be a non-empty string')
        if run['phase'] != 'phase0':
            errors.append('run.phase must be phase0')
        for field in (
                'replay_rate', 'wall_duration_sec',
                'target_source_duration_sec'):
            if not _finite_number(run[field], positive=True):
                errors.append('run.{} must be positive and finite'.format(
                    field))
        if not _finite_number(
                run['minimum_source_coverage_ratio'], minimum=0.0):
            errors.append('run minimum coverage must be finite')
        if not isinstance(run['timing_policy'], str):
            errors.append('run.timing_policy must be a string')
        if not isinstance(run['freshness_time_base'], str):
            errors.append('run.freshness_time_base must be a string')

    artifacts = report.get('artifacts')
    artifact_keys = {'software_revision', 'config_sha256', 'model_exports'}
    if _exact_mapping(artifacts, artifact_keys, 'artifacts', errors):
        if not isinstance(artifacts['software_revision'], str) or not \
                artifacts['software_revision']:
            errors.append('artifacts.software_revision must be non-empty')
        config = artifacts['config_sha256']
        if not isinstance(config, str) or not _SHA256_PATTERN.fullmatch(config):
            errors.append('artifacts.config_sha256 is invalid')
        if artifacts['model_exports'] != []:
            errors.append('artifacts.model_exports must be empty')

    coverage = report.get('coverage')
    expected_coverage = {
        'accuracy': 'not_applicable_phase0_no_model',
        'identity': 'not_applicable_phase0_no_tracker',
        'active_search': 'not_applicable_phase0_passive_only',
    }
    if _exact_mapping(coverage, expected_coverage, 'coverage', errors):
        for name, expected in expected_coverage.items():
            if coverage[name] != expected:
                errors.append('coverage.{} has an invalid value'.format(name))

    _validate_topic_metrics(report.get('topic_metrics'), errors)
    synchronization = report.get('synchronization')
    sync_keys = {'pair_count', 'p50_sec', 'p95_sec', 'maximum_sec'}
    if _exact_mapping(
            synchronization, sync_keys, 'synchronization', errors):
        if not _nonnegative_integer(synchronization['pair_count']):
            errors.append('synchronization.pair_count is invalid')
        for field in ('p50_sec', 'p95_sec', 'maximum_sec'):
            value = synchronization[field]
            if value is not None and not _finite_number(value, minimum=0.0):
                errors.append('synchronization.{} is invalid'.format(field))

    _validate_series_map(
        report.get('latency_metrics'), 'latency_metrics',
        {'count', 'mean_sec', 'p95_sec', 'maximum_sec'}, errors,
        count=True)
    localization = report.get('localization')
    localization_keys = {
        'mode_counts', 'mode_sequence', 'epoch_ids', 'epoch_id_sequence',
        'epochs_valid',
    }
    if _exact_mapping(
            localization, localization_keys, 'localization', errors):
        mode_counts = localization['mode_counts']
        if not isinstance(mode_counts, dict) or any(
                mode not in {
                    'OBSERVATION_ONLY', 'LOCAL_SESSION', 'WORLD', 'UNKNOWN'} or
                not _nonnegative_integer(count)
                for mode, count in mode_counts.items()):
            errors.append('localization.mode_counts is invalid')
        allowed_modes = {
            'OBSERVATION_ONLY', 'LOCAL_SESSION', 'WORLD', 'UNKNOWN'}
        if not isinstance(localization['mode_sequence'], list) or any(
                mode not in allowed_modes
                for mode in localization['mode_sequence']):
            errors.append('localization.mode_sequence is invalid')
        for field in ('epoch_ids', 'epoch_id_sequence'):
            values = localization[field]
            if not isinstance(values, list) or any(
                    type(value) is not int or value <= 0
                    for value in values):
                errors.append('localization.{} is invalid'.format(field))
        if type(localization['epochs_valid']) is not bool:
            errors.append('localization.epochs_valid must be boolean')

    semantic = report.get('semantic_counts')
    semantic_keys = {'regions', 'observations', 'tracked_objects'}
    if _exact_mapping(semantic, semantic_keys, 'semantic_counts', errors):
        if any(not _nonnegative_integer(value)
               for value in semantic.values()):
            errors.append('semantic_counts values must be non-negative')

    resources = report.get('resources')
    resource_keys = {
        'evaluator_cpu_percent', 'evaluator_rss_mb', 'system_cpu_percent',
        'system_ram_used_mb', 'tegrastats',
    }
    if _exact_mapping(resources, resource_keys, 'resources', errors):
        for name in resource_keys - {'tegrastats'}:
            series = resources[name]
            if series:
                _validate_series_map(
                    {name: series}, 'resources',
                    {'mean', 'p95', 'maximum'}, errors)
            elif not isinstance(series, dict):
                errors.append('resources.{} must be an object'.format(name))
        tegrastats = resources['tegrastats']
        allowed_tegrastats = {
            'ram_used_mb', 'gpu_utilization_percent', 'cpu_temperature_c',
            'gpu_temperature_c', 'input_power_mw',
        }
        if not isinstance(tegrastats, dict) or any(
                name not in allowed_tegrastats for name in tegrastats):
            errors.append('resources.tegrastats has invalid fields')
        elif tegrastats:
            _validate_series_map(
                tegrastats, 'resources.tegrastats',
                {'mean', 'p95', 'maximum'}, errors)

    safety = report.get('safety')
    safety_keys = {'motion_intent_count', 'forward_permission_violations'}
    if _exact_mapping(safety, safety_keys, 'safety', errors):
        if any(not _nonnegative_integer(value) for value in safety.values()):
            errors.append('safety values must be non-negative integers')

    gates = report.get('gates')
    if _exact_mapping(gates, _HARD_GATES, 'gates', errors):
        if any(type(value) is not bool for value in gates.values()):
            errors.append('gates values must be boolean')
    if type(report.get('passed')) is not bool:
        errors.append('passed must be boolean')
    return errors


def _same_values(reports, keys):
    values = [_value_at(report, keys) for _, report in reports]
    return (
        len(values) == 3 and
        all(value is not _MISSING for value in values) and
        all(value == values[0] for value in values[1:])
    )


def _safe_replay_rate(report):
    value = _value_at(report, ('run', 'replay_rate'))
    return value if _finite_number(value, positive=True) else None


def _safe_source_hashes(report):
    metrics = report.get('topic_metrics') if isinstance(report, dict) else None
    if not isinstance(metrics, dict):
        return {}
    return {
        name: values.get('source_sequence_sha256')
        if isinstance(values, dict) else None
        for name, values in sorted(metrics.items())
        if isinstance(name, str)
    }


def _ordered_reports(reports):
    return sorted(
        reports,
        key=lambda item: (
            _safe_replay_rate(item[1]) is None,
            _safe_replay_rate(item[1]) or 0.0,
            item[0],
        ),
    )


def compare(manifest_path, paths):
    """Validate exactly three formal reports against one manifest."""
    failures = []
    if not isinstance(paths, list):
        requested_paths = []
        failures.append('report paths must be a list')
    else:
        requested_paths = [str(path) for path in paths]
    if len(requested_paths) != 3:
        failures.append('exactly three report paths are required')

    resolved_paths = []
    for path in requested_paths:
        try:
            resolved_paths.append(str(Path(path).resolve()))
        except (OSError, RuntimeError, ValueError) as error:
            failures.append('{}: invalid report path: {}'.format(path, error))
    if len(set(resolved_paths)) != len(resolved_paths):
        failures.append('duplicate report path')

    manifest = None
    manifest_sha256 = None
    try:
        manifest = load_manifest(Path(manifest_path))
        manifest_sha256 = sha256_file(Path(manifest_path))
    except (
            OSError, UnicodeError, json.JSONDecodeError, ManifestError,
            TypeError, ValueError) as error:
        failures.append('could not load manifest: {}'.format(error))

    reports = []
    for path in requested_paths:
        try:
            with Path(path).open('r', encoding='utf-8') as stream:
                report = json.load(stream)
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            failures.append(
                '{}: could not load report: {}'.format(path, error))
            continue
        reports.append((path, report))
    if len(reports) != 3:
        failures.append('exactly three successfully parsed reports required')

    recomputed_gates = {}
    for path, report in reports:
        for error in _validate_report(report):
            failures.append('{}: malformed report: {}'.format(path, error))
        if manifest is None or not isinstance(report, dict):
            continue
        if report.get('dataset_id') != manifest['dataset_id']:
            failures.append('{}: dataset_id differs from manifest'.format(
                path))
        if report.get('manifest_sha256') != manifest_sha256:
            failures.append('{}: manifest checksum mismatch'.format(path))
        if report.get('manifest_capabilities') != manifest['capabilities']:
            failures.append('{}: manifest capabilities mismatch'.format(path))
        gates = compute_hard_gates(report, manifest['capabilities'])
        recomputed_gates[path] = gates
        if report.get('gates') != gates:
            failures.append('{}: stored gates differ from recomputation'.format(
                path))
        recomputed_passed = all(gates.values())
        if report.get('passed') is not recomputed_passed:
            failures.append('{}: stored passed differs from recomputation'.format(
                path))
        if not recomputed_passed:
            failures.append('{}: recomputed gates failed'.format(path))
        safety = report.get('safety')
        if (isinstance(safety, dict) and
                type(safety.get('forward_permission_violations')) is int and
                safety['forward_permission_violations'] > 0):
            failures.append('{}: forward permission violation'.format(path))

        run = report.get('run')
        if not isinstance(run, dict):
            continue
        rate = _safe_replay_rate(report)
        if run.get('timing_policy') != 'foxy_wall_time_scaled':
            failures.append('{}: invalid timing policy'.format(path))
        if rate in _FORMAL_DURATIONS and \
                run.get('wall_duration_sec') != _FORMAL_DURATIONS[rate]:
            failures.append('{}: invalid wall duration'.format(path))
        if run.get('target_source_duration_sec') != 45.0:
            failures.append('{}: invalid target source duration'.format(path))
        if run.get('minimum_source_coverage_ratio') != 0.90:
            failures.append('{}: invalid coverage ratio'.format(path))
        if run.get('freshness_time_base') != 'arrival_monotonic':
            failures.append('{}: invalid freshness time base'.format(path))

    rates = [_safe_replay_rate(report) for _, report in reports]
    if len(rates) != 3 or set(rates) != set(_FORMAL_DURATIONS):
        failures.append('reports must use exact unique formal rate set')
    for label, keys in _PROVENANCE_FIELDS:
        if not _same_values(reports, keys):
            failures.append('reports use different {} values'.format(label))

    ordered = _ordered_reports(reports)
    ordered_gates = {
        path: recomputed_gates[path]
        for path, _ in ordered if path in recomputed_gates
    }
    all_recomputed_pass = (
        len(ordered_gates) == 3 and
        all(all(gates.values()) for gates in ordered_gates.values())
    )
    return {
        'reports': [path for path, _ in ordered],
        'dataset_id': manifest.get('dataset_id')
        if isinstance(manifest, dict) else None,
        'replay_rates': {
            path: _safe_replay_rate(report) for path, report in ordered},
        'source_sequence_hashes': {
            path: _safe_source_hashes(report) for path, report in ordered},
        'recomputed_gates': ordered_gates,
        'failures': failures,
        'passed': not failures and all_recomputed_pass,
    }


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('reports', nargs='+', type=Path)
    arguments = parser.parse_args(argv)
    result = compare(arguments.manifest, arguments.reports)
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
