import hashlib
import json
import math
import re
from collections import Counter
from pathlib import Path


REPORT_SCHEMA_VERSION = '1.1.0'
TIMING_POLICY = 'foxy_wall_time_scaled'
MINIMUM_SOURCE_COVERAGE_RATIO = 0.90
REPLAY_RATE_RELATIVE_TOLERANCE = 0.15
_MAX_SCHEMA_NUMBER = 1e308
_SHA256_PATTERN = re.compile(r'[0-9a-f]{64}')
_TEGRASTATS_BOUNDARY_BYTES = 4096
_SENSOR_TOPICS = {'image', 'lidar', 'imu', 'local_pose', 'world_pose'}
_CAPABILITY_TOPICS = (
    ('camera', 'image'),
    ('lidar', 'lidar'),
    ('imu', 'imu'),
    ('local_pose', 'local_pose'),
    ('world_pose', 'world_pose'),
)
_CAPABILITY_KEYS = {
    'camera', 'lidar', 'imu', 'local_pose', 'world_pose',
    'query_events', 'annotations', 'active_motion',
}
_NUMBER_TOKEN = (
    r'[+-]?(?:(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)'
    r'(?:[eE][+-]?[0-9]+)?|nan|inf(?:inity)?)')


def percentile(values, quantile):
    """Return a deterministic linearly interpolated percentile."""
    quantile = float(quantile)
    if not math.isfinite(quantile):
        raise ValueError('percentile inputs must be finite')
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    if any(not math.isfinite(value) for value in ordered):
        raise ValueError('percentile inputs must be finite')
    position = max(0.0, min(1.0, quantile)) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


class TopicSeries:
    """Collect source and evaluator-receive timestamps for one topic."""

    def __init__(self):
        self.source_stamps = []
        self.receive_stamps = []

    def observe(self, source_stamp_ns, receive_stamp_ns):
        self.source_stamps.append(int(source_stamp_ns))
        self.receive_stamps.append(int(receive_stamp_ns))

    @staticmethod
    def _rate(stamps):
        if len(stamps) < 2:
            return 0.0
        duration = (max(stamps) - min(stamps)) / 1000000000.0
        return 0.0 if duration <= 0.0 else (len(stamps) - 1) / duration

    @staticmethod
    def _span(stamps):
        if len(stamps) < 2:
            return 0.0
        return (max(stamps) - min(stamps)) / 1000000000.0

    def report(self):
        serialized = json.dumps(
            self.source_stamps, separators=(',', ':')).encode('utf-8')
        source_start_ns = min(self.source_stamps) \
            if self.source_stamps else 0
        source_end_ns = max(self.source_stamps) \
            if self.source_stamps else 0
        return {
            'count': len(self.source_stamps),
            'source_start_ns': source_start_ns,
            'source_end_ns': source_end_ns,
            'source_span_sec': round(
                self._span(self.source_stamps), 9),
            'receive_span_sec': round(
                self._span(self.receive_stamps), 9),
            'source_rate_hz': round(self._rate(self.source_stamps), 3),
            'receive_rate_hz': round(self._rate(self.receive_stamps), 3),
            'source_sequence_sha256': hashlib.sha256(serialized).hexdigest(),
        }


def _finite_number(value, minimum=None, positive=False):
    if type(value) is int:
        valid = abs(value) <= _MAX_SCHEMA_NUMBER
    elif type(value) is float:
        valid = math.isfinite(value) and abs(value) <= _MAX_SCHEMA_NUMBER
    else:
        return False
    if not valid:
        return False
    if positive:
        return value > 0
    return minimum is None or value >= minimum


def _failed_hard_gates():
    return {
        'required_topic_window_complete': False,
        'sync_p95_at_most_80_ms': False,
        'manifest_localization_mode_respected': False,
        'replay_rate_consistent': False,
        'no_forward_permission': False,
    }


def _required_topics(capabilities):
    if (not isinstance(capabilities, dict) or
            set(capabilities) != _CAPABILITY_KEYS or
            any(type(value) is not bool for value in capabilities.values())):
        return None
    return [
        topic for capability, topic in _CAPABILITY_TOPICS
        if capabilities[capability]
    ]


def _localization_gate(localization, capabilities):
    if not isinstance(localization, dict):
        return False
    modes = localization.get('mode_sequence')
    epoch_ids = localization.get('epoch_id_sequence')
    if (not isinstance(modes, list) or not modes or
            not isinstance(epoch_ids, list) or
            len(epoch_ids) != len(modes) or
            localization.get('epochs_valid') is not True):
        return False
    if any(type(value) is not int or value <= 0 for value in epoch_ids):
        return False
    if any(second < first for first, second in zip(
            epoch_ids, epoch_ids[1:])):
        return False

    ranks = {
        'OBSERVATION_ONLY': 0,
        'LOCAL_SESSION': 1,
        'WORLD': 2,
    }
    try:
        mode_ranks = [ranks[mode] for mode in modes]
    except (KeyError, TypeError):
        return False
    if any(second < first for first, second in zip(
            mode_ranks, mode_ranks[1:])):
        return False
    if not (capabilities['local_pose'] and capabilities['imu']):
        final_rank = 0
    elif capabilities['world_pose']:
        final_rank = 2
    else:
        final_rank = 1
    return (
        mode_ranks[-1] == final_rank and
        all(rank <= final_rank for rank in mode_ranks)
    )


def compute_hard_gates(report, manifest_capabilities):
    """Recompute the exact five report gates from report-only evidence."""
    required_topics = _required_topics(manifest_capabilities)
    if not required_topics or not isinstance(report, dict):
        return _failed_hard_gates()
    run = report.get('run') if isinstance(report, dict) else None
    topic_metrics = report.get('topic_metrics') \
        if isinstance(report, dict) else None
    target = run.get('target_source_duration_sec') \
        if isinstance(run, dict) else None
    coverage = run.get('minimum_source_coverage_ratio') \
        if isinstance(run, dict) else None
    replay_rate = run.get('replay_rate') \
        if isinstance(run, dict) else None
    wall_duration = run.get('wall_duration_sec') \
        if isinstance(run, dict) else None
    if (not _finite_number(wall_duration, positive=True) or
            not _finite_number(replay_rate, positive=True) or
            not _finite_number(target, positive=True) or
            not _finite_number(coverage, minimum=0.0)):
        return _failed_hard_gates()
    try:
        expected_target = wall_duration * replay_rate
    except (ArithmeticError, OverflowError):
        return _failed_hard_gates()
    if not _finite_number(expected_target, positive=True):
        return _failed_hard_gates()
    policy_consistent = (
        target == expected_target and
        coverage == MINIMUM_SOURCE_COVERAGE_RATIO
    )

    window_complete = (
        policy_consistent and
        isinstance(topic_metrics, dict) and
        _finite_number(expected_target, positive=True)
    )
    rate_consistent = (
        isinstance(topic_metrics, dict) and
        _finite_number(replay_rate, positive=True))
    if isinstance(topic_metrics, dict):
        minimum_span = (
            expected_target * MINIMUM_SOURCE_COVERAGE_RATIO)
        for name in required_topics:
            metric = topic_metrics.get(name)
            if not isinstance(metric, dict):
                window_complete = False
                rate_consistent = False
                continue
            count = metric.get('count')
            source_span = metric.get('source_span_sec')
            receive_span = metric.get('receive_span_sec')
            if (type(count) is not int or count < 0 or
                    not _finite_number(count, minimum=0.0) or
                    not _finite_number(source_span, minimum=0.0) or
                    not _finite_number(receive_span, minimum=0.0)):
                return _failed_hard_gates()
            if count < 2 or source_span < minimum_span:
                window_complete = False
            if (not _finite_number(source_span, positive=True) or
                    not _finite_number(receive_span, positive=True)):
                rate_consistent = False
            elif rate_consistent:
                try:
                    observed_rate = source_span / receive_span
                    relative_error = abs(
                        observed_rate - replay_rate) / replay_rate
                except (ArithmeticError, OverflowError):
                    return _failed_hard_gates()
                if (not _finite_number(observed_rate, positive=True) or
                        not _finite_number(relative_error, minimum=0.0)):
                    return _failed_hard_gates()
                if relative_error > REPLAY_RATE_RELATIVE_TOLERANCE:
                    rate_consistent = False

    synchronization = report.get('synchronization') \
        if isinstance(report, dict) else None
    if (required_topics is not None and
            manifest_capabilities['camera'] and
            manifest_capabilities['lidar']):
        sync_gate = (
            isinstance(synchronization, dict) and
            type(synchronization.get('pair_count')) is int and
            synchronization['pair_count'] > 0 and
            _finite_number(synchronization.get('p95_sec'), minimum=0.0) and
            synchronization['p95_sec'] <= 0.08
        )
    else:
        sync_gate = required_topics is not None

    localization_gate = (
        required_topics is not None and
        _localization_gate(report.get('localization'), manifest_capabilities)
    ) if isinstance(report, dict) else False
    safety = report.get('safety') if isinstance(report, dict) else None
    no_forward_permission = (
        isinstance(safety, dict) and
        type(safety.get('forward_permission_violations')) is int and
        safety['forward_permission_violations'] == 0
    )
    return {
        'required_topic_window_complete': bool(window_complete),
        'sync_p95_at_most_80_ms': bool(sync_gate),
        'manifest_localization_mode_respected': bool(localization_gate),
        'replay_rate_consistent': bool(rate_consistent),
        'no_forward_permission': bool(no_forward_permission),
    }


def parse_tegrastats_line(line):
    """Extract report-only measurements from one tegrastats line."""
    patterns = {
        'ram_used_mb': r'RAM\s+(' + _NUMBER_TOKEN + r')/',
        'gpu_utilization_percent': (
            r'GR3D_FREQ\s+(' + _NUMBER_TOKEN + r')%'),
        'cpu_temperature_c': r'CPU@(' + _NUMBER_TOKEN + r')C',
        'gpu_temperature_c': r'GPU@(' + _NUMBER_TOKEN + r')C',
        'input_power_mw': r'VDD_IN\s+(' + _NUMBER_TOKEN + r')mW',
    }
    result = {}
    for name, pattern in patterns.items():
        match = re.search(pattern, str(line), flags=re.IGNORECASE)
        if match:
            value = float(match.group(1))
            if math.isfinite(value) and value >= 0.0:
                result[name] = value
    return result


def _capture_tegrastats_boundary(path):
    """Capture enough identity and boundary data to detect log replacement."""
    path = Path(path)
    try:
        if not path.is_file():
            return {'existed': False}
        metadata = path.stat()
        offset = metadata.st_size
        boundary_start = max(0, offset - _TEGRASTATS_BOUNDARY_BYTES)
        with path.open('rb') as stream:
            stream.seek(boundary_start)
            boundary = stream.read(offset - boundary_start)
    except OSError:
        return {'invalid': True}
    return {
        'existed': True,
        'device': metadata.st_dev,
        'inode': metadata.st_ino,
        'offset': offset,
        'boundary_start': boundary_start,
        'boundary_sha256': hashlib.sha256(boundary).hexdigest(),
    }


def _read_tegrastats_window(path, boundary):
    path = Path(path)
    try:
        if boundary is None:
            return path.read_bytes() if path.is_file() else b''
        if boundary.get('invalid'):
            return b''
        if not boundary.get('existed'):
            return path.read_bytes() if path.is_file() else b''
        if not path.is_file():
            return b''
        metadata = path.stat()
        if (metadata.st_dev != boundary['device'] or
                metadata.st_ino != boundary['inode'] or
                metadata.st_size < boundary['offset']):
            return b''
        with path.open('rb') as stream:
            stream.seek(boundary['boundary_start'])
            previous = stream.read(
                boundary['offset'] - boundary['boundary_start'])
            if hashlib.sha256(previous).hexdigest() != \
                    boundary['boundary_sha256']:
                return b''
            stream.seek(boundary['offset'])
            return stream.read()
    except (KeyError, OSError, TypeError):
        return b''


def summarize_tegrastats(path, boundary=None):
    """Summarize a tegrastats log, returning no data when unavailable."""
    fields = {}
    payload = _read_tegrastats_window(path, boundary)
    for line in payload.decode('utf-8', errors='replace').splitlines():
        for name, value in parse_tegrastats_line(line).items():
            fields.setdefault(name, []).append(value)
    return {
        name: {
            'mean': round(sum(values) / len(values), 3),
            'p95': round(percentile(values, 0.95), 3),
            'maximum': round(max(values), 3),
        }
        for name, values in sorted(fields.items())
        if values
    }


class EvaluationAccumulator:
    """Collect model-independent replay, safety, and resource evidence."""

    def __init__(
            self, manifest, manifest_sha256, run_id, software_revision,
            config_sha256, replay_rate, wall_duration_sec, timing_policy,
            freshness_time_base, tegrastats_path=''):
        self.manifest = manifest
        self.manifest_capabilities = dict(manifest['capabilities'])
        self.manifest_sha256 = str(manifest_sha256)
        self.run_id = str(run_id)
        self.software_revision = str(software_revision)
        self.config_sha256 = str(config_sha256)
        self.replay_rate = float(replay_rate)
        self.wall_duration_sec = float(wall_duration_sec)
        self.timing_policy = str(timing_policy)
        self.freshness_time_base = str(freshness_time_base)
        if not _SHA256_PATTERN.fullmatch(self.manifest_sha256):
            raise ValueError('manifest_sha256 must be lowercase SHA-256')
        if not self.run_id:
            raise ValueError('run_id must be non-empty')
        if not self.software_revision:
            raise ValueError('software_revision must be non-empty')
        if not _SHA256_PATTERN.fullmatch(self.config_sha256):
            raise ValueError('config_sha256 must be lowercase SHA-256')
        if not math.isfinite(self.replay_rate) or self.replay_rate <= 0.0:
            raise ValueError('replay_rate must be positive')
        if (not math.isfinite(self.wall_duration_sec) or
                self.wall_duration_sec <= 0.0):
            raise ValueError('wall_duration_sec must be positive')
        if self.timing_policy not in {
                'online_source_time', TIMING_POLICY}:
            raise ValueError('unsupported timing_policy: {}'.format(
                self.timing_policy))
        if self.freshness_time_base not in {
                'source_clock', 'arrival_monotonic'}:
            raise ValueError('unsupported freshness_time_base: {}'.format(
                self.freshness_time_base))
        if _required_topics(self.manifest_capabilities) is None:
            raise ValueError('manifest capabilities have an invalid shape')
        self.target_source_duration_sec = (
            self.wall_duration_sec * self.replay_rate)
        if not math.isfinite(self.target_source_duration_sec):
            raise ValueError('target source duration must be finite')
        self.tegrastats_path = tegrastats_path
        self.topics = {}
        self.pair_offsets_sec = []
        self.localization_modes = Counter()
        self.localization_mode_sequence = []
        self.localization_epochs = []
        self.localization_epochs_valid = True
        self.forward_permission_violations = 0
        self.motion_intent_count = 0
        self.semantic_region_count = 0
        self.observation_count = 0
        self.tracked_object_count = 0
        self.latencies_sec = {}
        self.process_cpu_percent = []
        self.process_rss_mb = []
        self.system_cpu_percent = []
        self.system_ram_used_mb = []
        self.resource_window_started = False
        self.tegrastats_boundary = None

    def start_resource_window(self):
        if self.resource_window_started:
            return
        self.resource_window_started = True
        self.tegrastats_boundary = _capture_tegrastats_boundary(
            self.tegrastats_path)

    def observe_topic(self, name, source_stamp_ns, receive_stamp_ns):
        name = str(name)
        if name in _SENSOR_TOPICS:
            self.start_resource_window()
        self.topics.setdefault(name, TopicSeries()).observe(
            source_stamp_ns, receive_stamp_ns)

    def observe_pair_offset(self, offset_ns):
        self.pair_offsets_sec.append(abs(int(offset_ns)) / 1000000000.0)

    def observe_localization(self, mode, epoch_id):
        mode = str(mode)
        self.localization_modes[mode] += 1
        self.localization_mode_sequence.append(mode)
        if type(epoch_id) is not int or epoch_id <= 0:
            self.localization_epochs_valid = False
            return
        if (self.localization_epochs and
                epoch_id < self.localization_epochs[-1]):
            self.localization_epochs_valid = False
        self.localization_epochs.append(epoch_id)

    def observe_motion_intent(self, forward_permitted):
        self.motion_intent_count += 1
        if forward_permitted:
            self.forward_permission_violations += 1

    def observe_latency(self, name, duration_sec):
        duration_sec = float(duration_sec)
        if math.isfinite(duration_sec) and duration_sec >= 0.0:
            self.latencies_sec.setdefault(
                str(name), []).append(duration_sec)

    def observe_resource(
            self, process_cpu_percent, process_rss_mb,
            system_cpu_percent, system_ram_used_mb):
        samples = (
            (self.process_cpu_percent, process_cpu_percent),
            (self.process_rss_mb, process_rss_mb),
            (self.system_cpu_percent, system_cpu_percent),
            (self.system_ram_used_mb, system_ram_used_mb),
        )
        for output, value in samples:
            value = float(value)
            if math.isfinite(value) and value >= 0.0:
                output.append(value)

    @staticmethod
    def _resource_series(values):
        if not values:
            return {}
        return {
            'mean': round(sum(values) / len(values), 3),
            'p95': round(percentile(values, 0.95), 3),
            'maximum': round(max(values), 3),
        }

    @staticmethod
    def _latency_series(values):
        return {
            'count': len(values),
            'mean_sec': round(sum(values) / len(values), 6),
            'p95_sec': round(percentile(values, 0.95), 6),
            'maximum_sec': round(max(values), 6),
        }

    def _required_topics(self):
        return _required_topics(self.manifest_capabilities)

    def required_topics_ready(self):
        required_topics = self._required_topics()
        return all(
            name in self.topics and self.topics[name].source_stamps
            for name in required_topics) if required_topics else False

    def finalize(self):
        """Build an evaluation_report.schema.json-compatible payload."""
        capabilities = self.manifest_capabilities
        topic_reports = {
            name: series.report()
            for name, series in sorted(self.topics.items())
        }
        sync_p95 = percentile(self.pair_offsets_sec, 0.95)
        report = {
            'schema_version': REPORT_SCHEMA_VERSION,
            'dataset_id': self.manifest['dataset_id'],
            'manifest_sha256': self.manifest_sha256,
            'manifest_capabilities': dict(capabilities),
            'run': {
                'run_id': self.run_id,
                'phase': 'phase0',
                'replay_rate': self.replay_rate,
                'timing_policy': self.timing_policy,
                'wall_duration_sec': self.wall_duration_sec,
                'target_source_duration_sec':
                self.target_source_duration_sec,
                'minimum_source_coverage_ratio':
                MINIMUM_SOURCE_COVERAGE_RATIO,
                'freshness_time_base': self.freshness_time_base,
            },
            'artifacts': {
                'software_revision': self.software_revision,
                'config_sha256': self.config_sha256,
                'model_exports': [],
            },
            'coverage': {
                'accuracy': 'not_applicable_phase0_no_model',
                'identity': 'not_applicable_phase0_no_tracker',
                'active_search': 'not_applicable_phase0_passive_only',
            },
            'topic_metrics': topic_reports,
            'synchronization': {
                'pair_count': len(self.pair_offsets_sec),
                'p50_sec': percentile(self.pair_offsets_sec, 0.50),
                'p95_sec': sync_p95,
                'maximum_sec': (
                    max(self.pair_offsets_sec)
                    if self.pair_offsets_sec else None),
            },
            'latency_metrics': {
                name: self._latency_series(values)
                for name, values in sorted(self.latencies_sec.items())
            },
            'localization': {
                'mode_counts': dict(sorted(self.localization_modes.items())),
                'mode_sequence': list(self.localization_mode_sequence),
                'epoch_ids': sorted(set(self.localization_epochs)),
                'epoch_id_sequence': list(self.localization_epochs),
                'epochs_valid': self.localization_epochs_valid,
            },
            'semantic_counts': {
                'regions': self.semantic_region_count,
                'observations': self.observation_count,
                'tracked_objects': self.tracked_object_count,
            },
            'resources': {
                'evaluator_cpu_percent': self._resource_series(
                    self.process_cpu_percent),
                'evaluator_rss_mb': self._resource_series(
                    self.process_rss_mb),
                'system_cpu_percent': self._resource_series(
                    self.system_cpu_percent),
                'system_ram_used_mb': self._resource_series(
                    self.system_ram_used_mb),
                'tegrastats': (
                    summarize_tegrastats(
                        self.tegrastats_path, self.tegrastats_boundary)
                    if self.resource_window_started else {}),
            },
            'safety': {
                'motion_intent_count': self.motion_intent_count,
                'forward_permission_violations':
                self.forward_permission_violations,
            },
        }
        gates = compute_hard_gates(report, capabilities)
        report['gates'] = gates
        report['passed'] = all(gates.values())
        return report
