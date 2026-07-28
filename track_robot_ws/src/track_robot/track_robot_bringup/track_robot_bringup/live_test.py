"""Bounded semantic live-test collection and review artifacts.

The pure report types intentionally do not import ROS, OpenCV, or cv_bridge.
Those runtime dependencies are loaded only when a live collector or overlay is
actually requested, keeping report generation usable in hardware-free tests.
"""

import json
import math
import os
import re
import subprocess
import tempfile
import threading
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path


_QUERY_RESULT = re.compile(
    r'(?:^|\n)ACCEPTED\s+query_id=(\d+)\s+version=(\d+)\b')
_MAX_SCORES = 1024
_ROS_CONTEXT_ENVIRONMENT_KEYS = (
    'ROS_DOMAIN_ID',
    'RMW_IMPLEMENTATION',
    'FASTRTPS_DEFAULT_PROFILES_FILE',
    'ROS_LOCALHOST_ONLY',
    'CYCLONEDDS_URI',
)
_ROS_ENVIRONMENT_LOCK = threading.Lock()
_MISSING = object()


def _utc_timestamp():
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def _stamp_ns(message):
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    try:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None


@contextmanager
def _temporary_ros_environment(environment):
    """Apply only ROS context settings while init/node creation inspects them."""

    environment = dict(environment)
    with _ROS_ENVIRONMENT_LOCK:
        previous = {
            key: os.environ.get(key, _MISSING)
            for key in _ROS_CONTEXT_ENVIRONMENT_KEYS
        }
        try:
            for key in _ROS_CONTEXT_ENVIRONMENT_KEYS:
                if key in environment:
                    os.environ[key] = str(environment[key])
                else:
                    os.environ.pop(key, None)
            yield
        finally:
            for key, value in previous.items():
                if value is _MISSING:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value


def _create_isolated_node(rclpy_api, environment):
    """Initialize one private context without retaining process env changes."""

    context = rclpy_api.context.Context()
    try:
        with _temporary_ros_environment(environment):
            rclpy_api.init(args=None, context=context)
            node = rclpy_api.create_node(
                'semantic_search_live_test_collector',
                context=context,
            )
    except Exception:
        try:
            context.try_shutdown()
        except Exception:
            pass
        raise
    return context, node


@dataclass(frozen=True)
class RegionSample:
    """The bounded scalar data needed to describe and draw one best ROI."""

    query_id: int
    query_version: int
    score: float
    x_offset: int
    y_offset: int
    width: int
    height: int
    stamp_ns: int = None


@dataclass
class Phase2Sample:
    """Bounded Phase 2 aggregate counters and latest localization state."""

    tracklet_messages: int = 0
    tracklet_count: int = 0
    localization_messages: int = 0
    object_messages: int = 0
    object_count: int = 0
    association_messages: int = 0
    association_matches: int = 0
    diagnostic_ranking_messages: int = 0
    diagnostic_candidate_count: int = 0
    calibration_mode: str = 'none'
    latest_memory_mode: int = None
    latest_localization_reason: str = ''


@dataclass
class LiveTestSummary:
    """Bounded live-test state with no unbounded ROS message history."""

    stage: str
    query: str
    frames: int
    nonempty_frames: int
    scores: list
    query_ids: set
    failures: list = field(default_factory=list)
    duration_sec: float = 0.0
    query_versions: set = field(default_factory=set)
    expected_query_id: int = None
    expected_query_version: int = None
    phase2: Phase2Sample = field(default_factory=Phase2Sample)
    readiness_snapshot: dict = field(default_factory=dict)
    warnings: list = field(default_factory=list)
    best_region: RegionSample = None
    latest_image: object = field(default=None, repr=False)
    overlay_image: object = field(default=None, repr=False)
    score_count: int = 0
    score_sum: float = 0.0
    score_minimum: float = None
    score_maximum: float = None
    nonfinite_score_count: int = 0
    collected_at: str = field(default_factory=lambda: datetime.now(
        timezone.utc).replace(microsecond=0).isoformat().replace('+00:00', 'Z'))


@dataclass(frozen=True)
class ReportResult:
    directory: Path
    report_path: Path
    overlay_path: Path = None


@dataclass(frozen=True)
class LiveTestResult:
    exit_code: int
    report_path: Path = None
    overlay_path: Path = None
    error: str = ''

    def __int__(self):
        return self.exit_code


def _score_metrics(summary):
    values = list(summary.scores)
    nonfinite = (
        int(summary.nonfinite_score_count)
        if summary.score_count
        else sum(
            1 for value in values
            if not _is_finite_number(value)
        )
    )
    if nonfinite:
        return {
            'count': int(summary.score_count or len(values)),
            'minimum': None,
            'maximum': None,
            'mean': None,
        }, nonfinite

    if summary.score_count:
        count = int(summary.score_count)
        minimum = summary.score_minimum
        maximum = summary.score_maximum
        mean = summary.score_sum / count if count else None
    else:
        count = len(values)
        minimum = min(values) if values else None
        maximum = max(values) if values else None
        mean = sum(values) / count if count else None
    return {
        'count': count,
        'minimum': minimum,
        'maximum': maximum,
        'mean': mean,
    }, 0


def _is_finite_number(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _query_failures(summary):
    failures = []
    identifiers = {int(value) for value in summary.query_ids}
    versions = {int(value) for value in summary.query_versions}
    if len(identifiers) > 1:
        failures.append('inconsistent query IDs observed: {}'.format(
            sorted(identifiers)))
    if len(versions) > 1:
        failures.append('inconsistent query versions observed: {}'.format(
            sorted(versions)))
    if (summary.expected_query_id is not None
            and identifiers != {int(summary.expected_query_id)}):
        failures.append(
            'observed query IDs {} do not match accepted query ID {}'.format(
                sorted(identifiers), summary.expected_query_id))
    if (summary.expected_query_version is not None
            and versions != {int(summary.expected_query_version)}):
        failures.append(
            'observed query versions {} do not match accepted version {}'.format(
                sorted(versions), summary.expected_query_version))
    return failures


def build_report(summary):
    """Build a strict-JSON-compatible report from bounded aggregate state."""

    failures = [str(value) for value in summary.failures]
    if summary.stage not in ('phase1', 'phase2', 'phase3'):
        failures.append('unsupported live-test stage {!r}'.format(summary.stage))
    if int(summary.frames) <= 0:
        failures.append('no semantic region frames were collected')
    if (int(summary.nonempty_frames) < 0
            or int(summary.nonempty_frames) > int(summary.frames)):
        failures.append('nonempty frame count is outside the frame range')
    failures.extend(_query_failures(summary))

    score, nonfinite_count = _score_metrics(summary)
    if nonfinite_count:
        failures.append('{} non-finite score value(s) observed'.format(
            nonfinite_count))

    frames = int(summary.frames)
    duration = float(summary.duration_sec)
    ratio = (
        float(summary.nonempty_frames) / frames
        if frames > 0 else 0.0
    )
    phase2 = summary.phase2
    if summary.stage in ('phase2', 'phase3'):
        required_streams = (
            ('tracklet_messages', phase2.tracklet_messages, 'tracklet'),
            (
                'localization_messages',
                phase2.localization_messages,
                'localization',
            ),
            ('object_messages', phase2.object_messages, 'object'),
        )
        for _name, count, label in required_streams:
            if int(count) <= 0:
                failures.append(
                    'no Phase 2 {} messages were collected'.format(label))
        if (summary.stage == 'phase3'
                and int(phase2.diagnostic_ranking_messages) <= 0):
            failures.append(
                'no Phase 3 diagnostic ranking messages were collected')
    return {
        'schema_version': 1,
        'stage': summary.stage,
        'query': {
            'text': summary.query,
            'accepted_query_id': summary.expected_query_id,
            'accepted_query_version': summary.expected_query_version,
            'observed_query_ids': sorted(int(v) for v in summary.query_ids),
            'observed_query_versions': sorted(
                int(v) for v in summary.query_versions),
        },
        'collection': {
            'timestamp': summary.collected_at,
            'duration_sec': duration,
        },
        'pipeline': {
            'status': 'FAIL' if failures else 'PASS',
            'failures': failures,
            'warnings': [str(value) for value in summary.warnings],
        },
        'semantic_result': {
            'status': 'REVIEW REQUIRED',
            'ground_truth_available': False,
        },
        'metrics': {
            'frames': frames,
            'nonempty_frames': int(summary.nonempty_frames),
            'nonempty_frame_ratio': ratio,
            'frame_rate_hz': frames / duration if duration > 0.0 else 0.0,
            'score': score,
            'phase2': {
                'tracklet_messages': int(phase2.tracklet_messages),
                'tracklet_count': int(phase2.tracklet_count),
                'localization_messages': int(phase2.localization_messages),
                'object_messages': int(phase2.object_messages),
                'object_count': int(phase2.object_count),
                'association_messages': int(phase2.association_messages),
                'association_matches': int(phase2.association_matches),
                'diagnostic_ranking_messages': int(
                    phase2.diagnostic_ranking_messages),
                'diagnostic_candidate_count': int(
                    phase2.diagnostic_candidate_count),
                'latest_memory_mode': phase2.latest_memory_mode,
                'latest_localization_reason': (
                    phase2.latest_localization_reason),
            },
        },
        'calibration': {
            'mode': phase2.calibration_mode,
            'state': (
                'UNCALIBRATED'
                if summary.stage == 'phase3' else 'NOT_APPLICABLE'),
        },
        'readiness': dict(summary.readiness_snapshot),
    }


def _default_report_directory():
    return (
        Path.home()
        / '.ros'
        / 'track_robot_semantic_search'
        / 'reports'
        / _utc_timestamp()
    )


def _atomic_json(path, payload):
    try:
        encoded = json.dumps(
            payload,
            allow_nan=False,
            indent=2,
            sort_keys=True,
        ) + '\n'
    except (TypeError, ValueError) as error:
        raise ValueError('report is not strict JSON: {}'.format(error)) from error

    descriptor, temporary_name = tempfile.mkstemp(
        prefix='{}.'.format(path.name),
        suffix='.tmp',
        dir=str(path.parent),
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(path))
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _write_overlay(summary, directory):
    if summary.best_region is None:
        summary.warnings.append(
            'overlay not written: no scored semantic region was collected')
        return None
    image_message = summary.overlay_image
    if image_message is None:
        image_message = summary.latest_image
        if image_message is None:
            summary.warnings.append(
                'overlay not written: no sensor image was collected')
            return None
        summary.warnings.append(
            'overlay used the latest image because no matching header stamp '
            'was available')

    try:
        import cv2
        from cv_bridge import CvBridge

        image = CvBridge().imgmsg_to_cv2(
            image_message, desired_encoding='bgr8')
        region = summary.best_region
        start = (int(region.x_offset), int(region.y_offset))
        end = (
            int(region.x_offset + region.width),
            int(region.y_offset + region.height),
        )
        cv2.rectangle(image, start, end, (0, 255, 0), 2)
        cv2.putText(
            image,
            'score={:.3f}'.format(region.score),
            (start[0], max(12, start[1] - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
        )
        overlay_path = directory / 'phase1_overlay.png'
        descriptor, temporary_name = tempfile.mkstemp(
            prefix='phase1_overlay.',
            suffix='.png',
            dir=str(directory),
        )
        os.close(descriptor)
        temporary = Path(temporary_name)
        try:
            if not cv2.imwrite(str(temporary), image):
                raise RuntimeError('OpenCV declined to encode the overlay')
            os.replace(str(temporary), str(overlay_path))
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
        return overlay_path
    except Exception as error:  # Overlay is explicitly best effort.
        summary.warnings.append(
            'overlay not written: {}'.format(error))
        return None


def write_report(summary, output_dir=None):
    """Write best-effort overlay plus strict JSON atomically."""

    directory = (
        Path(output_dir).expanduser()
        if output_dir is not None else _default_report_directory()
    )
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    overlay_path = _write_overlay(summary, directory)
    report_path = directory / 'report.json'
    _atomic_json(report_path, build_report(summary))
    return ReportResult(directory, report_path, overlay_path)


class _BoundedCollector:
    """ROS callbacks retaining bounded aggregates and at most two images."""

    def __init__(self, stage, query, duration_sec, query_id, query_version,
                 calibration_mode='none', readiness_snapshot=None):
        self.summary = LiveTestSummary(
            stage=stage,
            query=query,
            duration_sec=duration_sec,
            frames=0,
            nonempty_frames=0,
            scores=[],
            query_ids=set(),
            query_versions=set(),
            expected_query_id=query_id,
            expected_query_version=query_version,
            phase2=Phase2Sample(calibration_mode=calibration_mode),
            readiness_snapshot=dict(readiness_snapshot or {}),
        )
        self._latest_image_stamp = None

    def image(self, message):
        self.summary.latest_image = message
        self._latest_image_stamp = _stamp_ns(message)
        best = self.summary.best_region
        if (best is not None and best.stamp_ns is not None
                and best.stamp_ns == self._latest_image_stamp):
            self.summary.overlay_image = message

    def regions(self, message):
        summary = self.summary
        summary.frames += 1
        summary.query_ids.add(int(message.query_id))
        summary.query_versions.add(int(message.query_version))
        regions = tuple(message.regions)
        if regions:
            summary.nonempty_frames += 1
        array_stamp = _stamp_ns(message)
        for region in regions:
            summary.query_ids.add(int(region.query_id))
            summary.query_versions.add(int(region.query_version))
            self._score(float(region.fused_score))
            score = float(region.fused_score)
            best = summary.best_region
            if best is None or (
                    _is_finite_number(score)
                    and (not _is_finite_number(best.score)
                         or score > best.score)):
                roi = region.roi
                stamp = _stamp_ns(region)
                if stamp is None:
                    stamp = array_stamp
                summary.best_region = RegionSample(
                    query_id=int(region.query_id),
                    query_version=int(region.query_version),
                    score=score,
                    x_offset=int(roi.x_offset),
                    y_offset=int(roi.y_offset),
                    width=int(roi.width),
                    height=int(roi.height),
                    stamp_ns=stamp,
                )
                summary.overlay_image = (
                    summary.latest_image
                    if stamp is not None and stamp == self._latest_image_stamp
                    else None
                )

    def _score(self, score):
        summary = self.summary
        summary.score_count += 1
        if not _is_finite_number(score):
            summary.nonfinite_score_count += 1
        else:
            summary.score_sum += score
            summary.score_minimum = (
                score if summary.score_minimum is None
                else min(summary.score_minimum, score)
            )
            summary.score_maximum = (
                score if summary.score_maximum is None
                else max(summary.score_maximum, score)
            )
        if len(summary.scores) < _MAX_SCORES:
            summary.scores.append(score)

    def tracklets(self, message):
        sample = self.summary.phase2
        sample.tracklet_messages += 1
        sample.tracklet_count += len(message.tracklets)

    def localization(self, message):
        sample = self.summary.phase2
        sample.localization_messages += 1
        sample.latest_memory_mode = int(message.memory_mode)
        sample.latest_localization_reason = str(message.reason)

    def objects(self, message):
        sample = self.summary.phase2
        sample.object_messages += 1
        sample.object_count += len(message.objects)

    def association(self, message):
        sample = self.summary.phase2
        sample.association_messages += 1
        if int(message.decision) == int(message.DECISION_MATCHED):
            sample.association_matches += 1

    def diagnostic_ranking(self, message):
        sample = self.summary.phase2
        sample.diagnostic_ranking_messages += 1
        sample.diagnostic_candidate_count += len(message.objects)


def collect_live(
        stage,
        query,
        duration_sec,
        query_id,
        query_version,
        *,
        calibration_mode='none',
        readiness_snapshot=None,
        environment=None,
        monotonic=None):
    """Collect ROS messages for a bounded interval using lazy imports."""

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from track_robot_interfaces.msg import SemanticRegionArray

    monotonic = monotonic or time.monotonic
    collector = _BoundedCollector(
        stage,
        query,
        duration_sec,
        query_id,
        query_version,
        calibration_mode,
        readiness_snapshot,
    )
    context, node = _create_isolated_node(
        rclpy,
        os.environ if environment is None else environment,
    )
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        node.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            collector.image,
            qos_profile_sensor_data,
        )
        node.create_subscription(
            SemanticRegionArray,
            '/semantic_search/regions',
            collector.regions,
            10,
        )
        if stage in ('phase2', 'phase3'):
            from track_robot_interfaces.msg import (
                AssociationDebug,
                SemanticLidarTrackletArray,
                SemanticLocalizationState,
                SemanticObjectArray,
            )
            node.create_subscription(
                SemanticLidarTrackletArray,
                '/semantic_memory/lidar_tracklets',
                collector.tracklets,
                10,
            )
            node.create_subscription(
                SemanticLocalizationState,
                '/semantic_memory/localization_state',
                collector.localization,
                10,
            )
            node.create_subscription(
                SemanticObjectArray,
                '/semantic_memory/active_objects',
                collector.objects,
                10,
            )
            node.create_subscription(
                AssociationDebug,
                '/semantic_memory/association_debug',
                collector.association,
                10,
            )
            if stage == 'phase3':
                node.create_subscription(
                    SemanticObjectArray,
                    '/semantic_memory/diagnostic_ranking',
                    collector.diagnostic_ranking,
                    10,
                )

        deadline = monotonic() + float(duration_sec)
        while context.ok():
            remaining = deadline - monotonic()
            if remaining <= 0.0:
                break
            executor.spin_once(timeout_sec=min(0.1, remaining))
    finally:
        try:
            executor.remove_node(node)
            executor.shutdown()
        finally:
            try:
                node.destroy_node()
            finally:
                context.try_shutdown()
    return collector.summary


def _query_result(completed):
    if completed.returncode != 0:
        detail = str(completed.stderr or completed.stdout or '').strip()
        return None, 'semantic query failed: {}'.format(
            detail or 'exit code {}'.format(completed.returncode))
    match = _QUERY_RESULT.search(str(completed.stdout or ''))
    if match is None:
        return None, (
            'semantic query did not return a parseable ACCEPTED '
            'query_id/version result')
    query_id, query_version = (int(value) for value in match.groups())
    if query_id <= 0 or query_version <= 0:
        return None, 'semantic query returned invalid query identifiers'
    return (query_id, query_version), ''


def run_live_test(
        stage,
        query,
        duration_sec,
        output_dir,
        environment,
        *,
        runner=None,
        collector=None,
        query_timeout=7.5,
        calibration_mode='none',
        readiness_snapshot=None):
    """Submit one accepted query, collect bounded data, and write artifacts."""

    if stage not in ('phase1', 'phase2', 'phase3'):
        return LiveTestResult(
            4, error='stage must be phase1, phase2, or phase3')
    try:
        duration = float(duration_sec)
    except (TypeError, ValueError):
        return LiveTestResult(4, error='duration must be a finite positive number')
    if not math.isfinite(duration) or duration <= 0.0:
        return LiveTestResult(4, error='duration must be a finite positive number')

    runner = runner or subprocess.run
    collector = collector or collect_live
    command = [
        'ros2',
        'run',
        'track_robot_semantic_search',
        'semantic_search_query',
        str(query),
    ]
    try:
        completed = runner(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=float(query_timeout),
            env=dict(environment),
            shell=False,
        )
    except subprocess.TimeoutExpired:
        return LiveTestResult(
            4, error='semantic query timed out after {} seconds'.format(
                query_timeout))
    except OSError as error:
        return LiveTestResult(4, error='semantic query failed: {}'.format(error))

    accepted, error = _query_result(completed)
    if accepted is None:
        return LiveTestResult(4, error=error)
    query_id, query_version = accepted
    try:
        summary = collector(
            stage=stage,
            query=str(query),
            duration_sec=duration,
            query_id=query_id,
            query_version=query_version,
            calibration_mode=calibration_mode,
            readiness_snapshot=readiness_snapshot,
            environment=environment,
        )
        written = write_report(summary, output_dir)
    except KeyboardInterrupt:
        raise
    except Exception as error:
        return LiveTestResult(4, error='live collection failed: {}'.format(error))
    status = build_report(summary)['pipeline']['status']
    return LiveTestResult(
        0 if status == 'PASS' else 4,
        report_path=written.report_path,
        overlay_path=written.overlay_path,
    )


__all__ = [
    'LiveTestResult',
    'LiveTestSummary',
    'Phase2Sample',
    'RegionSample',
    'ReportResult',
    'build_report',
    'collect_live',
    'run_live_test',
    'write_report',
]
