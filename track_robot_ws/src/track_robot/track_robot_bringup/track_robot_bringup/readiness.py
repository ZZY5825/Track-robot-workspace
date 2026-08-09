"""Bounded, ROS-independent semantic-search readiness checks."""

import hashlib
import json
import math
import os
import re
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path

import yaml

from .control_config import managed_environment, resolve_stage


class CheckStatus(Enum):
    PASS = 'PASS'
    NOT_READY = 'NOT READY'
    DEGRADED = 'DEGRADED'
    FAIL = 'FAIL'


@dataclass(frozen=True)
class CheckResult:
    status: CheckStatus
    name: str
    detail: str
    action: str = ''

    @classmethod
    def pass_(cls, name, detail):
        return cls(CheckStatus.PASS, name, detail)

    @classmethod
    def not_ready(cls, name, detail, action=''):
        return cls(CheckStatus.NOT_READY, name, detail, action)

    @classmethod
    def degraded(cls, name, detail, action=''):
        return cls(CheckStatus.DEGRADED, name, detail, action)

    @classmethod
    def fail(cls, name, detail, action=''):
        return cls(CheckStatus.FAIL, name, detail, action)


class ReadinessReport:
    """A deterministic readiness summary suitable for terminals and JSON."""

    _PRIORITY = {
        CheckStatus.PASS: 0,
        CheckStatus.DEGRADED: 1,
        CheckStatus.NOT_READY: 2,
        CheckStatus.FAIL: 3,
    }

    def __init__(self, checks, stage='', timestamp=None, ros_domain_id='20'):
        self.checks = tuple(checks)
        self.stage = stage
        self.timestamp = timestamp or datetime.now(timezone.utc).replace(
            microsecond=0).isoformat().replace('+00:00', 'Z')
        self.ros_domain_id = str(ros_domain_id)

    @property
    def names(self):
        return tuple(check.name for check in self.checks)

    @property
    def overall(self):
        if not self.checks:
            return CheckStatus.PASS
        return max(self.checks, key=lambda check: self._PRIORITY[check.status]).status

    def render_text(self):
        if not self.checks:
            return 'Overall: {}'.format(self.overall.value)
        status_width = max(len(check.status.value) for check in self.checks)
        name_width = max(len(check.name) for check in self.checks)
        lines = []
        for check in self.checks:
            lines.append('{status:<{status_width}}  {name:<{name_width}}  {detail}'.format(
                status=check.status.value,
                status_width=status_width,
                name=check.name,
                name_width=name_width,
                detail=check.detail))
        lines.append('Overall: {}'.format(self.overall.value))
        return '\n'.join(lines)

    def as_dict(self):
        return {
            'stage': self.stage,
            'overall': self.overall.value,
            'checks': [
                {
                    'status': check.status.value,
                    'name': check.name,
                    'detail': check.detail,
                    'action': check.action,
                }
                for check in self.checks
            ],
            'timestamp': self.timestamp,
            'ros_domain_id': self.ros_domain_id,
        }

    def render_json(self):
        return json.dumps(self.as_dict(), sort_keys=True)


class StaticProbe:
    """Check local model, DDS, and calibration inputs without ROS imports."""

    def __init__(self, paths):
        self.paths = dict(paths)

    def _path(self, name):
        value = self.paths.get(name)
        return Path(value) if value not in (None, '') else None

    def _expected_sha256(self, path_key, defaults_key):
        expected = self.paths.get(
            path_key[:-5] + '_sha256'
            if path_key.endswith('_path') else path_key + '_sha256')
        if expected:
            return str(expected)
        defaults_path = self._path('defaults_path')
        if defaults_path is None:
            return ''
        try:
            defaults = yaml.safe_load(defaults_path.read_text(encoding='utf-8'))
            return str(defaults['models'][defaults_key])
        except (KeyError, OSError, TypeError, yaml.YAMLError):
            return ''

    def _directory(self, name, path_key, action):
        path = self._path(path_key)
        if path is not None and path.is_dir():
            return CheckResult.pass_(name, str(path))
        return CheckResult.not_ready(
            name, 'directory is missing: {}'.format(path), action)

    def _checkpoint(self, name, path_key, defaults_key, action):
        path = self._path(path_key)
        if path is None or not path.is_file():
            return CheckResult.not_ready(
                name, 'checkpoint is missing: {}'.format(path), action)
        expected = self._expected_sha256(path_key, defaults_key)
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if not expected:
            return CheckResult.fail(
                name, 'expected SHA-256 is not configured',
                'set {} in semantic_search_defaults.yaml'.format(
                    defaults_key))
        if actual != expected:
            return CheckResult.fail(
                name, 'SHA-256 mismatch for {}'.format(path),
                'restore the expected checkpoint')
        return CheckResult.pass_(name, 'SHA-256 verified')

    @staticmethod
    def _valid_measured_extrinsic(path):
        try:
            config = yaml.safe_load(path.read_text(encoding='utf-8'))
            calibration_id = config['calibration_id']
            if (not isinstance(calibration_id, str) or not calibration_id.strip()
                    or calibration_id == 'replace_with_measured_calibration_id'):
                return None
            if (config['parent_frame'] != 'base_link'
                    or config['child_frame'] != 'zed_camera_link'):
                return None
            translation = config['translation']
            rotation = config['rotation_rpy']
            values = [translation[key] for key in ('x', 'y', 'z')]
            values.extend(rotation[key] for key in ('roll', 'pitch', 'yaw'))
            if not all(math.isfinite(float(value)) for value in values):
                return None
            return calibration_id
        except (KeyError, OSError, TypeError, ValueError, yaml.YAMLError):
            return None

    def _calibration(self):
        mode = self.paths.get('extrinsic_mode', 'none')
        allow_degraded = self.paths.get('allow_degraded', False)
        if isinstance(allow_degraded, str):
            allow_degraded = allow_degraded.strip().lower() in (
                '1', 'true', 'yes', 'on')
        else:
            allow_degraded = bool(allow_degraded)
        if mode == 'prototype':
            if allow_degraded:
                return CheckResult.degraded(
                    'calibration', 'prototype camera extrinsic; not calibrated')
            return CheckResult.fail(
                'calibration', 'prototype camera extrinsic is not permitted',
                'pass allow_degraded:=true only for diagnostic operation')
        if mode == 'none':
            return CheckResult.not_ready(
                'calibration', 'measured camera extrinsic is not configured',
                'provide extrinsic_mode:=measured and an extrinsic_file')
        if mode != 'measured':
            return CheckResult.fail(
                'calibration', 'unknown camera extrinsic mode {!r}'.format(mode),
                'use measured, prototype, or none')

        path = self._path('extrinsic_file')
        if path is None or not path.is_file():
            return CheckResult.not_ready(
                'calibration', 'measured camera extrinsic file is missing: {}'.format(path),
                'provide a measured base_link -> zed_camera_link calibration')
        calibration_id = self._valid_measured_extrinsic(path)
        if calibration_id is None:
            return CheckResult.fail(
                'calibration', 'measured camera extrinsic file is invalid: {}'.format(path),
                'use the required measured calibration schema with finite values')
        return CheckResult.pass_('calibration', 'measured calibration {}'.format(calibration_id))

    def _phase3_safe_profile(self):
        path = self._path('phase3_profile_path')
        if path is None or not path.is_file():
            return CheckResult.not_ready(
                'phase3_safe_profile',
                'Phase 3 test profile is missing: {}'.format(path),
                'install phase123_test.yaml')
        try:
            parameters = yaml.safe_load(
                path.read_text(encoding='utf-8')
            )['semantic_memory']['ros__parameters']
        except (KeyError, OSError, TypeError, yaml.YAMLError):
            return CheckResult.fail(
                'phase3_safe_profile',
                'Phase 3 test profile is invalid: {}'.format(path),
                'restore the bounded phase123_test.yaml profile')
        required = {
            'camera_only_memory_enabled': True,
            'publish_diagnostic_ranking': True,
            'best_candidate_threshold_calibrated': False,
            'reidentification_mutation_enabled': False,
        }
        mismatches = [
            name for name, expected in required.items()
            if parameters.get(name) is not expected
        ]
        if mismatches:
            return CheckResult.fail(
                'phase3_safe_profile',
                'unsafe or missing values: {}'.format(', '.join(mismatches)),
                'restore the production-safe uncalibrated test profile')
        return CheckResult.pass_(
            'phase3_safe_profile',
            'diagnostic ranking enabled; production winner and re-ID mutation disabled')

    def check(self, stage):
        spec = resolve_stage(stage)
        results = []
        if spec.phase1:
            results.extend((
                self._directory(
                    'yolo_runtime', 'yolo_runtime_path',
                    'install the isolated YOLO-World runtime'),
                self._checkpoint(
                    'yolo_checkpoint', 'yolo_checkpoint_path',
                    'yolo_checkpoint_sha256',
                    'install yolov8s-worldv2.pt'),
                self._directory(
                    'clip_runtime', 'runtime_path',
                    'install the isolated CLIP task runtime'),
                self._checkpoint(
                    'clip_checkpoint', 'checkpoint_path',
                    'checkpoint_sha256',
                    'install ViT-B-32.pt'),
                self._directory(
                    'dino_repo', 'dino_repo_path',
                    'install the local DINOv3 Python source'),
                self._checkpoint(
                    'dino_checkpoint', 'dino_checkpoint_path',
                    'dino_checkpoint_sha256',
                    'install the DINOv3 checkpoint'),
                self._directory(
                    'cuda_runtime', 'cuda_runtime_path',
                    'install the JetPack CUDA runtime'),
            ))

        dds_profile = self._path('dds_profile')
        if dds_profile is not None and dds_profile.is_file():
            results.append(CheckResult.pass_('dds_profile', str(dds_profile)))
        else:
            results.append(CheckResult.not_ready(
                'dds_profile', 'DDS profile is missing: {}'.format(dds_profile),
                'install fastdds_semantic_search.xml'))

        if spec.memory:
            results.append(self._calibration())
        if spec.diagnostic_ranking:
            results.append(self._phase3_safe_profile())
        return tuple(results)


class RosCliProbe:
    """Use bounded ROS CLI observations; no command starts a ROS node."""

    _MESSAGE_HEADER = re.compile(r'(?:^|\n)header:\s*(?:\n|$)', re.IGNORECASE)
    _PUBLISHERS = re.compile(r'Publisher count:\s*(\d+)', re.IGNORECASE)
    _TRANSFORM = re.compile(r'(?:^|\n)\s*(?:-\s*)?Translation:\s*\[', re.IGNORECASE)
    _TOPIC_ABSENT = re.compile(r'\bunknown\s+topic\b|\bnot\s+found\b', re.IGNORECASE)

    def __init__(self, runner=None, environment=None, topic_timeout=20.0,
                 tf_timeout=0.03, dds_profile=None):
        self.runner = runner or subprocess.run
        base_environment = os.environ if environment is None else environment
        self.environment = managed_environment(
            base_environment, dds_profile=dds_profile)
        # Foxy's ``ros2 topic echo`` has no one-shot flag and buffers stdout
        # when captured through a pipe. Readiness intentionally terminates the
        # bounded echo after its timeout, so force each Python CLI subprocess
        # to expose received message headers before that termination.
        self.environment['PYTHONUNBUFFERED'] = '1'
        self.topic_timeout = float(topic_timeout)
        self.tf_timeout = float(tf_timeout)

    def _run(self, argv, timeout):
        kwargs = {
            'stdout': subprocess.PIPE,
            'stderr': subprocess.PIPE,
            'text': True,
            'timeout': timeout,
            'env': self.environment,
            'shell': False,
        }
        try:
            completed = self.runner(argv, **kwargs)
            return completed.returncode, self._text(completed.stdout), self._text(
                completed.stderr), False
        except subprocess.TimeoutExpired as error:
            return None, self._text(error.output), self._text(error.stderr), True
        except OSError as error:
            return 1, '', str(error), False

    def static(self, paths, stage):
        """Delegate local preflight to the ROS-independent static probe."""
        return StaticProbe(paths).check(stage)

    @staticmethod
    def _text(value):
        if value is None:
            return ''
        if isinstance(value, bytes):
            return value.decode(errors='replace')
        return str(value)

    @classmethod
    def _topic_is_absent(cls, output, stderr):
        return bool(cls._TOPIC_ABSENT.search(output + '\n' + stderr))

    @staticmethod
    def _command_detail(output, stderr):
        return (stderr or output).strip() or 'no diagnostic output'

    def publisher(self, name, topic):
        """Require a live publisher without waiting for the first message."""

        code, output, stderr, timed_out = self._run(
            ['ros2', 'topic', 'info', topic], self.topic_timeout)
        if code != 0:
            if timed_out:
                return CheckResult.not_ready(
                    name, 'timeout inspecting topic {}'.format(topic),
                    'wait for DDS discovery of {}'.format(topic))
            if code is not None and self._topic_is_absent(output, stderr):
                return CheckResult.not_ready(
                    name, 'topic {} is absent'.format(topic),
                    'start a publisher for {}'.format(topic))
            return CheckResult.fail(
                name, 'topic info failed: {}'.format(
                    self._command_detail(output, stderr)),
                'verify ros2 topic info {} can inspect the ROS graph'.format(topic))

        publishers = self._PUBLISHERS.search(output + '\n' + stderr)
        if publishers is None:
            return CheckResult.fail(
                name, 'could not determine publisher count for topic {}'.format(topic),
                'run ros2 topic info {}'.format(topic))
        if int(publishers.group(1)) == 0:
            return CheckResult.not_ready(
                name, 'no publishers for topic {}'.format(topic),
                'start a publisher for {}'.format(topic))
        return CheckResult.pass_(
            name, '{} has {} publisher{}'.format(
                topic,
                publishers.group(1),
                '' if publishers.group(1) == '1' else 's'))

    def topic(self, name, topic):
        publisher = self.publisher(name, topic)
        if publisher.status is not CheckStatus.PASS:
            return publisher

        _, output, stderr, _ = self._run([
            'ros2', 'topic', 'echo', topic,
            '--qos-profile', 'sensor_data', '--no-arr', '--no-str',
        ], self.topic_timeout)
        if self._MESSAGE_HEADER.search(output + '\n' + stderr) is None:
            return CheckResult.not_ready(
                name, 'timeout waiting for topic {}'.format(topic),
                'confirm {} is publishing messages'.format(topic))
        return CheckResult.pass_(
            name, '{} is publishing sensor messages'.format(topic))

    def transform(self, name, target, source):
        _, output, stderr, _ = self._run([
            'ros2', 'run', 'tf2_ros', 'tf2_echo', target, source,
        ], self.tf_timeout)
        if self._TRANSFORM.search(output + '\n' + stderr):
            return CheckResult.pass_(name, '{} -> {}'.format(target, source))
        return CheckResult.not_ready(
            name, 'timeout waiting for transform {} -> {}'.format(target, source),
            'publish the required TF path')

    def cmd_vel(self):
        code, output, stderr, _ = self._run(
            ['ros2', 'topic', 'info', '/cmd_vel'], self.topic_timeout)
        if code != 0:
            if code is not None and self._topic_is_absent(output, stderr):
                return CheckResult.pass_('cmd_vel', 'topic absent; zero publishers')
            return CheckResult.fail(
                'cmd_vel', 'topic info failed: {}'.format(
                    self._command_detail(output, stderr)),
                'run ros2 topic info /cmd_vel')
        publishers = self._PUBLISHERS.search(output + '\n' + stderr)
        if publishers is not None and int(publishers.group(1)) == 0:
            return CheckResult.pass_('cmd_vel', 'zero publishers')
        if publishers is not None:
            return CheckResult.fail(
                'cmd_vel', '{} publishers detected'.format(publishers.group(1)),
                'stop every /cmd_vel publisher before continuing')
        return CheckResult.fail(
            'cmd_vel', 'could not determine publisher count',
            'run ros2 topic info /cmd_vel')


_TOPICS = {
    'camera': '/zed/zed_node/left/image_rect_color',
    'camera_info': '/zed/zed_node/left/camera_info',
    'lidar': '/rslidar_points',
    'imu': '/imu/data_raw',
    'odometry': '/odom',
    'regions': '/semantic_search/regions',
    'tracklets': '/semantic_memory/lidar_tracklets',
    'localization': '/semantic_memory/localization_state',
    'memory': '/semantic_memory/active_objects',
    'diagnostic_ranking': '/semantic_memory/diagnostic_ranking',
    'best_candidate': '/semantic_memory/best_candidate',
}


def check_stage(stage, selection, paths, probe):
    """Return static and graph readiness for one declared stage.

    ``selection`` documents launch ownership and is deliberately not used to
    waive a stage requirement: reused external hardware must be ready too.
    """

    spec = resolve_stage(stage)
    checks = list(probe.static(paths, stage))
    if spec.camera:
        checks.append(probe.topic('camera', _TOPICS['camera']))
        checks.append(probe.topic('camera_info', _TOPICS['camera_info']))
    if spec.lidar:
        checks.append(probe.topic('lidar', _TOPICS['lidar']))
    if spec.imu:
        checks.append(probe.topic('imu', _TOPICS['imu']))
    if spec.base:
        checks.append(probe.topic('odometry', _TOPICS['odometry']))
    if spec.phase1:
        checks.append(probe.publisher('regions', _TOPICS['regions']))
    if spec.tracklets:
        checks.append(probe.topic('tracklets', _TOPICS['tracklets']))
    if spec.localization:
        checks.append(probe.topic('localization', _TOPICS['localization']))
    if spec.memory:
        checks.append(probe.topic('memory', _TOPICS['memory']))
        checks.append(probe.transform(
            'tf_camera_optical', 'base_link', 'zed_left_camera_optical_frame'))
        checks.append(probe.transform('tf_lidar', 'base_link', 'lidar_link'))
    if spec.diagnostic_ranking:
        checks.append(probe.publisher(
            'diagnostic_ranking', _TOPICS['diagnostic_ranking']))
        checks.append(probe.publisher(
            'best_candidate', _TOPICS['best_candidate']))
    checks.append(probe.cmd_vel())
    environment = getattr(probe, 'environment', {})
    return ReadinessReport(
        checks, stage=stage, ros_domain_id=environment.get('ROS_DOMAIN_ID', '20'))
