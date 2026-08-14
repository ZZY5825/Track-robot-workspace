"""Bounded ROS CLI readiness checks for live human following."""

import os
import re
import subprocess

from .control_config import managed_environment
from .readiness import CheckResult, ReadinessReport


_PUBLISHERS = re.compile(r'Publisher count:\s*(\d+)', re.IGNORECASE)
_TOPIC_ABSENT = re.compile(
    r'\bunknown\s+topic\b|\bnot\s+found\b', re.IGNORECASE)
_TRANSFORM = re.compile(
    r'(?:^|\n)\s*(?:-\s*)?Translation:\s*\[', re.IGNORECASE)
_MESSAGE = re.compile(r'(?:^|\n)header:\s*(?:\n|$)|(?:^|\n)---\s*(?:\n|$)')

_TOPICS = (
    ('image', '/zed/zed_node/left/image_rect_color'),
    ('camera_info', '/zed/zed_node/left/camera_info'),
    ('imu', '/zed/zed_node/imu/data'),
    ('cloud', '/rslidar_points'),
    ('bunker_status', '/bunker_status'),
    ('rc_state', '/bunker_rc_state'),
    ('odometry', '/odom'),
    ('target_state', '/human_tracking/target_state'),
    ('perception_health', '/perception/health'),
    ('avoidance', '/follow/avoidance_state'),
    ('safety', '/safety/state'),
)
_TRANSFORMS = (
    ('tf_camera', 'base_link', 'zed_left_camera_optical_frame'),
    ('tf_lidar', 'base_link', 'rslidar'),
)
_SERVICES = (
    ('safety_arm', '/safety/arm'),
    ('safety_disarm', '/safety/disarm'),
    ('reset_target', '/human_tracking/reset_target'),
)


def _text(value):
    if value is None:
        return ''
    if isinstance(value, bytes):
        return value.decode(errors='replace')
    return str(value)


class HumanFollowingProbe:
    """Inspect the graph without importing ROS client libraries."""

    def __init__(self, runner=None, environment=None, topic_timeout=1.0,
                 tf_timeout=0.2):
        self.runner = runner or subprocess.run
        self.environment = managed_environment(
            os.environ if environment is None else environment)
        self.environment['PYTHONUNBUFFERED'] = '1'
        self.topic_timeout = max(0.01, float(topic_timeout))
        self.tf_timeout = max(0.01, float(tf_timeout))

    def _run(self, argv, timeout):
        try:
            completed = self.runner(
                argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=float(timeout),
                env=self.environment,
                shell=False,
            )
            return (
                completed.returncode,
                _text(completed.stdout),
                _text(completed.stderr),
                False,
            )
        except subprocess.TimeoutExpired as error:
            return None, _text(error.output), _text(error.stderr), True
        except OSError as error:
            return 1, '', str(error), False

    @staticmethod
    def _detail(output, stderr):
        return (stderr or output).strip() or 'no diagnostic output'

    def topic(self, name, topic):
        code, output, stderr, timed_out = self._run(
            ['ros2', 'topic', 'info', topic], self.topic_timeout)
        combined = '{}\n{}'.format(output, stderr)
        publishers = _PUBLISHERS.search(combined)
        if code != 0 or publishers is None:
            detail = self._detail(output, stderr)
            if timed_out:
                detail = 'timed out: {}'.format(detail)
            return CheckResult.fail(
                name,
                'cannot determine exact publisher count for {}: {}'.format(
                    topic, detail),
                'verify ROS graph discovery on Domain 20')
        count = int(publishers.group(1))
        if count != 1:
            return CheckResult.fail(
                name,
                '{} has {} publishers; expected exactly one'.format(
                    topic, count),
                'stop duplicate publishers or start the missing dependency')

        _echo_code, echo, echo_error, _echo_timed_out = self._run([
            'ros2', 'topic', 'echo', topic,
            '--qos-profile', 'sensor_data', '--no-arr', '--no-str',
        ], self.topic_timeout)
        message = '{}\n{}'.format(echo, echo_error)
        if _MESSAGE.search(message) is None:
            return CheckResult.fail(
                name,
                '{} did not produce a fresh message within {:.3f}s'.format(
                    topic, self.topic_timeout),
                'restore a live publisher before starting human following')
        if name == 'bunker_status':
            return self._bunker_status(topic, message)
        return CheckResult.pass_(
            name, '{} has exactly one publisher and a fresh message'.format(
                topic))

    @staticmethod
    def _last_integer(message, field):
        values = re.findall(
            r'(?:^|\n){}:\s*(-?\d+)\s*(?:\n|$)'.format(
                re.escape(field)),
            message,
        )
        return int(values[-1]) if values else None

    def _bunker_status(self, topic, message):
        control_mode = self._last_integer(message, 'control_mode')
        vehicle_state = self._last_integer(message, 'vehicle_state')
        error_code = self._last_integer(message, 'error_code')
        if None in (control_mode, vehicle_state, error_code):
            return CheckResult.fail(
                'bunker_status',
                '{} message omitted control or base health fields'.format(
                    topic),
                'verify the Bunker status message contract')
        if control_mode != 1:
            return CheckResult.fail(
                'bunker_status',
                '{} reports control_mode {}; expected CAN mode 1'.format(
                    topic, control_mode),
                'return the base to CAN control before active operation')
        if vehicle_state != 0 or error_code != 0:
            return CheckResult.fail(
                'bunker_status',
                '{} reports vehicle_state {} and error_code {}'.format(
                    topic, vehicle_state, error_code),
                'clear the base fault before active operation')
        return CheckResult.pass_(
            'bunker_status',
            '{} is fresh, in CAN mode, and reports no base error'.format(
                topic))

    def transform(self, name, target, source):
        _code, output, stderr, _timed_out = self._run([
            'ros2', 'run', 'tf2_ros', 'tf2_echo', target, source,
        ], self.tf_timeout)
        if _TRANSFORM.search('{}\n{}'.format(output, stderr)):
            return CheckResult.pass_(
                name, 'transform {} -> {} is available'.format(
                    target, source))
        return CheckResult.fail(
            name,
            'required transform {} -> {} is unavailable'.format(
                target, source),
            'publish the complete physical TF tree')

    def service(self, name, service):
        code, output, stderr, timed_out = self._run([
            'ros2', 'service', 'type', service,
        ], self.topic_timeout)
        service_type = output.strip()
        if code == 0 and service_type == 'std_srvs/srv/Trigger':
            return CheckResult.pass_(
                name, '{} provides std_srvs/srv/Trigger'.format(service))
        detail = self._detail(output, stderr)
        if timed_out:
            detail = 'timed out: {}'.format(detail)
        return CheckResult.fail(
            name,
            'required Trigger service {} is unavailable: {}'.format(
                service, detail),
            'start the owning safety or tracking node')

    def cmd_vel(self, runtime_mode):
        code, output, stderr, timed_out = self._run([
            'ros2', 'topic', 'info', '/cmd_vel', '--verbose',
        ], self.topic_timeout)
        combined = '{}\n{}'.format(output, stderr)
        publishers = _PUBLISHERS.search(combined)
        if code != 0 and _TOPIC_ABSENT.search(combined):
            count = 0
        elif code == 0 and publishers is not None:
            count = int(publishers.group(1))
        else:
            detail = self._detail(output, stderr)
            if timed_out:
                detail = 'timed out: {}'.format(detail)
            return CheckResult.fail(
                'cmd_vel',
                'cannot determine /cmd_vel publisher ownership: {}'.format(
                    detail),
                'inspect /cmd_vel on Domain 20')

        expected = 1 if runtime_mode == 'active' else 0
        if count != expected:
            return CheckResult.fail(
                'cmd_vel',
                '/cmd_vel has {} publishers; {} requires exactly {}'.format(
                    count, runtime_mode, expected),
                'stop every unexpected command publisher')
        if runtime_mode == 'active':
            publisher_section = re.split(
                r'(?:^|\n)\s*Subscription count:',
                combined,
                maxsplit=1,
                flags=re.IGNORECASE,
            )[0]
            node_name = re.search(
                r'(?:^|\n)\s*Node name:\s*cmd_vel_gate\s*(?:\n|$)',
                publisher_section,
                re.IGNORECASE,
            )
            node_namespace = re.search(
                r'(?:^|\n)\s*Node namespace:\s*/\s*(?:\n|$)',
                publisher_section,
                re.IGNORECASE,
            )
            if node_name is None or node_namespace is None:
                return CheckResult.fail(
                    'cmd_vel',
                    '/cmd_vel publisher is not owned by /cmd_vel_gate',
                    'stop the unexpected publisher before active operation')
            return CheckResult.pass_(
                'cmd_vel', 'exactly one publisher owned by /cmd_vel_gate')
        return CheckResult.pass_('cmd_vel', 'zero /cmd_vel publishers')


def check_human_following(
        runtime_mode,
        *,
        runner=None,
        environment=None,
        topic_timeout=1.0,
        tf_timeout=0.2):
    """Return a complete fail-closed graph snapshot for one runtime mode."""

    if runtime_mode not in ('shadow', 'active'):
        raise ValueError('runtime mode must be shadow or active')
    probe = HumanFollowingProbe(
        runner=runner,
        environment=environment,
        topic_timeout=topic_timeout,
        tf_timeout=tf_timeout,
    )
    checks = [probe.topic(name, topic) for name, topic in _TOPICS]
    checks.extend(
        probe.transform(name, target, source)
        for name, target, source in _TRANSFORMS)
    checks.extend(
        probe.service(name, service) for name, service in _SERVICES)
    checks.append(probe.cmd_vel(runtime_mode))
    return ReadinessReport(
        checks,
        stage=runtime_mode,
        ros_domain_id=probe.environment['ROS_DOMAIN_ID'],
    )


__all__ = ['HumanFollowingProbe', 'check_human_following']
