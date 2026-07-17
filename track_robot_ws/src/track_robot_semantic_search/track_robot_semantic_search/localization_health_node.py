import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
from track_robot_interfaces.msg import SemanticLocalizationState

from .localization_health import (
    LocalizationHealthEvaluator,
    LocalizationSample,
    MemoryMode,
)


def stamp_ns(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def _validate_freshness_time_base(value):
    value = str(value)
    if value not in {'source_clock', 'arrival_monotonic'}:
        raise ValueError(
            'unsupported freshness_time_base: {}'.format(value))
    return value


def message_is_fresh(
        message, now_ns, timeout_sec, time_base='source_clock',
        arrival_ns=None):
    time_base = _validate_freshness_time_base(time_base)
    if message is None:
        return False
    if time_base == 'source_clock':
        reference_ns = stamp_ns(message.header.stamp)
    else:
        if arrival_ns is None:
            return False
        reference_ns = int(arrival_ns)
    age_ns = int(now_ns) - reference_ns
    return 0 <= age_ns <= int(float(timeout_sec) * 1000000000)


def odometry_frames_match(message, parent_frame, child_frame):
    return (
        message is not None and
        message.header.frame_id == parent_frame and
        message.child_frame_id == child_frame
    )


def stamp_valid_for_transform(stamp):
    seconds = int(stamp.sec)
    nanoseconds = int(stamp.nanosec)
    return (
        0 <= seconds <= 2147483647 and
        0 <= nanoseconds < 1000000000
    )


def odometry_transform_available(
        transform_buffer, message, parent_frame, child_frame):
    if not odometry_frames_match(message, parent_frame, child_frame):
        return False
    if not stamp_valid_for_transform(message.header.stamp):
        return False
    transform_time = Time(
        seconds=int(message.header.stamp.sec),
        nanoseconds=int(message.header.stamp.nanosec),
    )
    return transform_buffer.can_transform(
        parent_frame, child_frame, transform_time)


def world_values(message):
    if message is None:
        return math.inf, math.inf, math.nan, math.nan, math.nan
    pose = message.pose.pose
    covariance = message.pose.covariance
    x_variance = float(covariance[0])
    y_variance = float(covariance[7])
    if (not math.isfinite(x_variance) or x_variance < 0.0 or
            not math.isfinite(y_variance) or y_variance < 0.0):
        xy_variance = math.inf
    else:
        xy_variance = max(x_variance, y_variance)
    yaw = math.atan2(
        2.0 * (pose.orientation.w * pose.orientation.z +
               pose.orientation.x * pose.orientation.y),
        1.0 - 2.0 * (
            pose.orientation.y * pose.orientation.y +
            pose.orientation.z * pose.orientation.z),
    )
    return (
        xy_variance,
        float(covariance[35]),
        float(pose.position.x),
        float(pose.position.y),
        float(yaw),
    )


class LocalizationHealthNode(Node):
    def __init__(self):
        super().__init__('semantic_search_localization_health')
        self.imu_timeout = self.declare_parameter(
            'imu_timeout_sec', 0.25).value
        self.local_timeout = self.declare_parameter(
            'local_pose_timeout_sec', 0.30).value
        self.world_timeout = self.declare_parameter(
            'world_pose_timeout_sec', 0.30).value
        self.freshness_time_base = _validate_freshness_time_base(
            self.declare_parameter(
                'freshness_time_base', 'source_clock').value)
        self.local_frame = self.declare_parameter(
            'local_frame', 'odom').value
        self.world_frame = self.declare_parameter(
            'world_frame', 'map').value
        self.base_frame = self.declare_parameter(
            'base_frame', 'base_link').value
        self.evaluator = LocalizationHealthEvaluator(
            world_enabled=self.declare_parameter(
                'world_mode_enabled', False).value,
            world_stable_samples=self.declare_parameter(
                'world_stable_samples', 3).value,
            maximum_world_xy_variance_m2=self.declare_parameter(
                'maximum_world_xy_variance_m2', 0.25).value,
            maximum_world_yaw_variance_rad2=self.declare_parameter(
                'maximum_world_yaw_variance_rad2', 0.12).value,
            maximum_world_jump_m=self.declare_parameter(
                'maximum_world_jump_m', 0.50).value,
            maximum_world_yaw_jump_rad=self.declare_parameter(
                'maximum_world_yaw_jump_rad', 0.26).value,
        )
        self.imu = None
        self.local_pose = None
        self.world_pose = None
        self._source_stamps = {
            'imu': None,
            'local': None,
            'world': None,
        }
        self._source_timestamp_rollback = False
        self._world_source_timestamp_rollback = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Imu,
            self.declare_parameter('imu_topic', '/imu/data_raw').value,
            self._imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.declare_parameter('local_odometry_topic', '/odom').value,
            self._local,
            10,
        )
        self.create_subscription(
            Odometry,
            self.declare_parameter(
                'world_odometry_topic', '/localization/odometry').value,
            self._world,
            10,
        )
        self.publisher = self.create_publisher(
            DiagnosticArray,
            self.declare_parameter(
                'diagnostics_topic',
                '/semantic_search/localization_diagnostics').value,
            10,
        )
        self.state_publisher = self.create_publisher(
            SemanticLocalizationState,
            self.declare_parameter(
                'state_topic',
                '/semantic_memory/localization_state').value,
            10,
        )
        self._last_published_mode = None
        rate = max(1.0, float(self.declare_parameter(
            'publish_rate_hz', 10.0).value))
        self.create_timer(1.0 / rate, self._publish)

    def _store_received(self, topic, message):
        source_ns = stamp_ns(message.header.stamp)
        previous_source_ns = self._source_stamps[topic]
        rolled_back = (
            previous_source_ns is not None and
            source_ns < previous_source_ns)
        self._source_stamps[topic] = source_ns
        if rolled_back:
            if topic == 'world':
                self._world_source_timestamp_rollback = True
            else:
                self._source_timestamp_rollback = True
        return message, time.monotonic_ns()

    def _imu(self, message):
        self.imu = LocalizationHealthNode._store_received(
            self, 'imu', message)

    def _local(self, message):
        self.local_pose = LocalizationHealthNode._store_received(
            self, 'local', message)

    def _world(self, message):
        self.world_pose = LocalizationHealthNode._store_received(
            self, 'world', message)

    def _fresh(self, received, timeout, now_ns):
        if received is None:
            message = None
            arrival_ns = None
        else:
            message, arrival_ns = received
        return message_is_fresh(
            message, now_ns, timeout,
            time_base=self.freshness_time_base,
            arrival_ns=arrival_ns)

    def _world_values(self):
        return world_values(
            self.world_pose[0] if self.world_pose is not None else None)

    def _publish(self):
        covariance, yaw_variance, x, y, yaw = self._world_values()
        local_pose = (
            self.local_pose[0] if self.local_pose is not None else None)
        world_pose = (
            self.world_pose[0] if self.world_pose is not None else None)
        local_tf = odometry_transform_available(
            self.tf_buffer,
            local_pose,
            self.local_frame,
            self.base_frame,
        )
        world_tf = odometry_transform_available(
            self.tf_buffer,
            world_pose,
            self.world_frame,
            self.base_frame,
        )
        ros_now = self.get_clock().now()
        now_ns = (
            time.monotonic_ns()
            if self.freshness_time_base == 'arrival_monotonic'
            else ros_now.nanoseconds)
        local_pose_fresh = self._fresh(
            self.local_pose, self.local_timeout, now_ns)
        imu_fresh = self._fresh(
            self.imu, self.imu_timeout, now_ns)
        world_pose_fresh = self._fresh(
            self.world_pose, self.world_timeout, now_ns)
        decision = self.evaluator.update(LocalizationSample(
            stamp_ns=max(0, now_ns),
            local_pose_fresh=local_pose_fresh,
            imu_fresh=imu_fresh,
            local_tf_available=local_tf,
            world_pose_fresh=world_pose_fresh,
            world_tf_available=world_tf,
            world_pose_stamp_ns=(
                stamp_ns(world_pose.header.stamp)
                if world_pose is not None else -1),
            world_covariance_xy_m2=covariance,
            world_yaw_variance_rad2=yaw_variance,
            world_x=x,
            world_y=y,
            world_yaw=yaw,
            source_timestamp_rollback=self._source_timestamp_rollback,
            world_source_timestamp_rollback=(
                self._world_source_timestamp_rollback),
        ))
        self._source_timestamp_rollback = False
        self._world_source_timestamp_rollback = False
        status = DiagnosticStatus()
        status.name = 'semantic_search/localization'
        status.hardware_id = 'track_robot'
        status.message = decision.reason
        status.level = (
            DiagnosticStatus.OK if decision.mode == MemoryMode.WORLD
            else DiagnosticStatus.WARN
            if decision.mode == MemoryMode.LOCAL_SESSION
            else DiagnosticStatus.STALE)
        status.values = [
            KeyValue(key='memory_mode', value=decision.mode.name),
            KeyValue(key='epoch_id', value=str(decision.epoch_id)),
            KeyValue(
                key='epoch_changed',
                value=str(decision.epoch_changed).lower()),
            KeyValue(key='world_enabled', value=str(
                self.evaluator.world_enabled).lower()),
        ]
        array = DiagnosticArray()
        array.header.stamp = ros_now.to_msg()
        array.status = [status]
        self.publisher.publish(array)

        state = SemanticLocalizationState()
        state.header.stamp = ros_now.to_msg()
        state.memory_mode = int(decision.mode)
        state.localization_epoch_id = decision.epoch_id
        state.mode_changed = (
            self._last_published_mode is not None and
            self._last_published_mode != decision.mode)
        state.epoch_changed = decision.epoch_changed
        if decision.mode == MemoryMode.WORLD:
            state.canonical_frame_id = self.world_frame
        elif decision.mode == MemoryMode.LOCAL_SESSION:
            state.canonical_frame_id = self.local_frame
        else:
            state.canonical_frame_id = self.base_frame
        state.header.frame_id = state.canonical_frame_id
        state.local_frame_id = self.local_frame
        state.world_frame_id = self.world_frame
        state.base_frame_id = self.base_frame
        state.local_healthy = bool(
            local_pose_fresh and imu_fresh and local_tf)
        state.world_healthy = decision.mode == MemoryMode.WORLD
        state.reason = decision.reason
        self.state_publisher.publish(state)
        self._last_published_mode = decision.mode


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationHealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
