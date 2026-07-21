import bisect
import json
import math
import time
from pathlib import Path

import psutil
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2
from track_robot_interfaces.msg import (
    ObjectObservation3DArray,
    SearchMotionIntent,
    SemanticRegionArray,
    TrackedSemanticObjectArray,
)

from .evaluation import EvaluationAccumulator
from .manifest import load_manifest, sha256_file, write_json_atomic


def stamp_ns(stamp):
    """Convert a ROS stamp to integer nanoseconds."""
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


class StampPairMatcher:
    """Buffer complete stamp multisets for deterministic final pairing."""

    _TOPICS = ('image', 'lidar')

    def __init__(self):
        self._pending = {name: [] for name in self._TOPICS}

    def observe(self, topic, source_stamp_ns):
        if topic not in self._pending:
            raise ValueError('unsupported pair topic: {}'.format(topic))
        source_stamp_ns = int(source_stamp_ns)
        bisect.insort(self._pending[topic], source_stamp_ns)
        return []

    def flush(self):
        offsets = self._drain()
        for pending in self._pending.values():
            pending.clear()
        return offsets

    def _nearest_pair(self):
        images = self._pending['image']
        lidars = self._pending['lidar']
        if not images or not lidars:
            return None
        image_index = 0
        lidar_index = 0
        best = None
        while image_index < len(images) and lidar_index < len(lidars):
            image_stamp = images[image_index]
            lidar_stamp = lidars[lidar_index]
            candidate = (
                abs(image_stamp - lidar_stamp), image_stamp, lidar_stamp,
                image_index, lidar_index)
            if best is None or candidate < best:
                best = candidate
            if image_stamp <= lidar_stamp:
                image_index += 1
            else:
                lidar_index += 1
        return best

    def _drain(self):
        offsets = []
        while self._pending['image'] and self._pending['lidar']:
            pair = self._nearest_pair()
            distance, _, _, image_index, lidar_index = pair
            self._pending['image'].pop(image_index)
            self._pending['lidar'].pop(lidar_index)
            offsets.append(distance)
        return offsets


class SemanticSearchEvaluatorNode(Node):
    """Observe a bounded replay and atomically emit one JSON report."""

    def __init__(self):
        super().__init__('semantic_search_evaluator')
        manifest_path = str(self.declare_parameter(
            'manifest_path', '').value)
        self.output_path = Path(str(self.declare_parameter(
            'output_path',
            '/tmp/semantic_search_phase0_report.json').value))
        self.duration_sec = float(self.declare_parameter(
            'duration_sec', 30.0).value)
        run_id = str(self.declare_parameter(
            'run_id', 'phase0').value)
        replay_rate = float(self.declare_parameter(
            'replay_rate', 1.0).value)
        timing_policy = str(self.declare_parameter(
            'timing_policy', 'online_source_time').value)
        freshness_time_base = str(self.declare_parameter(
            'freshness_time_base', 'source_clock').value)
        software_revision = str(self.declare_parameter(
            'software_revision', 'unversioned').value)
        config_path = str(self.declare_parameter(
            'config_path', '').value)
        tegrastats_path = str(self.declare_parameter(
            'tegrastats_path', '').value)
        if not manifest_path:
            raise ValueError('manifest_path is required')
        if not config_path:
            raise ValueError('config_path is required')
        if (not math.isfinite(self.duration_sec) or
                self.duration_sec <= 0.0):
            raise ValueError('duration_sec must be positive')

        manifest = load_manifest(Path(manifest_path))
        self.metrics = EvaluationAccumulator(
            manifest=manifest,
            manifest_sha256=sha256_file(manifest_path),
            run_id=run_id,
            software_revision=software_revision,
            config_sha256=sha256_file(config_path),
            replay_rate=replay_rate,
            wall_duration_sec=self.duration_sec,
            timing_policy=timing_policy,
            freshness_time_base=freshness_time_base,
            tegrastats_path=tegrastats_path,
        )
        self.pair_matcher = StampPairMatcher()
        self.started_ros_ns = None
        self.finished = False
        self.process = psutil.Process()
        self.process.cpu_percent(interval=None)
        psutil.cpu_percent(interval=None)

        self.create_subscription(
            Image,
            self.declare_parameter(
                'image_topic',
                '/zed/zed_node/left/image_rect_color').value,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2,
            self.declare_parameter(
                'lidar_topic', '/rslidar_points').value,
            self.cloud_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            self.declare_parameter(
                'imu_topic', '/imu/data_raw').value,
            lambda message: self._observe(
                'imu', message.header.stamp),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            self.declare_parameter(
                'local_odometry_topic', '/odom').value,
            lambda message: self._observe(
                'local_pose', message.header.stamp),
            10,
        )
        self.create_subscription(
            Odometry,
            self.declare_parameter(
                'world_odometry_topic',
                '/localization/odometry').value,
            lambda message: self._observe(
                'world_pose', message.header.stamp),
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            self.declare_parameter(
                'diagnostics_topic',
                '/semantic_search/localization_diagnostics').value,
            self.diagnostic_callback,
            10,
        )
        self.create_subscription(
            SemanticRegionArray,
            self.declare_parameter(
                'semantic_regions_topic',
                '/semantic_search/regions').value,
            self.region_callback,
            10,
        )
        self.create_subscription(
            ObjectObservation3DArray,
            self.declare_parameter(
                'observations_topic',
                '/semantic_search/observations').value,
            self.observation_callback,
            10,
        )
        self.create_subscription(
            TrackedSemanticObjectArray,
            self.declare_parameter(
                'tracked_objects_topic',
                '/semantic_search/tracked_objects').value,
            self.tracked_callback,
            10,
        )
        self.create_subscription(
            SearchMotionIntent,
            self.declare_parameter(
                'motion_intent_topic',
                '/semantic_search/motion_intent').value,
            self.intent_callback,
            10,
        )
        self.create_timer(1.0, self.resource_callback)
        self.create_timer(0.1, self.finish_callback)

    def _observe(self, name, stamp):
        source_ns = stamp_ns(stamp)
        self.metrics.observe_topic(
            name, source_ns, time.monotonic_ns())
        sensor_topics = {
            'image', 'lidar', 'imu', 'local_pose', 'world_pose'}
        if self.started_ros_ns is None and name in sensor_topics:
            started_ros_ns = self.get_clock().now().nanoseconds
            self.process.cpu_percent(interval=None)
            psutil.cpu_percent(interval=None)
            self.started_ros_ns = started_ros_ns

    def image_callback(self, message):
        started_ns = time.perf_counter_ns()
        try:
            value = stamp_ns(message.header.stamp)
            self._observe('image', message.header.stamp)
            for offset_ns in self.pair_matcher.observe('image', value):
                self.metrics.observe_pair_offset(offset_ns)
        finally:
            self.metrics.observe_latency(
                'image_callback',
                (time.perf_counter_ns() - started_ns) / 1000000000.0,
            )

    def cloud_callback(self, message):
        started_ns = time.perf_counter_ns()
        try:
            value = stamp_ns(message.header.stamp)
            self._observe('lidar', message.header.stamp)
            for offset_ns in self.pair_matcher.observe('lidar', value):
                self.metrics.observe_pair_offset(offset_ns)
        finally:
            self.metrics.observe_latency(
                'lidar_callback',
                (time.perf_counter_ns() - started_ns) / 1000000000.0,
            )

    def diagnostic_callback(self, message):
        if not self.metrics.required_topics_ready():
            return
        for status in message.status:
            if status.name != 'semantic_search/localization':
                continue
            values = {item.key: item.value for item in status.values}
            try:
                epoch_id = int(values['epoch_id'])
            except (TypeError, ValueError):
                epoch_id = None
            except KeyError:
                epoch_id = None
            self.metrics.observe_localization(
                values.get('memory_mode', 'UNKNOWN'), epoch_id)

    def region_callback(self, message):
        if not self.metrics.required_topics_ready():
            return
        self._observe('semantic_regions', message.header.stamp)
        self.metrics.semantic_region_count += len(message.regions)

    def observation_callback(self, message):
        if not self.metrics.required_topics_ready():
            return
        self._observe('observations', message.header.stamp)
        self.metrics.observation_count += len(message.observations)

    def tracked_callback(self, message):
        if not self.metrics.required_topics_ready():
            return
        self._observe('tracked_objects', message.header.stamp)
        self.metrics.tracked_object_count += len(message.objects)

    def intent_callback(self, message):
        self.metrics.observe_motion_intent(message.forward_permitted)

    def resource_callback(self):
        if self.started_ros_ns is None or self.finished:
            return
        virtual_memory = psutil.virtual_memory()
        self.metrics.observe_resource(
            self.process.cpu_percent(interval=None),
            self.process.memory_info().rss / (1024.0 * 1024.0),
            psutil.cpu_percent(interval=None),
            virtual_memory.used / (1024.0 * 1024.0),
        )

    def finish_callback(self):
        if self.finished or self.started_ros_ns is None:
            return
        elapsed_ns = (
            self.get_clock().now().nanoseconds - self.started_ros_ns)
        if elapsed_ns < int(self.duration_sec * 1000000000):
            return
        for offset_ns in self.pair_matcher.flush():
            self.metrics.observe_pair_offset(offset_ns)
        report = self.metrics.finalize()
        json.dumps(report, allow_nan=False)
        write_json_atomic(self.output_path, report)
        self.finished = True
        self.get_logger().info('wrote {}'.format(self.output_path))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticSearchEvaluatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
