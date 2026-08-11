"""Enrich semantic observations with registered ZED depth."""

import copy

from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from track_robot_interfaces.msg import (
    SemanticLocalizationState,
    SemanticObservationArray,
)

from .depth_frame_buffer import DepthFrame, DepthFrameBuffer
from .phase4a_depth import CameraIntrinsics
from .spatial_observation import (
    SpatialObservationConfig,
    spatialize_observation,
)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SpatialObservationNode(Node):
    """Publish every observation array, adding metric depth when available."""

    _COUNTER_KEYS = (
        'matched_depth',
        'no_matching_depth',
        'depth_delta_exceeded',
        'insufficient_depth_samples',
        'depth_out_of_range',
        'tf_unavailable',
        'invalid_transformed_position',
        'camera_info_unavailable',
        'localization_unavailable',
    )

    def __init__(self):
        super().__init__('semantic_depth_enricher')
        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._intrinsics = None
        self._localization_epoch_id = 0

        input_topic = self.declare_parameter(
            'input_observations_topic',
            '/semantic_memory/observations').value
        output_topic = self.declare_parameter(
            'output_observations_topic',
            '/semantic_memory/spatial_observations').value
        depth_topic = self.declare_parameter(
            'depth_topic',
            '/zed/zed_node/depth/depth_registered').value
        camera_info_topic = self.declare_parameter(
            'camera_info_topic',
            '/zed/zed_node/left/camera_info').value
        localization_topic = self.declare_parameter(
            'localization_topic',
            '/semantic_search/phase4a/localization_state').value
        diagnostics_topic = self.declare_parameter(
            'diagnostics_topic',
            '/semantic_search/spatial_observation_diagnostics').value
        self._depth_buffer = DepthFrameBuffer(
            max_frames=int(self.declare_parameter(
                'depth_buffer_frames', 16).value),
            max_age_ns=int(float(self.declare_parameter(
                'depth_buffer_max_age_sec', 2.0).value) * 1_000_000_000),
        )
        self._maximum_depth_delta_ns = int(float(self.declare_parameter(
            'maximum_depth_delta_sec', 0.20).value) * 1_000_000_000)
        self._tf_timeout_sec = float(self.declare_parameter(
            'tf_timeout_sec', 0.05).value)
        self._config = SpatialObservationConfig(
            frame_id=str(self.declare_parameter(
                'frame_id', 'base_link').value),
            minimum_samples=int(self.declare_parameter(
                'minimum_depth_samples', 20).value),
            minimum_depth_m=float(self.declare_parameter(
                'minimum_depth_m', 0.3).value),
            maximum_depth_m=float(self.declare_parameter(
                'maximum_depth_m', 8.0).value),
            inner_fraction=float(self.declare_parameter(
                'depth_inner_fraction', 0.5).value),
        )
        self._config.validate()
        self._counters = {key: 0 for key in self._COUNTER_KEYS}

        self._publisher = self.create_publisher(
            SemanticObservationArray, output_topic, 10)
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, diagnostics_topic, 10)
        self._observation_subscription = self.create_subscription(
            SemanticObservationArray, input_topic, self._on_observations, 10)
        self._depth_subscription = self.create_subscription(
            Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data)
        self._localization_subscription = self.create_subscription(
            SemanticLocalizationState,
            localization_topic,
            self._on_localization,
            10)

    def _on_localization(self, message):
        self._localization_epoch_id = 0
        if (
                message.local_healthy
                and str(message.canonical_frame_id) == self._config.frame_id
                and int(message.localization_epoch_id) > 0):
            self._localization_epoch_id = int(message.localization_epoch_id)

    def _on_camera_info(self, message):
        try:
            intrinsics = CameraIntrinsics(
                fx=float(message.k[0]),
                fy=float(message.k[4]),
                cx=float(message.k[2]),
                cy=float(message.k[5]),
            )
            intrinsics.validate()
            self._intrinsics = intrinsics
        except (IndexError, TypeError, ValueError):
            self._intrinsics = None

    def _on_depth(self, message):
        try:
            depth = self._bridge.imgmsg_to_cv2(
                message, desired_encoding='32FC1')
            if depth.ndim != 2:
                raise ValueError('depth image is not two-dimensional')
            self._depth_buffer.push(DepthFrame(
                stamp_ns=_stamp_ns(message.header.stamp),
                frame_id=str(message.header.frame_id),
                image=depth,
            ))
        except (CvBridgeError, TypeError, ValueError):
            pass

    def _observation_stamp_ns(self, message, observation):
        if observation.camera_stamp_valid:
            return _stamp_ns(observation.camera_stamp)
        return _stamp_ns(message.header.stamp)

    def _on_observations(self, message):
        output = copy.deepcopy(message)
        enriched = []
        enriched_count = 0
        latest_reason = 'no_observations'
        depth_delta_ns = 0
        valid_depth_samples = 0
        depth_quality = 0.0
        for observation in message.observations:
            if self._intrinsics is None or self._localization_epoch_id <= 0:
                latest_reason = (
                    'camera_info_unavailable'
                    if self._intrinsics is None
                    else 'localization_unavailable')
                self._counters[latest_reason] += 1
                enriched.append(copy.deepcopy(observation))
                depth_delta_ns = 0
                valid_depth_samples = 0
                depth_quality = 0.0
                continue

            source_stamp_ns = self._observation_stamp_ns(message, observation)
            match = self._depth_buffer.nearest(source_stamp_ns,
                                               self._maximum_depth_delta_ns)
            if match is None:
                latest_reason = (
                    'no_matching_depth'
                    if self._depth_buffer.size == 0
                    else 'depth_delta_exceeded')
                self._counters[latest_reason] += 1
                enriched.append(copy.deepcopy(observation))
                depth_delta_ns = 0
                valid_depth_samples = 0
                depth_quality = 0.0
                continue

            depth_delta_ns = match.delta_ns
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._config.frame_id,
                    match.frame.frame_id,
                    Time(nanoseconds=match.frame.stamp_ns),
                    timeout=Duration(seconds=self._tf_timeout_sec),
                )
            except Exception:  # Foxy tf2 exception classes vary by patch.
                latest_reason = 'tf_unavailable'
                self._counters[latest_reason] += 1
                enriched.append(copy.deepcopy(observation))
                valid_depth_samples = 0
                depth_quality = 0.0
                continue

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            result = spatialize_observation(
                observation,
                depth=match.frame.image,
                intrinsics=self._intrinsics,
                translation=(
                    translation.x, translation.y, translation.z),
                quaternion=(
                    rotation.x, rotation.y, rotation.z, rotation.w),
                localization_epoch_id=self._localization_epoch_id,
                depth_stamp_ns=match.frame.stamp_ns,
                config=self._config,
            )
            enriched.append(result.observation)
            latest_reason = result.reason
            self._counters[latest_reason] += 1
            valid_depth_samples = result.valid_depth_samples
            depth_quality = result.depth_quality
            if result.accepted:
                enriched_count += 1
        output.observations = enriched
        self._publisher.publish(output)
        self._publish_diagnostics(
            enriched_count=enriched_count,
            latest_reason=latest_reason,
            depth_delta_ns=depth_delta_ns,
            valid_depth_samples=valid_depth_samples,
            depth_quality=depth_quality,
        )

    def _publish_diagnostics(
            self,
            *,
            enriched_count,
            latest_reason,
            depth_delta_ns,
            valid_depth_samples,
            depth_quality):
        status = DiagnosticStatus()
        status.name = 'semantic_search/spatial_observation'
        status.hardware_id = 'zed_registered_depth'
        status.level = (
            DiagnosticStatus.OK
            if enriched_count > 0
            else DiagnosticStatus.WARN)
        status.message = latest_reason
        status.values = [
            KeyValue(key='latest_reason', value=latest_reason),
            KeyValue(
                key='depth_delta_ms',
                value='{:.3f}'.format(depth_delta_ns / 1e6)),
            KeyValue(
                key='valid_depth_samples', value=str(valid_depth_samples)),
            KeyValue(
                key='depth_quality', value='{:.6f}'.format(depth_quality)),
        ] + [
            KeyValue(key=key, value=str(self._counters[key]))
            for key in self._COUNTER_KEYS
        ]
        output = DiagnosticArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.status = [status]
        self._diagnostics_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = SpatialObservationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
