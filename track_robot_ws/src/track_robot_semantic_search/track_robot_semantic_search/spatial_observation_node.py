"""ROS adapter that enriches semantic observations with registered ZED depth."""

import copy

from cv_bridge import CvBridge, CvBridgeError
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

from .phase4a_depth import CameraIntrinsics
from .spatial_observation import (
    SpatialObservationConfig,
    spatialize_observation,
)


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class SpatialObservationNode(Node):
    """Publish every observation array, adding metric depth when available."""

    def __init__(self):
        super().__init__('semantic_depth_enricher')
        self._bridge = CvBridge()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._depth = None
        self._depth_stamp_ns = 0
        self._depth_frame_id = ''
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
        self._maximum_depth_delta_ns = int(float(self.declare_parameter(
            'maximum_depth_delta_sec', 0.5).value) * 1_000_000_000)
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

        self._publisher = self.create_publisher(
            SemanticObservationArray, output_topic, 10)
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
        if (
                message.local_healthy
                and str(message.canonical_frame_id) == self._config.frame_id):
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
            self._depth = depth
            self._depth_stamp_ns = _stamp_ns(message.header.stamp)
            self._depth_frame_id = str(message.header.frame_id)
        except (CvBridgeError, TypeError, ValueError):
            self._depth = None
            self._depth_stamp_ns = 0
            self._depth_frame_id = ''

    def _observation_stamp_ns(self, message, observation):
        if observation.camera_stamp_valid:
            return _stamp_ns(observation.camera_stamp)
        return _stamp_ns(message.header.stamp)

    def _on_observations(self, message):
        output = copy.deepcopy(message)
        if (
                self._depth is None
                or self._intrinsics is None
                or not self._depth_frame_id
                or self._depth_stamp_ns <= 0
                or self._localization_epoch_id <= 0):
            self._publisher.publish(output)
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._config.frame_id,
                self._depth_frame_id,
                Time(nanoseconds=self._depth_stamp_ns),
                timeout=Duration(seconds=self._tf_timeout_sec),
            )
        except Exception:  # Foxy tf2 exception classes differ by patch release.
            self._publisher.publish(output)
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        enriched = []
        for observation in message.observations:
            source_stamp_ns = self._observation_stamp_ns(message, observation)
            if (
                    source_stamp_ns <= 0
                    or abs(source_stamp_ns - self._depth_stamp_ns)
                    > self._maximum_depth_delta_ns):
                enriched.append(copy.deepcopy(observation))
                continue
            spatial, _ = spatialize_observation(
                observation,
                depth=self._depth,
                intrinsics=self._intrinsics,
                translation=(
                    translation.x, translation.y, translation.z),
                quaternion=(
                    rotation.x, rotation.y, rotation.z, rotation.w),
                localization_epoch_id=self._localization_epoch_id,
                depth_stamp_ns=self._depth_stamp_ns,
                config=self._config,
            )
            enriched.append(spatial)
        output.observations = enriched
        self._publisher.publish(output)


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
