"""ROS publisher for the explicit Phase 4A fixed-base test session."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from track_robot_interfaces.msg import SemanticLocalizationState

from .fixed_base_session import FixedBaseSession


def _allocate_epoch() -> int:
    return max(1, int(time.time_ns()) & ((1 << 63) - 1))


class FixedBaseSessionNode(Node):
    """Publish a local-session contract without odometry, IMU, or TF."""

    def __init__(self):
        super().__init__('phase4a_fixed_base_session')
        state_topic = str(self.declare_parameter(
            'state_topic',
            '/semantic_search/phase4a/localization_state').value)
        frame_id = str(self.declare_parameter(
            'frame_id', 'base_link').value)
        publish_rate_hz = float(self.declare_parameter(
            'publish_rate_hz', 10.0).value)
        epoch_parameter = int(self.declare_parameter(
            'epoch_id', 0).value)
        if not 1.0 <= publish_rate_hz <= 100.0:
            raise ValueError('publish_rate_hz must be in [1, 100]')
        epoch_id = epoch_parameter if epoch_parameter > 0 else _allocate_epoch()
        self._session = FixedBaseSession(epoch_id, frame_id)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._publisher = self.create_publisher(
            SemanticLocalizationState, state_topic, qos)
        self._timer = self.create_timer(
            1.0 / publish_rate_hz, self._publish)
        self.get_logger().warn(
            'Phase 4A fixed-base test session asserted by operator; '
            'no odometry or IMU is being used')

    def _publish(self):
        now_ns = int(self.get_clock().now().nanoseconds)
        state = self._session.build_state(now_ns)
        message = SemanticLocalizationState()
        message.header.stamp.sec = int(now_ns // 1_000_000_000)
        message.header.stamp.nanosec = int(now_ns % 1_000_000_000)
        message.header.frame_id = state.canonical_frame_id
        message.memory_mode = SemanticLocalizationState.MEMORY_LOCAL_SESSION
        message.localization_epoch_id = state.localization_epoch_id
        message.mode_changed = False
        message.epoch_changed = False
        message.canonical_frame_id = state.canonical_frame_id
        message.local_frame_id = state.local_frame_id
        message.world_frame_id = ''
        message.base_frame_id = state.base_frame_id
        message.local_healthy = state.local_healthy
        message.world_healthy = state.world_healthy
        message.reason = state.reason
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = FixedBaseSessionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
