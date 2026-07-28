"""ROS adapter that prints and publishes Phase 4A approach advice."""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from track_robot_interfaces.msg import SemanticObjectArray, SemanticTask

from .phase4a_advisor import (
    AdvisoryGoal,
    AdvisoryInput,
    AdvisoryTarget,
    build_advice,
)


def _values(status):
    return {item.key: item.value for item in status.values}


class Phase4AAdvisorNode(Node):
    """Publish text only after a correlated planning-only success."""

    def __init__(self):
        super().__init__('phase4a_advisor')
        advisory_only = bool(self.declare_parameter(
            'advisory_only', True).value)
        if not advisory_only:
            raise ValueError('Phase 4A supports advisory_only=true only')
        self._standoff = float(self.declare_parameter(
            'standoff_distance', 0.8).value)
        self._query_text = ''
        self._query_id = 0
        self._query_version = 0
        self._target = None
        self._goal = None
        self._path = ()
        self._last_logged_text = None

        target_qos = QoSProfile(depth=1)
        target_qos.reliability = ReliabilityPolicy.RELIABLE
        target_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._target_subscription = self.create_subscription(
            SemanticObjectArray,
            '/semantic_search/phase4a/selected_target',
            self._on_target,
            target_qos)
        self._goal_subscription = self.create_subscription(
            PoseStamped,
            '/semantic_search/phase4/selected_goal',
            self._on_goal,
            5)
        self._path_subscription = self.create_subscription(
            Path,
            '/semantic_search/phase4/planned_path',
            self._on_path,
            5)
        self._planner_subscription = self.create_subscription(
            DiagnosticArray,
            '/semantic_search/phase4/diagnostics',
            self._on_planner_diagnostics,
            10)
        self._task_subscription = self.create_subscription(
            SemanticTask,
            '/semantic_memory/tasks',
            self._on_task,
            10)
        self._advice_publisher = self.create_publisher(
            String, '/semantic_search/phase4a/advice', 10)
        self._diagnostic_publisher = self.create_publisher(
            DiagnosticArray,
            '/semantic_search/phase4a/diagnostics',
            10)

    def _on_task(self, message):
        self._query_text = str(message.query_text)
        self._query_id = int(message.query_id)
        self._query_version = int(message.query_version)

    def _on_target(self, message):
        if len(message.objects) != 1:
            self._target = None
            return
        item = message.objects[0]
        self._target = AdvisoryTarget(
            query_text=self._query_text,
            memory_epoch_id=int(message.memory_epoch_id),
            global_object_id=int(item.global_object_id),
            localization_epoch_id=int(item.localization_epoch_id),
            query_id=int(item.active_query_id),
            query_version=int(item.active_query_version),
            x=float(item.position.x),
            y=float(item.position.y),
            z=float(item.position.z),
            confidence=float(item.task_relevance),
            uncertainty=float(item.uncertainty),
        )

    def _on_goal(self, message):
        self._goal = AdvisoryGoal(
            x=float(message.pose.position.x),
            y=float(message.pose.position.y),
        )

    def _on_path(self, message):
        self._path = tuple(
            (float(item.pose.position.x), float(item.pose.position.y))
            for item in message.poses)

    def _on_planner_diagnostics(self, message):
        if not message.status:
            return
        status = message.status[0]
        values = _values(status)

        def integer(key):
            try:
                return int(values.get(key, '0'))
            except ValueError:
                return 0

        planner_status = values.get('status', 'FAIL')
        result = build_advice(AdvisoryInput(
            planner_status=planner_status,
            planner_reason=values.get('reason', status.message),
            planner_memory_epoch_id=integer('memory_epoch_id'),
            planner_global_object_id=integer('global_object_id'),
            planner_localization_epoch_id=integer(
                'localization_epoch_id'),
            planner_query_id=integer('query_id'),
            planner_query_version=integer('query_version'),
            target=self._target,
            goal=self._goal,
            path=self._path,
            standoff_distance=self._standoff,
        ))
        text = String()
        text.data = result.text
        self._advice_publisher.publish(text)
        if result.text != self._last_logged_text:
            self.get_logger().info(result.text)
            self._last_logged_text = result.text
        self._publish_diagnostics(result, message)
        if result.status != 'READY':
            self._goal = None
            self._path = ()

    def _publish_diagnostics(self, result, planner_message):
        output = DiagnosticArray()
        output.header = planner_message.header
        status = DiagnosticStatus()
        status.name = 'semantic_search/phase4a_advisory'
        status.hardware_id = 'advisory_only'
        status.level = (
            DiagnosticStatus.OK
            if result.status == 'READY'
            else DiagnosticStatus.WARN)
        status.message = result.reason
        target = self._target
        goal = self._goal
        values = {
            'advisory_only': 'true',
            'status': result.status,
            'reason': result.reason,
            'range_m': '{:.3f}'.format(result.range_m),
            'bearing_deg': '{:.3f}'.format(result.bearing_deg),
            'path_length_m': '{:.3f}'.format(result.path_length_m),
            'memory_epoch_id': str(
                target.memory_epoch_id if target else 0),
            'global_object_id': str(
                target.global_object_id if target else 0),
            'localization_epoch_id': str(
                target.localization_epoch_id if target else 0),
            'query_id': str(target.query_id if target else self._query_id),
            'query_version': str(
                target.query_version if target else self._query_version),
            'target_x': '{:.3f}'.format(target.x if target else 0.0),
            'target_y': '{:.3f}'.format(target.y if target else 0.0),
            'target_z': '{:.3f}'.format(target.z if target else 0.0),
            'goal_x': '{:.3f}'.format(goal.x if goal else 0.0),
            'goal_y': '{:.3f}'.format(goal.y if goal else 0.0),
            'confidence': '{:.3f}'.format(
                target.confidence if target else 0.0),
            'uncertainty': '{:.3f}'.format(
                target.uncertainty if target else 1.0),
        }
        status.values = [
            KeyValue(key=key, value=value)
            for key, value in values.items()]
        output.status.append(status)
        self._diagnostic_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = Phase4AAdvisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
