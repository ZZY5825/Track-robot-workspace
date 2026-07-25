"""Exact-source-stamp image overlays for passive semantic-search review."""

from collections import OrderedDict
from dataclasses import dataclass
import math
import time

import cv2
import numpy


_BEST_COLOUR = (255, 255, 0)
_OTHER_COLOUR = (0, 191, 255)
_TEXT_COLOUR = (255, 255, 255)


@dataclass(frozen=True)
class OverlayRegion:
    """A validated, ROS-independent candidate rectangle."""

    x: int
    y: int
    width: int
    height: int
    score: float


class ExactStampBuffer:
    """Pair two bounded streams without ever falling back to latest data."""

    def __init__(self, capacity=8):
        capacity = int(capacity)
        if capacity < 1 or capacity > 64:
            raise ValueError('capacity must be within [1, 64]')
        self.capacity = capacity
        self.images = OrderedDict()
        self.regions = OrderedDict()

    def _insert(self, collection, stamp, value):
        collection[stamp] = value
        collection.move_to_end(stamp)
        while len(collection) > self.capacity:
            collection.popitem(last=False)

    def add_image(self, stamp, image):
        """Add an image and return its exact pending region pair, if any."""

        pending = self.regions.pop(stamp, None)
        if pending is not None:
            return image, pending
        self._insert(self.images, stamp, image)
        return None

    def add_regions(self, stamp, regions):
        """Add regions and return their exact pending image pair, if any."""

        image = self.images.pop(stamp, None)
        if image is not None:
            return image, regions
        self._insert(self.regions, stamp, regions)
        return None


def _visible_region(region, image_width, image_height):
    if (
            region.width <= 0
            or region.height <= 0
            or not math.isfinite(float(region.score))):
        return None
    left = max(0, int(region.x))
    top = max(0, int(region.y))
    right = min(image_width - 1, int(region.x) + int(region.width))
    bottom = min(image_height - 1, int(region.y) + int(region.height))
    if left > right or top > bottom:
        return None
    return left, top, right, bottom


def render_overlay(image, regions, query_id, query_version):
    """Return a labelled copy of one image and its exact-stamp candidates."""

    if not isinstance(image, numpy.ndarray):
        raise TypeError('image must be a numpy array')
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('image must be an HxWx3 colour array')
    if image.shape[0] < 1 or image.shape[1] < 1:
        raise ValueError('image dimensions must be positive')

    output = image.copy()
    height, width = output.shape[:2]
    ordered = []
    for region in regions:
        visible = _visible_region(region, width, height)
        if visible is not None:
            ordered.append((region, visible))
    ordered.sort(
        key=lambda item: (
            -float(item[0].score),
            int(item[0].x),
            int(item[0].y),
            int(item[0].width),
            int(item[0].height),
        ))

    state = 'CANDIDATES' if ordered else 'NO CANDIDATES'
    header = 'query={}/{} {} - NOT GROUND TRUTH'.format(
        int(query_id), int(query_version), state)
    cv2.rectangle(
        output,
        (0, 0),
        (width - 1, min(height - 1, 28)),
        (0, 0, 0),
        -1,
    )
    cv2.putText(
        output,
        header,
        (8, min(height - 1, 20)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        _TEXT_COLOUR,
        1,
        cv2.LINE_AA,
    )
    for rank, (region, visible) in enumerate(ordered, start=1):
        left, top, right, bottom = visible
        colour = _BEST_COLOUR if rank == 1 else _OTHER_COLOUR
        cv2.rectangle(output, (left, top), (right, bottom), colour, 2)
        cv2.putText(
            output,
            '#{} score={:.3f}'.format(rank, float(region.score)),
            (left, max(16, top - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            colour,
            1,
            cv2.LINE_AA,
        )
    return output


def _stamp_key(message):
    stamp = message.header.stamp
    return int(stamp.sec), int(stamp.nanosec)


class SemanticSearchLiveOverlay:
    """ROS adapter kept separate from pure buffering and rendering."""

    def __init__(self):
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import Image
        from track_robot_interfaces.msg import SemanticRegionArray

        class _OverlayNode(Node):
            def __init__(node):
                super().__init__('semantic_search_live_overlay')
                image_topic = node.declare_parameter(
                    'image_topic',
                    '/zed/zed_node/left/image_rect_color',
                ).value
                regions_topic = node.declare_parameter(
                    'regions_topic',
                    '/semantic_search/regions',
                ).value
                output_topic = node.declare_parameter(
                    'overlay_topic',
                    '/semantic_search/overlay_image',
                ).value
                capacity = int(node.declare_parameter(
                    'correlation_capacity', 8).value)
                node._buffer = ExactStampBuffer(capacity=capacity)
                node._bridge = CvBridge()
                node._last_warning_at = 0.0
                reliable = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                )
                output_qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                )
                node._publisher = node.create_publisher(
                    Image, output_topic, output_qos)
                node._image_subscription = node.create_subscription(
                    Image, image_topic, node._on_image,
                    qos_profile_sensor_data)
                node._region_subscription = node.create_subscription(
                    SemanticRegionArray,
                    regions_topic,
                    node._on_regions,
                    reliable,
                )

            def _warn(node, reason):
                now = time.monotonic()
                if now - node._last_warning_at >= 2.0:
                    node.get_logger().warning(reason)
                    node._last_warning_at = now

            def _on_image(node, message):
                pair = node._buffer.add_image(_stamp_key(message), message)
                if pair is not None:
                    node._publish_pair(*pair)

            def _on_regions(node, message):
                pair = node._buffer.add_regions(_stamp_key(message), message)
                if pair is not None:
                    node._publish_pair(*pair)

            def _publish_pair(node, image_message, region_message):
                try:
                    image = node._bridge.imgmsg_to_cv2(
                        image_message, desired_encoding='bgr8')
                    regions = [
                        OverlayRegion(
                            x=int(item.roi.x_offset),
                            y=int(item.roi.y_offset),
                            width=int(item.roi.width),
                            height=int(item.roi.height),
                            score=float(item.fused_score),
                        )
                        for item in region_message.regions
                    ]
                    rendered = render_overlay(
                        image,
                        regions,
                        region_message.query_id,
                        region_message.query_version,
                    )
                    output = node._bridge.cv2_to_imgmsg(
                        rendered, encoding='bgr8')
                    output.header = image_message.header
                    node._publisher.publish(output)
                except Exception as error:
                    node._warn(
                        'Unable to render correlated semantic overlay: '
                        '{}'.format(error))

        self.node = _OverlayNode()
        self._rclpy = rclpy

    def spin(self):
        self._rclpy.spin(self.node)

    def destroy(self):
        self.node.destroy_node()


def main(args=None):
    """Run the passive live-overlay node."""

    import rclpy

    rclpy.init(args=args)
    adapter = None
    try:
        adapter = SemanticSearchLiveOverlay()
        adapter.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if adapter is not None:
            adapter.destroy()
        rclpy.try_shutdown()


__all__ = [
    'ExactStampBuffer',
    'OverlayRegion',
    'SemanticSearchLiveOverlay',
    'main',
    'render_overlay',
]
