#!/usr/bin/env python3

import os
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ZedRgbImageSaver(Node):
    def __init__(self):
        super().__init__("zed_rgb_image_saver")

        self.bridge = CvBridge()

        # ZED 2i RGB image topic
        self.image_topic = "/zed/zed_node/rgb/image_rect_color"

        # Output folder
        self.save_dir = os.path.expanduser(
            "~/track_robot_ws/dataset/zed2i_rgb"
        )
        os.makedirs(self.save_dir, exist_ok=True)

        self.saved = False

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Subscribed to: {self.image_topic}")
        self.get_logger().info(f"Saving image to: {self.save_dir}")
        self.get_logger().info("Waiting for one image message...")

    def image_callback(self, msg):
        if self.saved:
            return

        try:
            # ROS Image -> OpenCV image
            # ZED RGB topic is usually compatible with bgr8 for OpenCV saving
            cv_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert ROS image: {e}")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"zed2i_rgb_{timestamp}.png"
        filepath = os.path.join(self.save_dir, filename)

        success = cv2.imwrite(filepath, cv_image)

        if success:
            self.get_logger().info(f"Saved image: {filepath}")
        else:
            self.get_logger().error(f"Failed to save image: {filepath}")

        self.saved = True
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = ZedRgbImageSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user.")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
