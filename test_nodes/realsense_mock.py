# Run with `python realsense_mock.py`.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge

FPS: float = 30.0

class MockRealSenseNode(Node):
    """Emulates a realsense camera by sending fake color and depth images to topics.

    The depth camera sends its data in an Image message with the U16C1 encoding,
    the RGB camera sends its data in an Image message with the BGR888 encoding."""
    def __init__(self):
        super().__init__('mock_realsense_node')

        # RealSense D435i standard topics
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)

        self.timer = self.create_timer(1.0 / FPS, self.timer_callback)
        self.bridge = CvBridge()

        self.width = 1280
        self.height = 720

        # Used to make a dynamic image for both topics.
        self.offset = 0.0

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        x = np.linspace(0, 4 * np.pi, self.width)
        wave = (np.sin(x + self.offset) + 1) / 2
        self.offset += 0.1

        base_2d = (wave * 255).astype(np.uint8)
        img_8bit = np.tile(base_2d, (self.height, 1))

        depth_mm = (img_8bit.astype(np.uint16) * 20) + 500
        depth_msg = self.bridge.cv2_to_imgmsg(depth_mm, encoding="16UC1")
        depth_msg.header.stamp = now
        depth_msg.header.frame_id = "camera_depth_optical_frame"

        color_img = cv2.applyColorMap(img_8bit, cv2.COLORMAP_JET)
        color_msg = self.bridge.cv2_to_imgmsg(color_img, encoding="bgr8")
        color_msg.header.stamp = now
        color_msg.header.frame_id = "camera_color_optical_frame"

        self.depth_pub.publish(depth_msg)
        self.color_pub.publish(color_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockRealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()