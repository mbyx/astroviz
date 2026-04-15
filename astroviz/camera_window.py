#!/usr/bin/env python3

import sys
import cv2
import numpy as np
import os

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QSizePolicy,
    QWidget,
    QVBoxLayout,
    QLabel,
    QComboBox,
    QHBoxLayout,
    QPushButton,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPixmap, QImage, QIcon

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


from astroviz.utils.window_style import DarkStyle
from astroviz.utils.heatmap_scale import HeatmapScaleControl
from astroviz.common._find import _find_pkg, _find_src_config

_src_config = _find_src_config()
if _src_config:
    _CONFIG_DIR = _src_config
else:
    _CONFIG_DIR = os.path.join(get_package_share_directory("astroviz"), "config")


_pkg = _find_pkg()
if _pkg:
    _PKG_DIR = _pkg
else:
    _PKG_DIR = get_package_share_directory("astroviz")

os.makedirs(_CONFIG_DIR, exist_ok=True)

CONFIG_PATH = os.path.join(_CONFIG_DIR, "dashboard_config.json")
ICONS_DIR = os.path.join(_PKG_DIR, "icons")

# Based on the optimum distance for a realsense 435i camera.
MIN_DISTANCE_MM = 300
MAX_DISTANCE_MM = 3000


def get_image_topic_type(node: Node, topic_name: str) -> str | None:
    """
    Check if a given topic is of type Image or CompressedImage.

    Args:
        node (Node): An active rclpy Node.
        topic_name (str): The name of the topic to check.

    Returns:
        Image | CompressedImage | None
    """
    mapping = {
        "sensor_msgs/msg/Image": Image,
        "sensor_msgs/msg/CompressedImage": CompressedImage,
    }

    topics = node.get_topic_names_and_types()
    for name, types in topics:
        if name == topic_name:
            for t in types:
                if t in mapping:
                    return mapping[t]
            return None
    return None


class CameraViewer(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Camera Viewer")
        self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, "astroviz_icon.png")))

        self.bridge = CvBridge()
        self.image_sub = None
        self.rotation_angle = 0

        self.central = QWidget()
        self.setCentralWidget(self.central)
        self.layout = QVBoxLayout(self.central)

        self.combo = QComboBox()
        self.combo.setFixedWidth(250)
        self.combo.currentTextChanged.connect(self.change_image_topic)
        self.layout.addWidget(self.combo, alignment=Qt.AlignmentFlag.AlignLeft)

        content_layout = QHBoxLayout()

        self.image_label = QLabel("Waiting for image...")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )

        self.depth_scale = HeatmapScaleControl(
            min_dist=MIN_DISTANCE_MM / 1000.0, max_dist=MAX_DISTANCE_MM / 1000.0
        )
        self.depth_scale.hide()

        content_layout.addWidget(self.image_label)
        content_layout.addWidget(self.depth_scale)

        self.layout.addLayout(content_layout)

        btn_layout = QHBoxLayout()
        self.btn_left = QPushButton("⟲ 90° Left")
        self.btn_left.clicked.connect(self.rotate_left)
        btn_layout.addWidget(self.btn_left, alignment=Qt.AlignmentFlag.AlignLeft)

        btn_layout.addStretch()

        self.btn_right = QPushButton("90° Right ⟳")
        self.btn_right.clicked.connect(self.rotate_right)
        btn_layout.addWidget(self.btn_right, alignment=Qt.AlignmentFlag.AlignRight)

        self.layout.addLayout(btn_layout)

        self.topic_timer = QTimer()
        self.topic_timer.timeout.connect(self.update_image_topics)
        self.topic_timer.start(1000)

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(
            lambda: rclpy.spin_once(self.node, timeout_sec=0)
        )
        self.ros_timer.start(30)

    def rotate_left(self):
        self.rotation_angle = (self.rotation_angle - 90) % 360

    def rotate_right(self):
        self.rotation_angle = (self.rotation_angle + 90) % 360

    def update_image_topics(self):
        current = self.combo.currentText()
        all_topics = self.node.get_topic_names_and_types()
        image_topics = [
            name
            for name, types in all_topics
            if (
                "sensor_msgs/msg/Image" in types
                or "sensor_msgs/msg/CompressedImage" in types
            )
        ]
        items = ["---"] + image_topics

        old = [self.combo.itemText(i) for i in range(self.combo.count())]
        if old == items:
            return

        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItems(items)
        if current in items:
            self.combo.setCurrentText(current)
        else:
            self.combo.setCurrentIndex(0)
            self.change_image_topic("---")
        self.combo.blockSignals(False)

    def change_image_topic(self, topic_name: str):
        if self.image_sub is not None:
            try:
                self.node.destroy_subscription(self.image_sub)
            except Exception:
                pass
            self.image_sub = None

        if topic_name == "---":
            return

        self.image_sub = self.node.create_subscription(
            get_image_topic_type(self.node, topic_name),
            topic_name,
            self.image_callback,
            QoSProfile(depth=10),
        )

    def _get_color_bar_pixmap(self, height: int, width: int = 40):
        gradient = np.linspace(255, 0, height).astype(np.uint8).reshape(-1, 1)
        bar_gray = np.tile(gradient, (1, width))
        bar_color = cv2.applyColorMap(bar_gray, cv2.COLORMAP_JET)
        h, w, c = bar_color.shape
        q_img = QImage(bar_color.data, w, h, w * c, QImage.Format.Format_BGR888)
        return QPixmap.fromImage(q_img)

    def image_callback(self, msg: Image | CompressedImage):
        """Attempts to create a viewable image from given Image message

        Although the message is of the Image type, due to differences in
        encoding, sometimes it is not feasible to show an image in the same
        way for every type of image. For example, a depth image can incorporate
        the depth into every pixel, which causes a different image encoding
        to be used. This encoding may be better visualized using a heatmap,
        which is what this method does."""
        try:
            msg_name = msg.__class__.__name__

            if msg_name == "CompressedImage":
                cv_raw = self.bridge.compressed_imgmsg_to_cv2(
                    msg, desired_encoding="rgb8"
                )
            else:
                if msg.encoding == "16UC1":
                    cv_raw = self.bridge.imgmsg_to_cv2(
                        msg, desired_encoding="passthrough"
                    )
                else:
                    cv_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

            # This encoding is used by depth images.
            if msg.encoding == "16UC1":
                if self.depth_scale.heatmap_type.currentText().lower() == "depth":
                    # The points are clipped to the optimum distance.
                    cv_clipped = np.clip(cv_raw, MIN_DISTANCE_MM, MAX_DISTANCE_MM)
                    # Scale the depth values to correspond to a grayscale color.
                    alpha = 255.0 / (MAX_DISTANCE_MM - MIN_DISTANCE_MM)
                    gray_8bit = cv2.convertScaleAbs(
                        cv_clipped - MIN_DISTANCE_MM, alpha=alpha
                    )
                    gray_8bit_blurred = cv2.medianBlur(gray_8bit, 3)

                    # Create a heatmap from the grayscale mapping.
                    cv_image_bgr = cv2.applyColorMap(
                        gray_8bit_blurred, cv2.COLORMAP_JET
                    )
                    cv_image = cv2.cvtColor(cv_image_bgr, cv2.COLOR_RGB2BGR)
                elif self.depth_scale.heatmap_type.currentText().lower() == "disparity":
                    depth_safe = np.maximum(cv_raw, 1.0)

                    # 'baseline_focal_constant' = baseline (mm) * focal_length (pixels)
                    baseline_focal_constant = 32000
                    disparity = baseline_focal_constant / depth_safe

                    MIN_DISP, MAX_DISP = 0, 128
                    disp_clipped = np.clip(disparity, MIN_DISP, MAX_DISP)

                    alpha = 255.0 / (MAX_DISP - MIN_DISP)
                    gray_8bit = cv2.convertScaleAbs(
                        disp_clipped - MIN_DISP, alpha=alpha
                    )

                    gray_8bit_blurred = cv2.medianBlur(gray_8bit, 3)
                    cv_image_bgr = cv2.applyColorMap(
                        gray_8bit_blurred, cv2.COLORMAP_JET
                    )
                    cv_image = cv2.cvtColor(cv_image_bgr, cv2.COLOR_RGB2BGR)
                self.depth_scale.show()
            else:
                cv_image = cv_raw
                self.depth_scale.hide()

            if self.rotation_angle == 90:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            elif self.rotation_angle == 180:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)
            elif self.rotation_angle == 270:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

            # Format detection for QImage
            if len(cv_image.shape) == 3:
                height, width, channel = cv_image.shape
                bytes_per_line = channel * width
                q_format = QImage.Format.Format_RGB888
            else:
                height, width = cv_image.shape
                bytes_per_line = width
                q_format = QImage.Format.Format_Grayscale8

            q_image = QImage(
                cv_image.data,
                width,
                height,
                bytes_per_line,
                q_format,
            )

            pixmap = QPixmap.fromImage(q_image)
            self.image_label.setPixmap(
                pixmap.scaled(
                    self.image_label.size(), Qt.AspectRatioMode.KeepAspectRatio
                )
            )
        except Exception as e:
            self.node.get_logger().warn(f"Image conversion failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    DarkStyle(app)

    node = rclpy.create_node("camera_viewer")
    window = CameraViewer(node)
    window.show()
    app.exec()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
