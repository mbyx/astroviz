#!/usr/bin/env python3
import sys
import math
import os

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QComboBox,
)
from PyQt6.QtGui import QPainter, QPen, QIcon
from PyQt6.QtCore import Qt, QRect, QRectF, QTimer, QPointF

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

from ament_index_python.packages import get_package_share_directory
from astroviz.utils.window_style import DarkStyle
from astroviz.common._find import _find_pkg, _find_src_config

_src_config = _find_src_config()
if _src_config:
    _CONFIG_DIR = _src_config
else:
    _CONFIG_DIR = os.path.join(
        get_package_share_directory("astroviz"), "config"
    )


_pkg = _find_pkg()
if _pkg:
    _PKG_DIR = _pkg
else:
    _PKG_DIR = get_package_share_directory("astroviz")

os.makedirs(_CONFIG_DIR, exist_ok=True)

CONFIG_PATH = os.path.join(_CONFIG_DIR, "dashboard_config.json")
ICONS_DIR = os.path.join(_PKG_DIR, "icons")


class CmdVelViewer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.cmd_vel = Twist()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # TODO: take these from params
        self.max_x = 1.0
        self.max_y = 1.0
        self.max_yaw = 1.0

    def set_cmd_vel(self, x: float, y: float, yaw: float):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        size = min(w, h)
        r_full = size / 2
        margin = 60
        r = r_full - margin
        cx, cy = w / 2, h / 2

        size_i = int(r * 2)
        x0, y0 = int(cx - r), int(cy - r)
        circ_rect = QRect(x0, y0, size_i, size_i)

        # draw circle
        painter.setPen(QPen(Qt.GlobalColor.white, 3))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(circ_rect)

        # draw cross
        bar_len = r
        painter.drawLine(
            int(cx - bar_len), int(cy), int(cx + bar_len), int(cy)
        )
        painter.drawLine(
            int(cx), int(cy - bar_len), int(cx), int(cy + bar_len)
        )
        # # draw sides
        # painter.drawLine(
        #     int(cx - bar_len), int(cy + bar_len),
        #     int(cx - bar_len), int(cy - bar_len)
        # )
        # painter.drawLine(
        #     int(cx + bar_len), int(cy + bar_len),
        #     int(cx - bar_len), int(cy + bar_len)
        # )
        # painter.drawLine(
        #     int(cx + bar_len), int(cy + bar_len),
        #     int(cx + bar_len), int(cy - bar_len)
        # )
        # painter.drawLine(
        #     int(cx + bar_len), int(cy - bar_len),
        #     int(cx - bar_len), int(cy - bar_len)
        # )

        # draw linear vel arrow
        painter.setPen(QPen(Qt.GlobalColor.yellow, 6))
        # NOTE: x and y are swapped and inverted in Qt
        # cmd-vel's convention is x forward and y towards right
        y_len = -self.x / self.max_x * 2 * r
        x_len = -self.y / self.max_y * 2 * r
        x1, y1 = cx, cy
        x2, y2 = cx + x_len, cy + y_len

        painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        if self.x != 0 or self.y != 0:
            line_len = math.sqrt(x_len**2 + y_len**2)
            arrow_len = 0.1 * line_len
            angle = math.atan2(
                y2 - y1, x2 - x1
            )  # angle of the tip's lines
            painter.drawLine(
                int(x2),
                int(y2),
                int(x2 - arrow_len * math.cos(angle - math.pi / 6)),
                int(y2 - arrow_len * math.sin(angle - math.pi / 6)),
            )
            painter.drawLine(
                int(x2),
                int(y2),
                int(x2 - arrow_len * math.cos(angle + math.pi / 6)),
                int(y2 - arrow_len * math.sin(angle + math.pi / 6)),
            )

        if self.yaw != 0:

            def draw_yaw(angle_offs: int):
                # Arc parameters
                arc_radius = r * 1.2
                arc_span_deg = abs(
                    self.yaw / self.max_yaw * 30
                )  # degrees

                # Direction: left (positive yaw) or right (negative yaw)
                sign = 1 if self.yaw >= 0 else -1

                # Draw arc (Qt expects 1/16 degrees for start & span)
                rect = QRectF(
                    cx - arc_radius,
                    cy - arc_radius,
                    2 * arc_radius,
                    2 * arc_radius,
                )
                start_angle = (
                    angle_offs * 16
                )  # 0° = east, counterclockwise +
                span_angle = sign * arc_span_deg * 16

                painter.setPen(QPen(Qt.GlobalColor.green, 4))
                painter.drawArc(rect, int(start_angle), int(span_angle))

                # Compute endpoint of arc
                end_angle = (
                    (angle_offs + sign * arc_span_deg) * math.pi / 180
                )
                x_end = cx + arc_radius * math.cos(end_angle)
                y_end = cy - arc_radius * math.sin(end_angle)

                # Compute tangent vector (points along the arc direction)
                # Parameterization:
                # x = cx + R*cos(theta), y = cy - R*sin(theta)
                # For increasing theta the derivative is:
                # dx = -R*sin(theta), dy = -R*cos(theta)
                # If the arc runs CW (sign = -1) the direction is reversed,
                # so include sign.
                dx = -sign * arc_radius * math.sin(end_angle)
                dy = -sign * arc_radius * math.cos(end_angle)

                # tangent direction along the arc
                tangent_angle = math.atan2(dy, dx)

                # --- arrowhead ---
                arrow_size = 0.1 * arc_radius
                alpha = math.pi / 6  # 30° wing angle

                p1 = QPointF(
                    x_end
                    - arrow_size * math.cos(tangent_angle - alpha),
                    y_end
                    - arrow_size * math.sin(tangent_angle - alpha),
                )
                p2 = QPointF(
                    x_end
                    - arrow_size * math.cos(tangent_angle + alpha),
                    y_end
                    - arrow_size * math.sin(tangent_angle + alpha),
                )

                painter.setPen(QPen(Qt.GlobalColor.green, 4))
                painter.drawLine(QPointF(x_end, y_end), p1)
                painter.drawLine(QPointF(x_end, y_end), p2)

            draw_yaw(0)
            draw_yaw(90)
            draw_yaw(180)
            draw_yaw(270)


class MainWindow(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Cmd-Vel Direction")
        self.setWindowIcon(
            QIcon(os.path.join(ICONS_DIR, "astroviz_icon.png"))
        )

        self.setGeometry(100, 100, 400, 400)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.cmd_vel_viewer = CmdVelViewer()
        layout.addWidget(self.cmd_vel_viewer)

        self.combo = QComboBox(self.centralWidget())
        self.combo.setFixedWidth(150)
        self.combo.raise_()
        self.combo.currentTextChanged.connect(self.change_topic)

        self.imu_sub = None
        self._populate_topics()

        self.topic_timer = QTimer(self)
        self.topic_timer.timeout.connect(self._populate_topics)
        self.topic_timer.start(1000)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(
            lambda: rclpy.spin_once(self.node, timeout_sec=0)
        )
        self.ros_timer.start(50)

    def showEvent(self, event):
        super().showEvent(event)
        self._reposition_combo()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reposition_combo()

    def _reposition_combo(self):
        margin = 5
        cw = self.centralWidget().width()
        x = cw - self.combo.width() - margin
        y = margin
        self.combo.move(x, y)

    def _populate_topics(self):
        current = self.combo.currentText()
        all_topics = self.node.get_topic_names_and_types()
        imu_topics = [
            name
            for name, types in all_topics
            if "geometry_msgs/msg/Twist" in types
        ]
        items = ["---"] + imu_topics

        old = [
            self.combo.itemText(i) for i in range(self.combo.count())
        ]
        if old == items:
            return

        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItems(items)
        if current in items:
            self.combo.setCurrentText(current)
        else:
            self.combo.setCurrentIndex(0)
            self.change_topic("---")
        self.combo.blockSignals(False)

    def change_topic(self, topic_name: str):
        if self.imu_sub is not None:
            try:
                self.node.destroy_subscription(self.imu_sub)
            except Exception:
                pass
            self.imu_sub = None

        if topic_name == "---":
            return

        self.imu_sub = self.node.create_subscription(
            Twist, topic_name, self.callback, QoSProfile(depth=1)
        )

    def callback(self, msg: Twist):
        x, y, yaw = (
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
        )
        self.cmd_vel_viewer.set_cmd_vel(x, y, yaw)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    DarkStyle(app)
    node = rclpy.create_node("cmd_vel_window")
    window = MainWindow(node)
    window.show()
    app.exec()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
