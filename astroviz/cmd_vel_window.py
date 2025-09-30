#!/usr/bin/env python3
import sys
import math
import os
from typing import Optional

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
from sensor_msgs.msg import LaserScan

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


class CmdVelScanViewer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # TODO: take these from params
        self.max_x = 0.5
        self.max_y = 0.5
        self.max_yaw = 1.0

        # LaserScan drawing state
        self._scan: Optional[LaserScan] = None
        self.scan_thresh: float = (
            0.6  # meters; can be overridden via ROS param
        )

    def set_cmd_vel(self, x: float, y: float, yaw: float):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.update()

    def set_scan(self, scan: LaserScan):
        self._scan = scan
        self.update()

    def set_scan_threshold(self, meters: float):
        if meters <= 0:
            meters = 0.001  # avoid divide-by-zero; effectively hides points
        if abs(self.scan_thresh - meters) > 1e-6:
            self.scan_thresh = meters
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        size = min(w, h)
        r_full = size / 2
        margin = 60
        r = max(1.0, r_full - margin)
        cx, cy = w / 2, h / 2

        # draw circle
        size_i = int(r * 2)
        x0, y0 = int(cx - r), int(cy - r)
        circ_rect = QRect(x0, y0, size_i, size_i)

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

        # draw obstalce lines
        painter.setPen(QPen(Qt.GlobalColor.white, 1))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        tl_x = int(cx - int(self.scan_thresh * 60))
        tl_y = int(cy - int(self.scan_thresh * 90))
        tr_x = int(cx + int(self.scan_thresh * 60))
        tr_y = int(cy - int(self.scan_thresh * 90))
        bl_x = int(cx - int(self.scan_thresh * 60))
        bl_y = int(cy + int(self.scan_thresh * 90))
        br_x = int(cx + int(self.scan_thresh * 60))
        br_y = int(cy + int(self.scan_thresh * 90))
        bar_len = r * 0.86
        painter.drawLine(tl_x, tl_y, tr_x, tr_y)
        painter.drawLine(bl_x, bl_y, br_x, br_y)
        painter.drawLine(tl_x, tl_y, bl_x, bl_y)
        painter.drawLine(tr_x, tr_y, br_x, br_y)

        # draw linear vel arrow
        painter.setPen(QPen(Qt.GlobalColor.yellow, 6))
        # NOTE: x and y are swapped and inverted in Qt
        # cmd-vel's convention is x forward and y towards right
        y_len = -self.x / self.max_x * r
        x_len = -self.y / self.max_y * r
        x1, y1 = cx, cy
        x2, y2 = cx + x_len, cy + y_len

        painter.drawLine(int(x1), int(y1), int(x2), int(y2))

        if self.x != 0 or self.y != 0:
            line_len = math.sqrt(x_len**2 + y_len**2)
            arrow_len = 0.1 * line_len
            angle = math.atan2(y2 - y1, x2 - x1)  # angle of the tip's lines
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
                arc_span_deg = abs(self.yaw / self.max_yaw * 30)  # degrees

                # Direction: left (positive yaw) or right (negative yaw)
                sign = 1 if self.yaw >= 0 else -1

                # Draw arc (Qt expects 1/16 degrees for start & span)
                rect = QRectF(
                    cx - arc_radius,
                    cy - arc_radius,
                    2 * arc_radius,
                    2 * arc_radius,
                )
                start_angle = angle_offs * 16  # 0° = east, counterclockwise +
                span_angle = sign * arc_span_deg * 16

                painter.setPen(QPen(Qt.GlobalColor.green, 4))
                painter.drawArc(rect, int(start_angle), int(span_angle))

                # Compute endpoint of arc
                end_angle = (angle_offs + sign * arc_span_deg) * math.pi / 180
                x_end = cx + arc_radius * math.cos(end_angle)
                y_end = cy - arc_radius * math.sin(end_angle)

                # tangent direction along the arc
                dx = -sign * arc_radius * math.sin(end_angle)
                dy = -sign * arc_radius * math.cos(end_angle)
                tangent_angle = math.atan2(dy, dx)

                # --- arrowhead ---
                arrow_size = 0.1 * arc_radius
                alpha = math.pi / 6  # 30° wing angle

                p1 = QPointF(
                    x_end - arrow_size * math.cos(tangent_angle - alpha),
                    y_end - arrow_size * math.sin(tangent_angle - alpha),
                )
                p2 = QPointF(
                    x_end - arrow_size * math.cos(tangent_angle + alpha),
                    y_end - arrow_size * math.sin(tangent_angle + alpha),
                )

                painter.setPen(QPen(Qt.GlobalColor.green, 4))
                painter.drawLine(QPointF(x_end, y_end), p1)
                painter.drawLine(QPointF(x_end, y_end), p2)

            draw_yaw(0)
            draw_yaw(90)
            draw_yaw(180)
            draw_yaw(270)

        # -------------------- LaserScan points (red) --------------------
        if self._scan is not None and self.scan_thresh > 0:
            painter.setPen(QPen(Qt.GlobalColor.red, 5))
            scale = r / self.scan_thresh  # threshold maps to circle radius
            angle = self._scan.angle_min
            angle = angle + math.pi / 2  # rotate to match viewer's angle
            inc = (
                self._scan.angle_increment
                if self._scan.angle_increment != 0
                else 0.0
            )
            rng_min = max(0.0, self._scan.range_min)
            rng_max = (
                self._scan.range_max
                if self._scan.range_max > 0
                else float("inf")
            )

            for i, rng in enumerate(self._scan.ranges):
                a = angle + i * inc
                if math.isfinite(rng) and rng >= rng_min:
                    if rng < self.scan_thresh and rng < rng_max:
                        # Convert polar → pixel coords; +x right, +y up
                        px = cx + (rng * scale) * math.cos(a)
                        py = cy - (rng * scale) * math.sin(a)
                        painter.drawPoint(int(px), int(py))


class MainWindow(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Mobile Base Viewer")
        self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, "astroviz_icon.png")))

        self.setGeometry(100, 100, 500, 500)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.cmd_vel_viewer = CmdVelScanViewer()
        layout.addWidget(self.cmd_vel_viewer)

        # --- Selectors (top-right) ---
        self.cmd_combo = QComboBox(self.centralWidget())
        self.cmd_combo.setFixedWidth(180)
        self.cmd_combo.currentTextChanged.connect(self.change_cmd_topic)

        self.scan_combo = QComboBox(self.centralWidget())
        self.scan_combo.setFixedWidth(180)
        self.scan_combo.currentTextChanged.connect(self.change_scan_topic)

        # ROS params
        try:
            self.node.declare_parameter("scan_threshold", 1.0)
        except Exception:
            pass  # already declared
        th = (
            self.node.get_parameter("scan_threshold").value
            if self.node.has_parameter("scan_threshold")
            else 1.0
        )
        self.cmd_vel_viewer.set_scan_threshold(float(th))

        # Subscriptions
        self.twist_sub = None
        self.scan_sub = None

        self._populate_topics()

        # Timers
        self.topic_timer = QTimer(self)
        self.topic_timer.timeout.connect(self._populate_topics)
        self.topic_timer.start(1000)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(
            lambda: rclpy.spin_once(self.node, timeout_sec=0)
        )
        self.ros_timer.start(50)

        # Poll param in case changed externally
        self.param_timer = QTimer(self)
        self.param_timer.timeout.connect(self._poll_scan_threshold_param)
        self.param_timer.start(500)

        # Window update timer
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: self.cmd_vel_viewer.update())
        self.ros_timer.start(20)

    def _poll_scan_threshold_param(self):
        try:
            p = self.node.get_parameter("scan_threshold").value
            self.cmd_vel_viewer.set_scan_threshold(float(p))
        except Exception:
            pass

    def showEvent(self, event):
        super().showEvent(event)
        self._reposition_selectors()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._reposition_selectors()

    def _reposition_selectors(self):
        margin = 5
        spacing = 6
        cw = self.centralWidget().width()
        # place cmd combo at top-right
        x = cw - self.cmd_combo.width() - margin
        self.cmd_combo.move(x, margin)
        # place scan combo below
        self.scan_combo.move(x, margin + self.cmd_combo.height() + spacing)

    def _populate_topics(self):
        # Twist topics
        current_cmd = self.cmd_combo.currentText()
        # LaserScan topics
        current_scan = self.scan_combo.currentText()

        all_topics = self.node.get_topic_names_and_types()
        twist_topics = [
            name
            for name, types in all_topics
            if "geometry_msgs/msg/Twist" in types
        ]
        scan_topics = [
            name
            for name, types in all_topics
            if "sensor_msgs/msg/LaserScan" in types
        ]

        cmd_items = ["---"] + twist_topics
        scan_items = ["---"] + scan_topics

        old_cmd = [
            self.cmd_combo.itemText(i) for i in range(self.cmd_combo.count())
        ]
        old_scan = [
            self.scan_combo.itemText(i) for i in range(self.scan_combo.count())
        ]

        if old_cmd != cmd_items:
            self.cmd_combo.blockSignals(True)
            self.cmd_combo.clear()
            self.cmd_combo.addItems(cmd_items)
            if current_cmd in cmd_items:
                self.cmd_combo.setCurrentText(current_cmd)
            else:
                self.cmd_combo.setCurrentIndex(0)
                self.change_cmd_topic("---")
            self.cmd_combo.blockSignals(False)

        if old_scan != scan_items:
            self.scan_combo.blockSignals(True)
            self.scan_combo.clear()
            self.scan_combo.addItems(scan_items)
            if current_scan in scan_items:
                self.scan_combo.setCurrentText(current_scan)
            else:
                self.scan_combo.setCurrentIndex(0)
                self.change_scan_topic("---")
            self.scan_combo.blockSignals(False)

    def change_cmd_topic(self, topic_name: str):
        if self.twist_sub is not None:
            try:
                self.node.destroy_subscription(self.twist_sub)
            except Exception:
                pass
            self.twist_sub = None

        if topic_name == "---":
            return

        self.twist_sub = self.node.create_subscription(
            Twist, topic_name, self._twist_cb, QoSProfile(depth=1)
        )

    def change_scan_topic(self, topic_name: str):
        if self.scan_sub is not None:
            try:
                self.node.destroy_subscription(self.scan_sub)
            except Exception:
                pass
            self.scan_sub = None

        if topic_name == "---":
            return

        self.scan_sub = self.node.create_subscription(
            LaserScan, topic_name, self._scan_cb, QoSProfile(depth=1)
        )

    def _twist_cb(self, msg: Twist):
        x, y, yaw = msg.linear.x, msg.linear.y, msg.angular.z
        self.cmd_vel_viewer.set_cmd_vel(x, y, yaw)

    def _scan_cb(self, msg: LaserScan):
        self.cmd_vel_viewer.set_scan(msg)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    DarkStyle(app)
    node = rclpy.create_node("cmd_vel_laserscan_window")
    window = MainWindow(node)
    window.show()
    app.exec()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
