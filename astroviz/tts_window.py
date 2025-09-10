#!/usr/bin/env python3
import sys
import os

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QComboBox,
    QSizePolicy,
    QSpacerItem,
)
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import QTimer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from std_msgs.msg import String

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


class MainWindow(QMainWindow):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("TTS Window")
        self.setWindowIcon(QIcon(os.path.join(ICONS_DIR, "astroviz_icon.png")))

        self.setGeometry(100, 100, 300, 150)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(10, 60, 10, 10)  # left, top, right, bottom

        # Topic selector
        self.combo = QComboBox(self.centralWidget())
        self.combo.setFixedWidth(150)
        self.combo.raise_()
        self.combo.currentTextChanged.connect(self.change_topic)

        # Add spacer to keep the buttons close together
        layout.addItem(
            QSpacerItem(
                0,
                0,
                QSizePolicy.Policy.Minimum,
                QSizePolicy.Policy.Expanding,
            )
        )

        # Customize status bar
        sb = self.statusBar()
        sb.setSizeGripEnabled(True)
        sb.setStyleSheet(
            """
            QStatusBar {
                background: #3a3a3a;
                color: lightgrey;
                font-style: italic;
                border-top: 1px solid #444;
            }
        """
        )

        # Buttons for pre-defined messages
        predef_msgs = [
            "Hello, I'm Shelfy!",
            "Please, fill your order by pressing the buttons.",
        ]
        for msg in predef_msgs:
            btn = QPushButton(msg)
            btn.clicked.connect(lambda _, m=msg: self.send_predef_msg(m))
            layout.addWidget(btn)
        layout.addSpacing(15)  # add some space to separate from the next part

        # Input box for custom messages
        self.input_box = QLineEdit()
        self.input_box.setPlaceholderText("Type something")
        self.button_custom_msg = QPushButton("Send custom message")
        self.button_custom_msg.clicked.connect(self.send_custom_msg)

        # Add widgets to layout
        layout.addWidget(self.input_box)
        layout.addWidget(self.button_custom_msg)

        self.str_pub = None
        self._populate_topics()

        self.topic_timer = QTimer(self)
        self.topic_timer.timeout.connect(self._populate_topics)
        self.topic_timer.start(1000)

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(
            lambda: rclpy.spin_once(self.node, timeout_sec=0)
        )
        self.ros_timer.start(50)

    def send_predef_msg(self, message: str):
        if not self.str_pub:
            self.statusBar().showMessage("No topic selected", 3000)
            return
        self.str_pub.publish(String(data=message))
        self.statusBar().showMessage(f"Sent: '{message}'", 2000)

    def send_custom_msg(self):
        text = self.input_box.text()
        if self.str_pub is not None:
            msg = String()
            msg.data = text
            self.str_pub.publish(msg)
            self.statusBar().showMessage(f"Sent: '{text}'", 2000)
        else:
            self.statusBar().showMessage("No topic selected", 3000)
        self.input_box.clear()

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
            if "std_msgs/msg/String" in types
        ]
        items = ["---"] + imu_topics

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
            self.change_topic("---")
        self.combo.blockSignals(False)

    def change_topic(self, topic_name: str):
        if self.str_pub is not None:
            try:
                self.node.destroy_publisher(self.str_pub)
            except Exception:
                pass
            self.str_pub = None

        if topic_name == "---":
            return

        self.str_pub = self.node.create_publisher(
            String, topic_name, QoSProfile(depth=1)
        )


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    # DarkStyle(app)
    node = rclpy.create_node("tts_window")
    window = MainWindow(node)
    window.show()
    app.exec()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
