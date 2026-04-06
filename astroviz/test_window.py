#!/usr/bin/env python3
import sys

import rclpy
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from rclpy.node import Node

from astroviz.utils.window_style import DarkStyle


class TestWindow(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.setWindowTitle("AstroViz Test")
        self.resize(400, 300)

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)

        label = QLabel("Hello World")

        font = QFont("Arial", 24, QFont.Weight.Bold)
        label.setFont(font)

        label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(label)
        self.setLayout(layout)


def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)

    if DarkStyle:
        DarkStyle(app)

    node = rclpy.create_node("test_window_node")

    win = TestWindow(node)
    win.show()

    exit_code = app.exec()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
