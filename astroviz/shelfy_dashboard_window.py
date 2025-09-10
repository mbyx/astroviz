#!/usr/bin/env python3
import sys
import os
from typing import Optional, Tuple, List

from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QGridLayout,
    QScrollArea,
    QSizePolicy,
    QSplitter,
    QVBoxLayout,
)
from PyQt6.QtGui import QIcon
from PyQt6.QtCore import Qt

import rclpy
from rclpy.node import Node

# Import your existing windows
from astroviz.tts_window import MainWindow as TTSWindow
from astroviz.cmd_vel_window import MainWindow as CmdVelWindow
from astroviz.gstreamer_shelfy_window import GstreamerWindow

from ament_index_python.packages import get_package_share_directory
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


class MultiWindowHost(QMainWindow):
    """
    Host window:
      - Left pane: a single widget (e.g., GStreamer video) in a QSplitter.
      - Right pane: a scrollable QGridLayout for other widgets.
    """

    def __init__(self, grid_shape: Tuple[int, int] = (2, 2)):
        super().__init__()
        self.setWindowTitle("Shelfy Teleoperation Dashboard")
        self.setWindowIcon(
            QIcon(os.path.join(ICONS_DIR, "astroviz_icon.png"))
        )
        self._owned_widgets: List[QWidget] = []

        self._rows, self._cols = grid_shape
        self._next_index = 0  # for auto placement on the right grid

        # ---------------- Splitter: [ left | right ] ----------------
        self._splitter = QSplitter(Qt.Orientation.Horizontal, self)
        self.setCentralWidget(self._splitter)

        # Left container (will hold exactly one child widget, e.g., GStreamer)
        self._left_container = QWidget(self._splitter)
        self._left_container.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        self._left_layout = QVBoxLayout(self._left_container)
        self._left_layout.setContentsMargins(0, 0, 0, 0)
        self._left_layout.setSpacing(0)
        self._splitter.addWidget(self._left_container)

        # Right side: scrollable grid container
        self._right_scroll = QScrollArea(self._splitter)
        self._right_scroll.setWidgetResizable(True)

        self._right_container = QWidget()
        self._right_scroll.setWidget(self._right_container)

        self._grid = QGridLayout(self._right_container)
        self._grid.setContentsMargins(10, 10, 10, 10)
        self._grid.setHorizontalSpacing(10)
        self._grid.setVerticalSpacing(10)

        self._splitter.addWidget(self._right_scroll)

        # Give the left pane more width by default (tweak to taste)
        self._splitter.setStretchFactor(0, 3)  # left
        self._splitter.setStretchFactor(1, 2)  # right

        # Optional: a subtle status bar for the host
        self.statusBar().setStyleSheet(
            """
            QStatusBar {
                background: #3a3a3a;
                color: #FFFFFF;
                border-top: 1px solid #444;
            }
            """
        )

    # ---------------- Helpers ----------------

    def _normalize_child(self, w: QWidget) -> QWidget:
        """
        Make sure a QMainWindow/QWidget can live as a child panel:
        - remove window decorations
        - give reasonable size policy
        """
        if isinstance(w, QMainWindow):
            w.setWindowFlags(Qt.WindowType.Widget)
        w.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred
        )
        return w

    # ---------------- Public API ----------------

    def add_left_widget(self, w: QWidget):
        """
        Put a single widget on the left side (e.g., the GStreamer view).
        Replaces any existing left widget.
        """
        w = self._normalize_child(w)
        self._owned_widgets.append(w)

        # Clear previous
        while self._left_layout.count():
            item = self._left_layout.takeAt(0)
            old = item.widget()
            if old is not None:
                old.setParent(None)

        self._left_layout.addWidget(w)

    def add_widget(
        self,
        w: QWidget,
        row: Optional[int] = None,
        col: Optional[int] = None,
        row_span: int = 1,
        col_span: int = 1,
    ):
        """
        Add any QWidget (including a QMainWindow instance) into the RIGHT grid.
        If row/col are not provided, auto-place in the next slot.
        """
        w = self._normalize_child(w)
        w.setParent(self._right_container)
        self._owned_widgets.append(w)

        if row is None or col is None:
            idx = self._next_index
            row = idx // self._cols
            col = idx % self._cols
            self._next_index += 1

        self._grid.addWidget(w, row, col, row_span, col_span)

    def set_initial_splitter_sizes(self, left_px: int, right_px: int):
        """Optionally set initial pixel sizes of left/right panes."""
        self._splitter.setSizes([left_px, right_px])

    def closeEvent(self, event):
        for w in self._owned_widgets:
            try:
                w.close()  # triggers their own cleanup
            except Exception:
                pass
        super().closeEvent(event)


def main():
    from astroviz.utils.window_style import DarkStyle

    predef_msgs = [
        "Hello, I'm Shelfy!",
        "Please, fill your order by pressing the buttons.",
        "Please, load the order you see on the screen.",
    ]

    rclpy.init()
    app = QApplication(sys.argv)
    DarkStyle(app)

    # Shared node
    node = rclpy.create_node("astroviz_dashboard")

    host = MultiWindowHost(grid_shape=(2, 2))
    host.resize(1100, 1280)
    host.set_initial_splitter_sizes(left_px=700, right_px=400)

    # LEFT: GStreamer view (occupies entire left pane)
    gst = GstreamerWindow(port=5000, width=960, height=540)
    host.add_left_widget(gst)

    # RIGHT: Grid of other panels
    host.add_widget(TTSWindow(node, predef_msgs), row=0, col=0)
    host.add_widget(CmdVelWindow(node), row=1, col=0)
    # Example: add more to the right grid
    # host.add_widget(AnotherPanel(node),  row=0, col=1)
    # host.add_widget(YetAnother(node),    row=1, col=1)

    host.show()
    app.exec()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
