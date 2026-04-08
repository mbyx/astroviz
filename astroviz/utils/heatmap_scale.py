from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel, QSizePolicy
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt
import numpy as np
import cv2

class DepthScaleControl(QWidget):
    """A dedicated widget for displaying a depth heatmap scale."""

    def __init__(self, min_dist: float, max_dist: float, parent=None):
        super().__init__(parent)
        self.min_dist = min_dist
        self.max_dist = max_dist

        self.setFixedWidth(80)
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(5, 5, 5, 5)
        self.layout.setSpacing(5)

        self.title = QLabel("Depth (m)")
        self.title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title.setStyleSheet("font-weight: bold; color: white;")

        self.label_max = QLabel(f"{self.max_dist:.1f}m")
        self.label_max.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.bar_label = QLabel()
        self.bar_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.bar_label.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)
        self.bar_label.setFixedWidth(50)
        self.bar_label.setFixedHeight(250)

        self.label_min = QLabel(f"{self.min_dist:.1f}m")
        self.label_min.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.layout.addWidget(self.title)
        self.layout.addWidget(self.label_max)
        self.layout.addWidget(self.bar_label, stretch=1, alignment=Qt.AlignmentFlag.AlignCenter)
        self.layout.addWidget(self.label_min)

    def resizeEvent(self, event):
        """Redraw the gradient whenever the widget size changes."""
        super().resizeEvent(event)
        self._update_bar()

    def _update_bar(self):
        """Generates the JET colormap gradient."""
        height = self.bar_label.height()
        if height <= 0:
            return

        gradient = np.linspace(255, 0, height).astype(np.uint8).reshape(-1, 1)
        bar_gray = np.tile(gradient, (1, 30))
        bar_color = cv2.applyColorMap(bar_gray, cv2.COLORMAP_JET)

        h, w, c = bar_color.shape
        q_img = QImage(bar_color.data, w, h, w * c, QImage.Format.Format_BGR888)
        self.bar_label.setPixmap(QPixmap.fromImage(q_img))

    def update_range(self, min_m: float, max_m: float):
        """Dynamically update labels if the sensor range changes."""
        self.min_dist = min_m
        self.max_dist = max_m
        self.label_max.setText(f"{max_m:.1f}m")
        self.label_min.setText(f"{min_m:.1f}m")