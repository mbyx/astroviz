from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QComboBox,
)
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtCore import Qt
import numpy as np
import cv2


class HeatmapScaleControl(QWidget):
    """A dynamic heatmap scale that adjusts tick count based on widget height."""

    def __init__(self, min_dist: float, max_dist: float, parent=None):
        super().__init__(parent)
        self.min_dist = min_dist
        self.max_dist = max_dist

        # Pixels required between each numeric tick.
        self.tick_spacing = 50

        self.main_layout = QVBoxLayout(self)
        self.main_layout.setContentsMargins(5, 5, 5, 5)

        self.heatmap_type = QComboBox()
        self.heatmap_type.addItems(["Depth", "Disparity"])
        self.main_layout.addWidget(self.heatmap_type)

        self.title = QLabel("Depth\n(m)")
        self.main_layout.addWidget(self.title)

        self.scale_container = QHBoxLayout()
        self.scale_container.setSpacing(10)

        self.bar_label = QLabel()
        self.bar_label.setFixedWidth(30)
        self.bar_label.setSizePolicy(
            QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding
        )
        self.bar_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.bar_label.setMinimumSize(1, 1)

        self.ticks_layout = QVBoxLayout()
        self.ticks_layout.setContentsMargins(0, 0, 0, 0)
        self.tick_labels = []

        self.scale_container.addWidget(self.bar_label)
        self.scale_container.addLayout(self.ticks_layout)
        self.main_layout.addLayout(self.scale_container)

        self.heatmap_type.currentTextChanged.connect(self._on_type_changed)

        self.create_ticks(4)

    def create_ticks(self, count):
        """Removes old labels and creates new ones based on current height."""
        while self.ticks_layout.count():
            item = self.ticks_layout.takeAt(0)
            widget = item.widget()
            if widget:
                widget.deleteLater()

        self.tick_labels = []
        count = max(5, count)

        for i in range(count):
            label = QLabel("--")
            if i == 0:
                label.setAlignment(
                    Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft
                )
            elif i == count - 1:
                label.setAlignment(
                    Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignLeft
                )
            else:
                label.setAlignment(
                    Qt.AlignmentFlag.AlignCenter | Qt.AlignmentFlag.AlignLeft
                )

            self.ticks_layout.addWidget(label)
            self.tick_labels.append(label)

        self.update_label_text()

    def update_label_text(self):
        """Updates the text values for every visible tick."""
        count = len(self.tick_labels)
        if count < 2:
            return

        values = np.linspace(self.max_dist, self.min_dist, count)
        mode = self.heatmap_type.currentText()
        unit = "px" if mode == "Disparity" else "m"

        for i, val in enumerate(values):
            self.tick_labels[i].setText(f"{val:.1f}{unit}")

    def resizeEvent(self, event):
        """Handle resizing: Update bar image and calculate tick density."""
        super().resizeEvent(event)

        available_height = self.bar_label.height()
        new_count = available_height // self.tick_spacing

        if new_count != len(self.tick_labels) and available_height > 10:
            self.create_ticks(new_count)

        self._update_bar()

    def _update_bar(self):
        """Redraws the OpenCV JET colormap to match the current label height."""
        height = self.bar_label.height()
        width = self.bar_label.width()

        if height <= 2:
            return

        # Generate a vertical gradient (0-255)
        # Note: 0 is Blue (cold/far), 255 is Red (hot/near)
        gradient = np.linspace(0, 255, height).astype(np.uint8).reshape(-1, 1)
        bar_gray = np.tile(gradient, (1, width))
        bar_color = cv2.applyColorMap(bar_gray, cv2.COLORMAP_JET)

        h, w, c = bar_color.shape
        bytes_per_line = c * w
        q_img = QImage(
            bar_color.data, w, h, bytes_per_line, QImage.Format.Format_BGR888
        )

        pixmap = QPixmap.fromImage(q_img)
        self.bar_label.setPixmap(pixmap)

    def _on_type_changed(self, text):
        """Handle switching between Depth and Disparity units."""
        unit = "px" if text == "Disparity" else "m"
        self.title.setText(f"{text}\n({unit})")
        self.update_label_text()

    def update_range(self, min_val: float, max_val: float):
        """External method to update the sensor bounds."""
        self.min_dist = min_val
        self.max_dist = max_val
        self.update_label_text()
