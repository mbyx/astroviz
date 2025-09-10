import argparse
import os
import sys
from PyQt6.QtWidgets import (
    QApplication,
    QLabel,
    QWidget,
    QGridLayout,
    QSizePolicy,
)
from PyQt6.QtCore import Qt, pyqtSignal, QObject
from PyQt6.QtGui import QImage, QPixmap, QIcon, QPainter, QPen, QColor
from threading import Thread

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


class GStreamerPipeline:
    def __init__(self, pipeline_str, callback):
        import gi

        gi.require_version("Gst", "1.0")
        from gi.repository import Gst, GLib

        self.Gst = Gst
        self.GLib = GLib
        self.Gst.init(None)
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("appsink")
        self.appsink.connect("new-sample", self.on_new_sample)
        self.callback = callback

    def start(self):
        self.loop = self.GLib.MainLoop()
        self.pipeline.set_state(self.Gst.State.PLAYING)
        self.thread = Thread(target=self.loop.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.pipeline.set_state(self.Gst.State.NULL)
        self.loop.quit()

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()
        caps = sample.get_caps()
        width = caps.get_structure(0).get_value("width")
        height = caps.get_structure(0).get_value("height")
        stride = 4 * width  # BGRx format

        success, map_info = buf.map(self.Gst.MapFlags.READ)
        if success:
            # Make an owned copy of the bytes BEFORE unmapping
            frame_bytes = bytes(map_info.data)
            buf.unmap(map_info)

            # Call the callback directly (no extra thread needed)
            self.callback(width, height, stride, frame_bytes)

        return self.Gst.FlowReturn.OK


class WebcamDisplay(QObject):
    image_updated = pyqtSignal(QImage)

    def __init__(self, label):
        super().__init__()
        self.label = label
        self._last_image = None
        self.image_updated.connect(self._update)

    def update_image(self, w, h, stride, data: bytes):
        # Construct from bytes, then .copy() to detach (QImage owns its own buffer)
        img = QImage(
            data, w, h, stride, QImage.Format.Format_RGB32
        ).copy()

        painter = QPainter(img)
        pen = QPen(QColor("green"))
        pen.setWidth(8)
        pen.setStyle(Qt.PenStyle.DashLine)
        painter.setPen(pen)
        cx, cy = w // 2, h // 2
        bottom_half_d = int(0.389 * w)
        top_half_d = int(0.157 * w)
        center_offs = -int(0.0148 * w)
        painter.drawLine(
            center_offs + cx - bottom_half_d, h, cx - top_half_d, cy
        )
        painter.drawLine(
            center_offs + cx + bottom_half_d, h, cx + top_half_d, cy
        )
        painter.end()

        self.image_updated.emit(img)

    def _render(self):
        if self._last_image is None:
            return
        target_size = self.label.size()
        if target_size.width() <= 0 or target_size.height() <= 0:
            return
        scaled = self._last_image.scaled(
            target_size,
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation,
        )
        self.label.setPixmap(QPixmap.fromImage(scaled))

    def _update(self, img):
        self._last_image = img
        self._render()


class GstreamerWindow(QWidget):
    def __init__(self, port=5000, width=1280, height=720, flip=1):
        super().__init__()
        self.setWindowTitle("Front Camera")
        self.setWindowIcon(
            QIcon(os.path.join(ICONS_DIR, "astroviz_icon.png"))
        )
        main_layout = QGridLayout(self)
        self.port = port
        self.width = width
        self.height = height
        self.flip = flip

        # Video display
        self.camera_label = QLabel(self)
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding
        )
        if flip == 0:
            self.camera_label.setMinimumSize(480, 280)
        else:
            self.camera_label.setMinimumSize(280, 480)
        main_layout.addWidget(self.camera_label, 0, 0)

        # Construct pipeline
        pipeline_str = (
            f"udpsrc port={self.port} ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! "
            f"videoconvert ! videoscale ! video/x-raw,format=BGRx,width={self.width},height={self.height} ! "
            f"videoflip method={self.flip} ! appsink name=appsink emit-signals=true sync=false max-buffers=1 drop=true "
        )
        self.display = WebcamDisplay(self.camera_label)
        self.pipeline = GStreamerPipeline(
            pipeline_str, self.display.update_image
        )
        self.pipeline.start()

    def closeEvent(self, event):
        self.pipeline.stop()
        super().closeEvent(event)


def main(args=None):
    from astroviz.utils.window_style import DarkStyle

    parser = argparse.ArgumentParser(description="Gstreamer Viewer")
    parser.add_argument(
        "--port", type=int, default=5000, help="UDP port to listen on"
    )
    parser.add_argument(
        "--width", type=int, default=960, help="Video width"
    )
    parser.add_argument(
        "--height", type=int, default=540, help="Video height"
    )
    parsed_args = parser.parse_args(args)

    app = QApplication(sys.argv)
    DarkStyle(app)
    window = GstreamerWindow(
        port=parsed_args.port,
        width=parsed_args.width,
        height=parsed_args.height,
        flip=1,
    )
    window.show()
    app.exec()


if __name__ == "__main__":
    main()

"""
Usage:

The image is automatically rotated to match the webcam orientation on Shelfy.
Forward lines are drawn to see where the robot will be moving forward.
```
ros2 run astroviz gstreamer_shelfy_viewer \
--port 5000 --width 1280 --height 720
```
"""
