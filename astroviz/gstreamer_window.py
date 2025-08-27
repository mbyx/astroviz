import argparse
import sys
from PyQt6.QtWidgets import QApplication, QLabel, QWidget, QGridLayout
from PyQt6.QtCore import Qt, pyqtSignal, QObject
from PyQt6.QtGui import QImage, QPixmap
from threading import Thread
import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib


class GStreamerPipeline:
    def __init__(self, pipeline_str, callback):
        Gst.init(None)
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("appsink")
        self.appsink.connect("new-sample", self.on_new_sample)
        self.callback = callback

    def start(self):
        self.loop = GLib.MainLoop()
        self.pipeline.set_state(Gst.State.PLAYING)
        self.thread = Thread(target=self.loop.run, daemon=True)
        self.thread.start()

    def stop(self):
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()
        caps = sample.get_caps()
        width = caps.get_structure(0).get_value("width")
        height = caps.get_structure(0).get_value("height")
        stride = 4 * width  # BGRx format
        success, map_info = buf.map(Gst.MapFlags.READ)
        if success:
            data = map_info.data
            Thread(
                target=self.callback,
                args=(width, height, stride, data),
                daemon=True,
            ).start()
            buf.unmap(map_info)
        return Gst.FlowReturn.OK


class WebcamDisplay(QObject):
    image_updated = pyqtSignal(QImage)

    def __init__(self, label):
        super().__init__()
        self.label = label
        self.image_updated.connect(self._update)

    def update_image(self, w, h, stride, data):
        img = QImage(data, w, h, stride, QImage.Format.Format_RGB32)
        self.image_updated.emit(img)

    def _update(self, img):
        self.label.setPixmap(QPixmap.fromImage(img))


class GstreamerWindow(QWidget):
    def __init__(self, port=5000, width=1280, height=720, flip=0):
        super().__init__()
        self.setWindowTitle("Gstreamer Viewer")
        main_layout = QGridLayout(self)
        self.port = port
        self.width = width
        self.height = height
        self.flip = flip
        # Video display
        self.camera_label = QLabel(self)
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(self.camera_label, 0, 0)

        # Construct pipeline
        pipeline_str = (
            f"udpsrc port={self.port} ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! "
            f"videoconvert ! videoscale ! video/x-raw,format=BGRx,width={self.width},height={self.height} ! "
            f"videoflip method={self.flip} ! appsink name=appsink emit-signals=true async=false "
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
    parser = argparse.ArgumentParser(description="Gstreamer Viewer")
    parser.add_argument(
        "--port", type=int, default=5000, help="UDP port to listen on"
    )
    parser.add_argument("--width", type=int, default=960, help="Video width")
    parser.add_argument("--height", type=int, default=540, help="Video height")
    parser.add_argument(
        "--flip", type=int, default=0, help="videoflip method (0-8)"
    )
    parsed_args = parser.parse_args(args)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = GstreamerWindow(
        port=parsed_args.port,
        width=parsed_args.width,
        height=parsed_args.height,
        flip=parsed_args.flip,
    )
    window.show()
    app.exec()


if __name__ == "__main__":
    main()

"""
Usage:

```
ros2 run astroviz gstreamer_viewer \
--port 5000 --width 1280 --height 720 --flip 0
```

Available `videoflip` methods are:
- (0): none
- (1): clockwise
- (2): rotate-180
- (3): counterclockwise
- (4): horizontal-flip
- (5): vertical-flip
- (6): upper-left-diagonal
- (7): upper-right-diagonal
- (8): automatic
"""
