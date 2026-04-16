import sys
import random
import rclpy
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QStandardItem, QStandardItemModel
from PyQt6.QtWidgets import (
    QApplication, 
    QLabel, 
    QVBoxLayout, 
    QWidget, 
    QTextEdit, 
    QComboBox, 
    QHBoxLayout,
)

from rclpy.node import Node
from rcl_interfaces.msg import Log 
from std_msgs.msg import String

from astroviz.utils.window_style import DarkStyle

def get_log_topic_type(node: Node, topic_name: str):
    mapping = {
        "std_msgs/msg/String": String,
        "rcl_interfaces/msg/Log": Log,
    }
    topics = node.get_topic_names_and_types()
    for name, types in topics:
        if name == topic_name:
            for t in types:
                if t in mapping:
                    return mapping[t]
    return None

class CheckableComboBox(QComboBox):
    """Custom ComboBox that shows a comma-separated list of checked items."""
    def __init__(self):
        super().__init__()
        self.setModel(QStandardItemModel(self))
        self.view().pressed.connect(self.handle_item_pressed)
        self.setEditable(True)
        self.lineEdit().setReadOnly(True)
        self.lineEdit().setPlaceholderText("Select topics...")

    def handle_item_pressed(self, index):
        item = self.model().itemFromIndex(index)
        if item.checkState() == Qt.CheckState.Checked:
            item.setCheckState(Qt.CheckState.Unchecked)
        else:
            item.setCheckState(Qt.CheckState.Checked)
        self._update_display_text()

    def _update_display_text(self):
        selected = self.checked_items()
        self.lineEdit().setText(", ".join(selected))

    def checked_items(self):
        checked = []
        for i in range(self.count()):
            item = self.model().item(i)
            if item and item.checkState() == Qt.CheckState.Checked:
                checked.append(item.text())
        return checked

    def addItem(self, text):
        item = QStandardItem(text)
        item.setCheckable(True)
        item.setCheckState(Qt.CheckState.Unchecked)
        self.model().appendRow(item)

class LogsWindow(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.logs = []
        self.subscriptions = {} 
        
        self.setWindowTitle("Multi-Topic System Logs")
        self.resize(850, 550)
        
        self._init_ui()

        # Timers
        self.topic_timer = QTimer()
        self.topic_timer.timeout.connect(self.refresh_topics_list)
        self.topic_timer.start(2000)

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.ros_timer.start(30)

        self.dummy_timer = QTimer()
        self.dummy_timer.timeout.connect(self.add_dummy_logs)
        self.dummy_timer.start(4000)

    def _init_ui(self):
        layout = QVBoxLayout(self)
        
        header = QHBoxLayout()
        title = QLabel("System Logs")
        title.setFont(QFont("Arial", 16, QFont.Weight.Bold))
        
        self.topic_selector = CheckableComboBox()
        self.topic_selector.setFixedWidth(400)
        # Connect the data change to our subscription logic
        self.topic_selector.model().dataChanged.connect(self.sync_subscriptions)
        
        header.addWidget(title)
        header.addStretch()
        header.addWidget(self.topic_selector)
        layout.addLayout(header)
        
        self.text_area = QTextEdit()
        self.text_area.setReadOnly(True)
        self.text_area.setFont(QFont("Courier New", 10))
        self.text_area.setStyleSheet("background-color: #121212; color: #00FF00; border: 1px solid #333;")
        layout.addWidget(self.text_area)

    def refresh_topics_list(self):
        all_topics = self.node.get_topic_names_and_types()
        log_topics = [
            name for name, types in all_topics
            if ("std_msgs/msg/String" in types or "rcl_interfaces/msg/Log" in types)
        ]
        
        existing_items = [self.topic_selector.itemText(i) for i in range(self.topic_selector.count())]
        for topic in log_topics:
            if topic not in existing_items:
                self.topic_selector.addItem(topic)

    def sync_subscriptions(self):
        selected_topics = self.topic_selector.checked_items()
        
        # Unsubscribe from deselected
        for topic in list(self.subscriptions.keys()):
            if topic not in selected_topics:
                self.node.destroy_subscription(self.subscriptions[topic])
                del self.subscriptions[topic]
                self.add_to_log(f"--- [DISCONNECTED]: {topic} ---")

        # Subscribe to newly selected
        for topic in selected_topics:
            if topic not in self.subscriptions:
                topic_type = get_log_topic_type(self.node, topic)
                if topic_type:
                    # Using a default argument 't=topic' to capture the current value in the loop
                    sub = self.node.create_subscription(
                        topic_type, 
                        topic, 
                        lambda msg, t=topic: self._log_callback(msg, t), 
                        10
                    )
                    self.subscriptions[topic] = sub
                    self.add_to_log(f"--- [SUBSCRIBED]: {topic} ---")

    def _log_callback(self, msg, topic_name):
        if isinstance(msg, Log):
            level_map = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
            level = level_map.get(msg.level, "UNKNOWN")
            text = f"[{level}] [{topic_name}]: {msg.msg}"
        else:
            text = f"[{topic_name}]: {msg.data}"
        self.add_to_log(text)

    def add_dummy_logs(self):
        dummies = [
            "IMU Data Stream: Stability 98%",
            "Motor PWM: 1450us",
            "Lidar Scanning: 360deg coverage complete",
            "Camera Node: Frame dropped",
            "Global Planner: Calculating new route..."
        ]
        self.add_to_log(f"[DUMMY] {random.choice(dummies)}")

    def add_to_log(self, text):
        self.logs.append(text)
        if len(self.logs) > 500:
            self.logs.pop(0)
        self._update_text_area()

    def _update_text_area(self):
        self.text_area.setText("\n".join(self.logs))
        self.text_area.verticalScrollBar().setValue(
            self.text_area.verticalScrollBar().maximum()
        )

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    DarkStyle(app) if DarkStyle else None
        
    node = rclpy.create_node("log_aggregator_node")
    win = LogsWindow(node)
    win.show()
    
    app.exec()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# import sys

# import rclpy
# from PyQt6.QtCore import Qt, QTimer
# from PyQt6.QtGui import QFont
# from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QTextEdit

# from rclpy.node import Node
# from std_msgs.msg import String

# from astroviz.utils.window_style import DarkStyle

# class LogsWindow(QWidget):
#     def __init__(self, node:Node):
#         super().__init__()
#         self.node = node
#         self.logs = []
        
#         self.setWindowTitle("System logs")
#         self.resize(600, 400)
        
#         # Subscribe to log topics
#         self.log_sub = self.node.create_subscription(String, '/logs', self._log_callback, 10)
        
#         # Dummy Nodes
#         self.logs = [
#             "[INFO] System started",
#             "[INFO] Initializing sensors...",
#             "[INFO] Camera initialized successfully",
#             "[WARNING] LiDAR temperature high",
#             "[INFO] Motor controller online",
#             "[ERROR] GPS signal weak",
#             "[INFO] Navigation ready",
#             "[INFO] Teleoperation enabled",
#         ]
        
#         self._init_ui()
        
#     def _log_callback(self, msg):
#         self.logs.append(msg.data)
#         if len(self.logs) > 100:  # number of logs to keep
#             self.logs.pop(0)
#         self._update_text_area()
        
#     def _init_ui(self):
#         layout = QVBoxLayout(self)
        
#         # Title 
#         title = QLabel("System Logs")
#         font = QFont("Arial", 20, QFont.Weight.Bold)
#         title.setFont(font)
#         layout.addWidget(title)
        
#         # Text area to display logs
#         self.text_area = QTextEdit()
#         self.text_area.setReadOnly(True)
#         self.text_area.setFont(QFont("Courier", 10))
#         layout.addWidget(self.text_area)
        
#         self.setLayout(layout)
        
#         self._update_text_area()    # display initial logs
        
#     def _update_text_area(self):
#         self.text_area.setText("\n".join(self.logs))  # Display all logs
#         # Auto-scroll to bottom
#         self.text_area.verticalScrollBar().setValue(self.text_area.verticalScrollBar().maximum())
        
# def main():
#     rclpy.init()
#     app = QApplication(sys.argv)
    
#     if DarkStyle:
#         DarkStyle(app)
        
#     node = rclpy.create_node("logs_node")
#     win = LogsWindow(node)
#     win.show()
    
#     exit_code = app.exec()
#     node.destroy_node()
#     rclpy.shutdown()
#     sys.exit(exit_code)
    
# if __name__ == "__main__":
#     main()