import sys
import rclpy
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import (
    QApplication, 
    QLabel, 
    QVBoxLayout, 
    QWidget, 
    QPushButton, 
    QComboBox, 
    QHBoxLayout
)

from rclpy.node import Node
from std_msgs.msg import Bool
from astroviz.utils.window_style import DarkStyle

class StartStopWindow(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.car_running = False
        self.status_sub = None
        self.control_pub = None
        
        # List of dummy topics to inject
        self.dummy_topic_names = ["/dummy/car_1/status", "/dummy/car_2/status", "/simulation/robot/power"]
        
        self.setWindowTitle("Car Control")
        self.resize(450, 350)
        
        self._init_ui()
        
        # Timer to refresh topics (Real + Dummy)
        self.topic_timer = QTimer()
        self.topic_timer.timeout.connect(self.update_bool_topics)
        self.topic_timer.start(1000)

        # Timer to spin ROS2
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.ros_timer.start(30)
        
        # Timer to update UI
        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self._update_ui_state)
        self.ui_timer.start(100)

    def _init_ui(self):
        layout = QVBoxLayout(self)
        
        header = QHBoxLayout()
        header.addWidget(QLabel("Control Topic:"))
        self.combo = QComboBox()
        self.combo.currentTextChanged.connect(self.change_bool_topic)
        header.addWidget(self.combo)
        layout.addLayout(header)

        self.status_label = QLabel("Status: Unknown")
        self.status_label.setFont(QFont("Arial", 22, QFont.Weight.Bold))
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.status_label)
        
        self.start_btn = QPushButton("START CAR")
        self.start_btn.setMinimumHeight(60)
        self.start_btn.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        self.start_btn.setStyleSheet("background-color: #2e7d32; color: white; border-radius: 10px;")
        self.start_btn.clicked.connect(self._start_car)
        layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("STOP CAR")
        self.stop_btn.setMinimumHeight(60)
        self.stop_btn.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        self.stop_btn.setStyleSheet("background-color: #c62828; color: white; border-radius: 10px;")
        self.stop_btn.clicked.connect(self._stop_car)
        layout.addWidget(self.stop_btn)

    def update_bool_topics(self):
        current = self.combo.currentText()
        
        # Get REAL topics from ROS graph
        all_topics = self.node.get_topic_names_and_types()
        real_bool_topics = [name for name, types in all_topics if "std_msgs/msg/Bool" in types]
        
        # Merge REAL topics with our DUMMY topics
        items = ["---"] + real_bool_topics + self.dummy_topic_names
        
        # Deduplicate while keeping order
        items = list(dict.fromkeys(items))

        if [self.combo.itemText(i) for i in range(self.combo.count())] == items:
            return

        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItems(items)
        self.combo.setCurrentText(current if current in items else "---")
        self.combo.blockSignals(False)

    def change_bool_topic(self, topic_name):
        # Reset state
        self.car_running = False
        
        if self.status_sub:
            self.node.destroy_subscription(self.status_sub)
        if self.control_pub:
            self.node.destroy_publisher(self.control_pub)
        
        if topic_name == "---":
            self.status_sub = None
            self.control_pub = None
            return

        # Check if it's a dummy topic
        if topic_name in self.dummy_topic_names:
            self.node.get_logger().info(f"Simulating Dummy Topic: {topic_name}")
            self.status_sub = None
            self.control_pub = "DUMMY_MODE" # Placeholder
        else:
            # Real ROS2 Logic
            self.status_sub = self.node.create_subscription(Bool, topic_name, self._status_callback, 10)
            self.control_pub = self.node.create_publisher(Bool, topic_name, 10)
            self.node.get_logger().info(f"Connected to Real Topic: {topic_name}")

    def _status_callback(self, msg):
        self.car_running = msg.data
        
    def _update_ui_state(self):
        if self.combo.currentText() == "---":
            self.status_label.setText("Select a Topic")
            self.status_label.setStyleSheet("color: gray;")
        elif self.car_running:
            self.status_label.setText("CAR RUNNING")
            self.status_label.setStyleSheet("color: #4caf50;")
        else:
            self.status_label.setText("CAR STOPPED")
            self.status_label.setStyleSheet("color: #f44336;")
            
    def _start_car(self):
        self.car_running = True 
        if self.control_pub and self.control_pub != "DUMMY_MODE":
            self.control_pub.publish(Bool(data=True))
        self.node.get_logger().info(f"Command: START on {self.combo.currentText()}")
    
    def _stop_car(self):
        self.car_running = False 
        if self.control_pub and self.control_pub != "DUMMY_MODE":
            self.control_pub.publish(Bool(data=False))
        self.node.get_logger().info(f"Command: STOP on {self.combo.currentText()}")

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    if DarkStyle:
        DarkStyle(app)
    
    node = rclpy.create_node("car_control_gui")
    win = StartStopWindow(node)
    win.show()
    
    app.exec()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()