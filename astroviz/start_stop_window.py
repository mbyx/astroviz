import sys

import rclpy
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont, QColor 
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QPushButton

from rclpy.node import Node
from std_msgs.msg import Bool

from astroviz.utils.window_style import DarkStyle

class StartStopWindow(QWidget):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.car_running = False
        
        self.setWindowTitle("Car Control")
        self.resize(400, 300)
        
        
        # Subscrie to car status topics
        self.status_sub = self.node.create_subscription(Bool, '/car/status', self._status_callback, 10)
        self._init_ui()
        
        #Timer to update the UI from ROS callbacks
        self.timer = QTimer()
        self.timer.timeout.connect(self._update_ui)
        self.timer.start(100)  # evry 100 ms
        
    def _status_callback(self, msg):
        self.car_running = msg.data
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
        
        # Status
        self.status_label = QLabel("Car Status: Stopped")
        font = QFont("Arial", 20, QFont.Weight.Bold)
        self.status_label.setFont(font)
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.status_label)
        
        # Start Btn
        self.start_btn = QPushButton("Start Car")
        self.start_btn.setFont(QFont("Arial", 14))
        self.start_btn.setStyleSheet("background-color: green; color: white;")
        self.start_btn.clicked.connect(self._start_car)
        layout.addWidget(self.start_btn)
        
        # Stop Btn
        self.stop_btn = QPushButton("Stop Car")
        self.stop_btn.setFont(QFont("Arial", 14))
        self.stop_btn.setStyleSheet("background-color: red; color: white;")
        self.stop_btn.clicked.connect(self._stop_car)
        layout.addWidget(self.stop_btn)
        
        self.setLayout(layout)
        
    def _update_ui(self):
        if self.car_running:
            self.status_label.setText("Car Status: Running")
            self.status_label.setStyleSheet("color: green;")
        else:
            self.status_label.setText("Car Status: Stopped")
            self.status_label.setStyleSheet("color: red;")        
            
    def _start_car(self):
        msg = Bool(data= True)
        # self.start_pub.publish(msg)    # publish to car control topic
        self.car_running = True  # update when start btn clicked
        self.node.get_logger().info("Start command sent")
    
    def _stop_car(self):
        msg = Bool(data= False)
        # self.start_pub.publish(msg)    # publish to car control topic
        self.car_running = False  # update when stop btn clicked
        self.node.get_logger().info("Stop command sent")
        
def main():
    rclpy.init()
    app = QApplication(sys.argv)
    
    if DarkStyle:
        DarkStyle(app)
    
    node = rclpy.create_node("start_stop_node")
    win = StartStopWindow(node)
    win.show()
    exit_code = app.exec()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
            
            
        
        