import sys

import rclpy
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget, QTextEdit

from rclpy.node import Node
from std_msgs.msg import String

from astroviz.utils.window_style import DarkStyle

class LogsWindow(QWidget):
    def __init__(self, node:Node):
        super().__init__()
        self.node = node
        self.logs = []
        
        self.setWindowTitle("System logs")
        self.resize(600, 400)
        
        # Subscribe to log topics
        self.log_sub = self.node.create_subscription(String, '/logs', self._log_callback, 10)
        
        # Dummy Nodes
        self.logs = [
            "[INFO] System started",
            "[INFO] Initializing sensors...",
            "[INFO] Camera initialized successfully",
            "[WARNING] LiDAR temperature high",
            "[INFO] Motor controller online",
            "[ERROR] GPS signal weak",
            "[INFO] Navigation ready",
            "[INFO] Teleoperation enabled",
        ]
        
        self._init_ui()
        
    def _log_callback(self, msg):
        self.logs.append(msg.data)
        if len(self.logs) > 100:  # number of logs to keep
            self.logs.pop(0)
        self._update_text_area()
        
    def _init_ui(self):
        layout = QVBoxLayout(self)
        
        # Title 
        title = QLabel("System Logs")
        font = QFont("Arial", 20, QFont.Weight.Bold)
        title.setFont(font)
        layout.addWidget(title)
        
        # Text area to display logs
        self.text_area = QTextEdit()
        self.text_area.setReadOnly(True)
        self.text_area.setFont(QFont("Courier", 10))
        layout.addWidget(self.text_area)
        
        self.setLayout(layout)
        
        self._update_text_area()    # display initial logs
        
    def _update_text_area(self):
        self.text_area.setText("\n".join(self.logs))  # Display all logs
        # Auto-scroll to bottom
        self.text_area.verticalScrollBar().setValue(self.text_area.verticalScrollBar().maximum())
        
def main():
    rclpy.init()
    app = QApplication(sys.argv)
    
    if DarkStyle:
        DarkStyle(app)
        
    node = rclpy.create_node("logs_node")
    win = LogsWindow(node)
    win.show()
    
    exit_code = app.exec()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)
    
if __name__ == "__main__":
    main()