from astroviz.camera_window import CameraViewer
from astroviz.gps_map_window import GPSMapWindow
from astroviz.grid_map_window import GridMapViewer
from astroviz.imu_window import MainWindow as IMUWindow
from astroviz.lidar_window import LiDARViewer
from astroviz.motor_state_window import MotorTableViewer
from astroviz.orthogonal_window import OrthogonalViewer
from astroviz.plot_window import GraphViewer
from astroviz.record_manager_window import RecordManagerViewer
from astroviz.robot_state_window import RobotStateViewer
from astroviz.teleoperation_window import TeleoperationViewer
from astroviz.test_window import TestWindow
from astroviz.start_stop_window import StartStopWindow
from astroviz.logs_window import LogsWindow

VIEW_TYPES = {
    "GPS Map": GPSMapWindow,
    "Camera": CameraViewer,
    "IMU": IMUWindow,
    "LiDAR": LiDARViewer,
    "Teleoperation": TeleoperationViewer,
    "System Health": GraphViewer,
    "GridMap": GridMapViewer,
    "Orthogonal": OrthogonalViewer,
    "Motor State": MotorTableViewer,
    "Robot State": RobotStateViewer,
    "Record Manager": RecordManagerViewer,
    "Test Window": TestWindow,
    "Start/Stop": StartStopWindow,
    "Logs": LogsWindow,
}
