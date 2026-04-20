# Implemented Windows

This document lists all the GUI windows currently available in the `astroviz` package, including a brief description and representative image.

| Name           | Image                                                                 | Description                                                                 |
|----------------|------------------------------------------------------------------------|-----------------------------------------------------------------------------|
| Map Viewer     | ![Map Viewer](images/map_viewer.png)                                  | Displays real-time GPS position and supports waypoint interaction.          |
| LiDAR Viewer   | ![LiDAR Viewer](images/lidar_viewer.png)                              | 3D visualization of LiDAR point cloud data in real time.                    |
| IMU Viewer     | ![IMU Viewer](images/imu_viewer.png)                                  | Shows IMU angles and accelerations in a simplified visual representation.   |
| Camera Viewer  | ![Camera Viewer](images/camera_viewer.png) *(if exists)*              | Live feed of the robot's onboard camera for teleoperation purposes.         |
| Plot Viewer    | ![Plot Viewer](images/plot_viewer.png)                                 | Displays real-time data from various sensors and allows for data analysis.  |
| GridMap Viewer | ![GridMap Viewer](images/grid_map_viewer.png)                           | Displays a grid map of the environment, useful for navigation and planning. |
| Ortogonal Viewer | ![Ortogonal Viewer](images/orthogonal_viewer.png)                     | Displays orthogonal views of the robot and the environment for better spatial understanding. |
| Gstreamer Viewer | ![Gstreamer Viewer](images/gstreamer_viewer.png) *(if exists)* | Displays video streams from the robot's cameras using GStreamer.            |
| Robot State Viewer   | ![Robot State](images/robot_state_viewer.png)                                 | Displays the robot's state, including joint positions and visual meshes.    |
| Motor State Viewer | ![Motor State](images/motor_state_viewer.png)                                 | Displays the state of the robot's motors, including temperature and voltage. |
| Record Manager Viewer | ![Record Manager](images/record_manager_viewer.png)                         | Easy management for recording sessions |

# Topics Used
This lists all of the topics each window listens for.
- Camera: All topics with the `Image`/`CompressedImage` type.
- AudioPlayer: All topics with the `AudioStamped` type.
- GPSMap: All topics with the `NavSatFix` type.
- GridMap: All topics with the `OccupancyGrid` type. It also publishes waypoints with the `Path` type.
- IMU: All topics with the `Imu` type.
- Lidar: All topics with the `PointCloud2` type.
- PlotWindow: All topics with the `BatteryState` type.

