import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launches the astroviz dashboard.

    If `use_mock_camera` is true, the mock camera node is run for testing.
    If `use_rs_camera` is true, the realsense camera node is run as well."""

    use_mock_camera = LaunchConfiguration("use_mock_camera")
    use_rs_camera = LaunchConfiguration("use_rs_camera")

    declare_use_mock_camera = DeclareLaunchArgument(
        "use_mock_camera",
        default_value="false",
        description="If true, runs the mock camera node.",
    )

    declare_use_realsense_camera = DeclareLaunchArgument(
        "use_rs_camera",
        default_value="false",
        description="If true, runs the realsense camera node.",
    )

    # Attempt to find the realsense2 launch file.
    realsense_launch_dir = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch"
    )

    return LaunchDescription(
        [
            declare_use_mock_camera,
            declare_use_realsense_camera,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_launch_dir, "rs_launch.py")
                ),
                condition=IfCondition(use_rs_camera),
                launch_arguments={
                    # Profiles and Hardware.
                    "depth_module.depth_profile": "848x480x30",
                    "rgb_camera.color_profile": "848x480x30",
                    "depth_module.infra_profile": "848x480x30",
                    # Hardware/Software enables.
                    "align_depth.enable": "true",
                    "enable_infra1": "true",
                    "enable_infra2": "true",
                    "enable_depth": "true",

                    "pointcloud.enable": "true",
                    "pointcloud.stream_filter": "1",
                    "pointcloud.stream_index_filter": "1",
                    # Post-processing Filters.
                    "decimation_filter.enable": "true",
                    "decimation_filter.filter_magnitude": "2",
                    "hdr_merge.enable": "true",
                    "spatial_filter.enable": "true",
                    "temporal_filter.enable": "true",
                    "hole_filling_filter.enable": "true",
                    "diagnostics_period": "1.0",
                }.items(),
            ),
            Node(
                package="astroviz",
                executable="dummy_camera_publisher",
                name="dummy_camera_publisher",
                condition=IfCondition(use_mock_camera),
                output="screen",
            ),
            Node(
                package="astroviz",
                executable="dashboard_viewer",
                name="dashboard_viewer",
                output="screen",
            ),
        ]
    )
