import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launches the astroviz dashboard with the realsense-ros integration.

    If `use_mock_camera` is true, the mock camera node is run instead for testing."""

    use_mock_camera = LaunchConfiguration("use_mock_camera")

    declare_use_mock_camera = DeclareLaunchArgument(
        "use_mock_camera",
        default_value="false",
        description="If true, runs the mock camera node instead of the real integration",
    )

    # Attempt to find the realsense2 launch file.
    realsense_launch_dir = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch"
    )

    return LaunchDescription(
        [
            declare_use_mock_camera,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(realsense_launch_dir, "rs_launch.py")
                ),
                condition=UnlessCondition(use_mock_camera),
                launch_arguments={
                    # Profiles and Hardware.
                    "depth_module.depth_profile": "848x480x30",
                    "rgb_camera.color_profile": "848x480x30",
                    # Hardware/Software enables.
                    "align_depth.enable": "true",
                    "pointcloud.enable": "true",
                    "enable_infra1": "true",
                    "enable_infra2": "true",
                    # Post-processing Filters.
                    "decimation_filter.enable": "true",
                    "decimation_filter.filter_magnitude": "1",
                    "hdr_merge.enable": "true",
                    "spatial_filter.enable": "true",
                    "temporal_filter.enable": "true",
                    "hole_filling_filter.enable": "true",
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
