"""SLAM (slam_toolbox) + Nav2, for building a map while driving smabo.

slam_toolbox publishes map -> odom from the LD06 /scan; Nav2 navigation servers
plan/control on top. Save the map with the /slam_toolbox/save_map service, then
switch to nav2.launch.py (AMCL) for repeated runs.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("smabo_navigation")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    params_file = LaunchConfiguration("params_file")
    slam_params = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(pkg, "config", "nav2_params.yaml")
        ),
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=os.path.join(pkg, "config", "slam_toolbox.yaml"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": "true",
            }.items(),
        ),
    ])
