"""Nav2 with AMCL localisation against a saved/placeholder map.

Composes nav2_bringup's localization_launch.py (map_server + AMCL) and
navigation_launch.py (planner/controller/bt/behaviors) with smabo's params.
Assumes odom -> base_link (smabo_brain_ros/odom_node) and /scan are present.

Use slam.launch.py instead when you still need to build the map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("smabo_navigation")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file", default_value=os.path.join(pkg, "config", "nav2_params.yaml")
        ),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(pkg, "maps", "empty_map.yaml")
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("nav2_bringup"),
                                      "launch", "localization_launch.py"])
            ),
            launch_arguments={
                "map": map_yaml,
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            }.items(),
        ),
    ])
