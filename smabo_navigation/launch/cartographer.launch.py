"""Cartographer 2D SLAM: /scan + /odom -> /map, map -> odom.

Run by smabo_brain_ros/nav_manager in "mapping" mode (one child process, so it
stops both nodes together). Configuration: config/cartographer_2d.lua.

  ros2 launch smabo_navigation cartographer.launch.py [resolution:=0.05]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("smabo_navigation"), "config")
    use_sim_time = {"use_sim_time": False}
    return LaunchDescription([
        DeclareLaunchArgument("resolution", default_value="0.05"),
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[use_sim_time],
            arguments=["-configuration_directory", config_dir,
                       "-configuration_basename", "cartographer_2d.lua"],
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node",
            output="screen",
            parameters=[use_sim_time],
            arguments=["-resolution", LaunchConfiguration("resolution"),
                       "-publish_period_sec", "1.0"],
        ),
    ])
