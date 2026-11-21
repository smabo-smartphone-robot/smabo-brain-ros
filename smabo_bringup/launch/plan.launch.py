"""The motion-planning stack: MoveIt2 move_group + servo_trajectory_bridge.

Started and stopped by mode_manager (smabo-web's Motion Plan tab), which also
switches robot_state_publisher to the arm URDF first. Can be run by hand on top
of bringup.launch.py too.

Arguments:
  sim  (false) simulate the arm: servo_trajectory_bridge publishes
       /joint_states from commands (no ESP32 needed for MoveIt).
  arm / hand / breadboard (true|false), mount (none|tt_motor|dg01d_e|neck)
       robot configuration for the URDF and SRDF move_group reads
       (default: the arm alone).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

MODEL_ARGS = {"arm": "true", "hand": "false", "breadboard": "false", "mount": "none"}


def generate_launch_description():
    moveit = get_package_share_directory("smabo_moveit_config")
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in MODEL_ARGS.items()],

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(moveit, "launch", "move_group.launch.py")),
            launch_arguments={k: LaunchConfiguration(k) for k in MODEL_ARGS}.items(),
        ),
        # MoveIt FollowJointTrajectory -> /servo/command
        Node(
            package="smabo_brain_ros",
            executable="servo_trajectory_bridge",
            name="servo_trajectory_bridge",
            output="screen",
            parameters=[{"simulate": ParameterValue(LaunchConfiguration("sim"), value_type=bool)}],
        ),
    ])
