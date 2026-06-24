"""Publish the smabo robot_description (+ optional joint_state_publisher/rviz).

Other bringup launches include this to get TF for the static links. Joint angles
for the servo joints normally come from /joint_states (ESP32 via rosbridge); set
``use_jsp_gui:=true`` to drive them manually when running standalone.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    xacro_path = PathJoinSubstitution(
        [FindPackageShare("smabo_description"), "urdf", "smabo.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(
            [FindExecutable(name="xacro"), " ", xacro_path]
        )
    }
    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("smabo_description"), "rviz", "smabo.rviz"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_jsp_gui", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_jsp_gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_cfg],
            condition=IfCondition(use_rviz),
        ),
    ])
