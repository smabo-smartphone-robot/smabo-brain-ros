"""Launch MoveIt2 move_group for the smabo arm.

Builds the MoveIt config from smabo_description's xacro + this package's config/
and starts move_group. Execution goes out as a FollowJointTrajectory action to
smabo_brain_ros/servo_trajectory_bridge (see config/moveit_controllers.yaml).

The robot configuration is picked with the same args as smabo.urdf.xacro and
passed to both the URDF and the SRDF xacro: ``arm`` / ``hand`` / ``breadboard``
(true|false) and ``mount`` (none|tt_motor|neck).

Set ``use_rsp:=true`` to also start robot_state_publisher (standalone use); the
top-level bringup launch starts it once for the whole system instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

MODEL_ARGS = {"arm": "true", "hand": "false", "breadboard": "false", "mount": "none"}


def _launch_setup(context):
    mappings = {k: LaunchConfiguration(k).perform(context) for k in MODEL_ARGS}
    description_xacro = os.path.join(
        get_package_share_directory("smabo_description"), "urdf", "smabo.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("smabo", package_name="smabo_moveit_config")
        .robot_description(file_path=description_xacro, mappings=mappings)
        .robot_description_semantic(file_path="config/smabo.srdf.xacro", mappings=mappings)
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"publish_robot_description_semantic": True},
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rsp")),
            parameters=[moveit_config.robot_description],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rsp", default_value="false"),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in MODEL_ARGS.items()],
        OpaqueFunction(function=_launch_setup),
    ])
