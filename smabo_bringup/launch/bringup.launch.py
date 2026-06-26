"""Bring up the whole smabo ROS 2 graph.

Composes, in one place:
  - robot_state_publisher (smabo_description)         → static TF + URDF
  - odom_node (smabo_brain_ros)                       → /wheel_vel -> /odom + tf
  - prefix relays (smabo_brain_ros)                   → /web,/esp32,/app -> canonical
  - rosbridge_suite                                   → WS<->ROS2 for web/app/esp32
  - Nav2 (+ AMCL or SLAM)                             → /odom,/scan -> /cmd_vel
  - MoveIt2 move_group + servo_trajectory_bridge      → arm planning -> /servo/command
  - vision (webrtc_camera + image_processor + policies)→ WebRTC video -> detections -> behaviours

Arguments:
  sim          (false) simulate the arm: servo_trajectory_bridge publishes
               /joint_states from commands (no ESP32 needed for MoveIt).
  slam         (false) build a map with slam_toolbox instead of AMCL+map.
  use_nav      (true)  start Nav2.
  use_moveit   (true)  start MoveIt2 + the servo trajectory bridge.
  use_vision   (true)  start the vision pipeline (webrtc_camera ingests the
               smabo-app WebRTC video as /camera/image_raw, then detection).
  use_rosbridge(true)  start rosbridge_suite.
  rosbridge_port (9090).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    slam = LaunchConfiguration("slam")
    use_nav = LaunchConfiguration("use_nav")
    use_moveit = LaunchConfiguration("use_moveit")
    use_vision = LaunchConfiguration("use_vision")
    use_rosbridge = LaunchConfiguration("use_rosbridge")
    rosbridge_port = LaunchConfiguration("rosbridge_port")

    brain_ros = get_package_share_directory("smabo_brain_ros")
    description = get_package_share_directory("smabo_description")
    navigation = get_package_share_directory("smabo_navigation")
    moveit = get_package_share_directory("smabo_moveit_config")
    bringup = get_package_share_directory("smabo_bringup")

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("use_nav", default_value="true"),
        DeclareLaunchArgument("use_moveit", default_value="true"),
        DeclareLaunchArgument("use_vision", default_value="true"),
        DeclareLaunchArgument("use_rosbridge", default_value="true"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),

        # --- robot model / TF ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description, "launch", "description.launch.py")
            ),
        ),

        # --- odometry: /wheel_vel -> /odom + tf ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(brain_ros, "launch", "odom.launch.py")
            ),
        ),

        # --- prefix relays: /web,/esp32,/app -> canonical ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(brain_ros, "launch", "relays.launch.py")
            ),
        ),

        # --- rosbridge: WS <-> ROS2 ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, "launch", "rosbridge.launch.py")
            ),
            launch_arguments={"port": rosbridge_port}.items(),
            condition=IfCondition(use_rosbridge),
        ),

        # --- Nav2: AMCL+map (default) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation, "launch", "nav2.launch.py")
            ),
            condition=IfCondition(PythonExpression(
                ["'", use_nav, "' == 'true' and '", slam, "' == 'false'"]
            )),
        ),

        # --- Nav2: SLAM (slam:=true) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(navigation, "launch", "slam.launch.py")
            ),
            condition=IfCondition(PythonExpression(
                ["'", use_nav, "' == 'true' and '", slam, "' == 'true'"]
            )),
        ),

        # --- MoveIt2 move_group ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit, "launch", "move_group.launch.py")
            ),
            condition=IfCondition(use_moveit),
        ),

        # --- servo trajectory bridge (MoveIt FollowJointTrajectory -> /servo/command) ---
        Node(
            package="smabo_brain_ros",
            executable="servo_trajectory_bridge",
            name="servo_trajectory_bridge",
            output="screen",
            parameters=[{"simulate": ParameterValue(sim, value_type=bool)}],
            condition=IfCondition(use_moveit),
        ),

        # --- vision: webrtc_camera (WebRTC video -> /camera/image_raw) + detection + policies ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(brain_ros, "launch", "vision.launch.py")
            ),
            condition=IfCondition(use_vision),
        ),
    ])
