"""Bring up the whole smabo ROS 2 graph.

Composes, in one place:
  - robot_state_publisher (smabo_description)         → static TF + URDF
  - odom_node (smabo_brain_ros)                       → /wheel_vel -> /odom + tf
  - prefix relays (smabo_brain_ros)                   → /web,/esp32,/app -> canonical
  - rosbridge_suite                                   → WS<->ROS2 for web/app/esp32
  - Nav2 navigation servers + nav_manager (smabo_brain_ros)
                                                      → /odom,/scan -> /cmd_vel; nav_manager
                                                        switches SLAM (build a map) / AMCL (saved
                                                        map) at runtime from smabo-web's Nav tab
  - MoveIt2 move_group + servo_trajectory_bridge      → arm planning -> /servo/command
  - robot_model_server (smabo_brain_ros)              → URDF / SRDF / meshes for smabo-web (/smabo/get_file)
  - vision (webrtc_camera + image_processor + policies)→ WebRTC video -> detections -> behaviours
  - speech_recognizer                                 → /speech/audio (STT) -> /speech/recognized

Arguments:
  sim          (false) simulate the arm: servo_trajectory_bridge publishes
               /joint_states from commands (no ESP32 needed for MoveIt).
  slam         (false) start in mapping mode (slam_toolbox) instead of AMCL+map.
               Either way, the Nav tab switches between building a map, saving
               it and navigating on a saved one without restarting.
  map          map to start AMCL on: a yaml path or the name of a saved map
               (default: smabo_navigation/maps/empty_map.yaml). For smabo-sim
               maps/sim_room.yaml is its world exported as a map.
  maps_dir     where maps are saved / listed (default: $SMABO_MAPS_DIR, else
               ~/smabo-maps; docker-compose mounts the host's ~/smabo-maps).
  use_nav      (true)  start Nav2.
  use_moveit   (true)  start MoveIt2 + the servo trajectory bridge.
  use_vision   (true)  start the vision pipeline (webrtc_camera ingests the
               smabo-app WebRTC video as /camera/image_raw, then detection).
  use_speech   (true)  start speech_recognizer (/speech/audio -> STT ->
               /speech/recognized).
  use_rosbridge(true)  start rosbridge_suite.
  rosbridge_port (9090).
  arm / hand / breadboard (true|false), mount (none|tt_motor|neck)
               robot configuration, passed to smabo.urdf.xacro and the SRDF.
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
    use_speech = LaunchConfiguration("use_speech")
    use_rosbridge = LaunchConfiguration("use_rosbridge")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    model_args = {k: LaunchConfiguration(k) for k in ("arm", "hand", "breadboard", "mount")}

    brain_ros = get_package_share_directory("smabo_brain_ros")
    description = get_package_share_directory("smabo_description")
    navigation = get_package_share_directory("smabo_navigation")
    moveit = get_package_share_directory("smabo_moveit_config")
    bringup = get_package_share_directory("smabo_bringup")
    nav2_params = os.path.join(navigation, "config", "nav2_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("use_nav", default_value="true"),
        DeclareLaunchArgument("use_moveit", default_value="true"),
        DeclareLaunchArgument("use_vision", default_value="true"),
        DeclareLaunchArgument("use_speech", default_value="true"),
        DeclareLaunchArgument("use_rosbridge", default_value="true"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument(
            "map", default_value=os.path.join(navigation, "maps", "empty_map.yaml")),
        DeclareLaunchArgument("maps_dir", default_value=""),
        DeclareLaunchArgument("arm", default_value="true"),
        DeclareLaunchArgument("hand", default_value="false"),
        DeclareLaunchArgument("breadboard", default_value="false"),
        DeclareLaunchArgument("mount", default_value="none"),

        # --- robot model / TF ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description, "launch", "description.launch.py")
            ),
            launch_arguments=model_args.items(),
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

        # --- Nav2 navigation servers (always) + nav_manager, which runs either
        #     slam_toolbox (mapping) or map_server+AMCL (navigation) and switches
        #     between them on request from smabo-web ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("nav2_bringup"), "launch",
                             "navigation_launch.py")
            ),
            launch_arguments={"params_file": nav2_params, "autostart": "true"}.items(),
            condition=IfCondition(use_nav),
        ),
        Node(
            package="smabo_brain_ros",
            executable="nav_manager",
            name="nav_manager",
            output="screen",
            parameters=[{
                "initial_mode": PythonExpression(["'mapping' if '", slam, "' == 'true' else 'navigation'"]),
                "initial_map": LaunchConfiguration("map"),
                "maps_dir": LaunchConfiguration("maps_dir"),
                "nav2_params": nav2_params,
                "slam_params": os.path.join(navigation, "config", "slam_toolbox.yaml"),
            }],
            condition=IfCondition(use_nav),
        ),

        # --- MoveIt2 move_group ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit, "launch", "move_group.launch.py")
            ),
            launch_arguments=model_args.items(),
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

        # --- robot model server (URDF / SRDF / meshes over rosbridge for smabo-web) ---
        Node(
            package="smabo_brain_ros",
            executable="robot_model_server",
            name="robot_model_server",
            output="screen",
            condition=IfCondition(use_rosbridge),
        ),

        # --- vision: webrtc_camera (WebRTC video -> /camera/image_raw) + detection + policies ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(brain_ros, "launch", "vision.launch.py")
            ),
            condition=IfCondition(use_vision),
        ),

        # --- speech: /speech/audio (STT) -> /speech/recognized ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(brain_ros, "launch", "speech.launch.py")
            ),
            condition=IfCondition(use_speech),
        ),
    ])
