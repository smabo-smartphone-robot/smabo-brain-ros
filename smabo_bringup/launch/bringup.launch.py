"""Bring up the smabo ROS 2 graph.

Composes, in one place:
  - robot_state_publisher (smabo_description)         → static TF + URDF
  - odom_node (smabo_brain_ros)                       → /wheel_vel -> /odom + tf
  - prefix relays (smabo_brain_ros)                   → /web,/esp32,/app -> canonical
  - rosbridge_suite                                   → WS<->ROS2 for web/app/esp32
  - mode_manager (smabo_brain_ros)                    → starts / stops one of these on request
                                                        from smabo-web (Navigation / Motion Plan
                                                        tabs), switching the URDF to its robot:
      nav   nav.launch.py: Nav2 servers + nav_manager → /odom,/scan -> /cmd_vel; nav_manager
                                                        switches SLAM (build a map) / AMCL (saved
                                                        map) at runtime from smabo-web's Nav tab
      plan  plan.launch.py: MoveIt2 move_group +
            servo_trajectory_bridge                   → arm planning -> /servo/command
  - robot_model_server (smabo_brain_ros)              → URDF / SRDF / meshes for smabo-web (/smabo/get_file)
  - vision (webrtc_camera + image_processor + policies)→ WebRTC video -> detections -> behaviours
  - speech_recognizer                                 → /speech/audio (STT) -> /speech/recognized

Arguments:
  mode         (none)  start this mode right away: none | nav | plan. Either way
               smabo-web can start / stop / switch them later.
  sim          (false) plan mode: simulate the arm: servo_trajectory_bridge publishes
               /joint_states from commands (no ESP32 needed for MoveIt).
  slam         (false) nav mode: start in mapping mode (SLAM, Cartographer) even when a map
               is given. Either way, the Nav tab switches between building a map,
               saving it and navigating on a saved one without restarting.
  map          map to start AMCL on: a yaml path or the name of a saved map. Empty
               (default): nav mode starts by building a map (SLAM), since AMCL has
               nothing to localize against. For smabo-sim maps/sim_room.yaml is its
               world exported as a map.
  maps_dir     where maps are saved / listed (default: $SMABO_MAPS_DIR, else
               ~/smabo-maps; docker-compose mounts the host's ~/smabo-maps).
  use_vision   (true)  start the vision pipeline (webrtc_camera ingests the
               smabo-app WebRTC video as /camera/image_raw, then detection).
  use_speech   (true)  start speech_recognizer (/speech/audio -> STT ->
               /speech/recognized).
  use_rosbridge(true)  start rosbridge_suite.
  rosbridge_port (9090).
  arm / hand / breadboard (true|false), mount (none|tt_motor|dg01d_e|neck)
               robot configuration published until a mode starts (each mode
               then publishes its own robot; see mode_manager.MODELS).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    slam = LaunchConfiguration("slam")
    use_vision = LaunchConfiguration("use_vision")
    use_speech = LaunchConfiguration("use_speech")
    use_rosbridge = LaunchConfiguration("use_rosbridge")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    model_args = {k: LaunchConfiguration(k) for k in ("arm", "hand", "breadboard", "mount")}

    brain_ros = get_package_share_directory("smabo_brain_ros")
    description = get_package_share_directory("smabo_description")
    bringup = get_package_share_directory("smabo_bringup")

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("mode", default_value="none"),
        DeclareLaunchArgument("use_vision", default_value="true"),
        DeclareLaunchArgument("use_speech", default_value="true"),
        DeclareLaunchArgument("use_rosbridge", default_value="true"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        DeclareLaunchArgument("map", default_value=""),
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

        # --- mode manager: starts / stops nav (nav.launch.py) or plan (plan.launch.py)
        #     on request from smabo-web, switching the URDF to that mode's robot ---
        Node(
            package="smabo_brain_ros",
            executable="mode_manager",
            name="mode_manager",
            output="screen",
            parameters=[{
                "initial_mode": LaunchConfiguration("mode"),
                "slam": ParameterValue(slam, value_type=bool),
                "map": LaunchConfiguration("map"),
                "maps_dir": LaunchConfiguration("maps_dir"),
                "sim": ParameterValue(sim, value_type=bool),
            }],
        ),

        # --- protocol check (/brain/info for clients, /hello from them) ---
        Node(
            package="smabo_brain_ros",
            executable="protocol_node",
            name="protocol_node",
            output="screen",
            condition=IfCondition(use_rosbridge),
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
