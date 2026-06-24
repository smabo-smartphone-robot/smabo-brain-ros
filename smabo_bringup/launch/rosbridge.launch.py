"""rosbridge_suite — the single WS<->ROS2 boundary for smabo.

smabo-web / smabo-app / smabo-esp32 connect here over WebSocket (default :9090,
the same port the legacy smabo-brain relay used, so it is a drop-in). rosapi is
included so the web client can query topics/services for the 3D viewer.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = ParameterValue(LaunchConfiguration("port"), value_type=int)
    # tf2_web_republisher is optional (feeds TF to the ros3djs viewer). Off by
    # default since it isn't packaged on every Humble mirror; enable when present.
    use_tf2_web = LaunchConfiguration("tf2_web_republisher")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="9090"),
        DeclareLaunchArgument("tf2_web_republisher", default_value="false"),

        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[{
                "port": port,
                # smabo clients publish/subscribe in rosbridge v2 JSON.
                "call_services_in_new_thread": True,
                "send_action_goals_in_new_thread": True,
            }],
        ),
        Node(
            package="rosapi",
            executable="rosapi_node",
            name="rosapi",
            output="screen",
        ),
        Node(
            package="tf2_web_republisher",
            executable="tf2_web_republisher",
            name="tf2_web_republisher",
            output="screen",
            condition=IfCondition(use_tf2_web),
        ),
    ])
