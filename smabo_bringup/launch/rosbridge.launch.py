"""rosbridge_suite — the single WS<->ROS2 boundary for smabo.

smabo-web / smabo-app / smabo-esp32 connect here over WebSocket (default :9090,
the same port smabo-brain uses, so clients need no change). rosapi is
included for tools that list topics/services over rosbridge.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    port = ParameterValue(LaunchConfiguration("port"), value_type=int)

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="9090"),

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
                # The node's own default is 1 MB; larger messages are split into
                # "fragment" ops, which roslibjs cannot reassemble. Meshes served by
                # /smabo/get_file (base_link.stl ≈ 1 MB → ~1.4 MB base64) exceed it.
                # 10 MB matches rosbridge's official launch file default.
                "max_message_size": 10000000,
            }],
        ),
        Node(
            package="rosapi",
            executable="rosapi_node",
            name="rosapi",
            output="screen",
        ),
    ])
