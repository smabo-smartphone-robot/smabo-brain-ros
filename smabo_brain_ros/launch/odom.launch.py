from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="smabo_brain_ros",
            executable="odom_node",
            name="odom",
            output="screen",
            parameters=[{
                "wheel_separation": 0.15,
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_tf": True,
                "input_topic": "wheel_vel",
                "output_topic": "odom",
            }],
        ),
    ])
