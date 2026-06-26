"""Vision pipeline: webrtc_camera + image_processor + gaze / neck / drive nodes.

webrtc_camera_node terminates smabo-app's WebRTC video (camera bypasses
rosbridge for real-time — see design.md) and republishes frames as a native
sensor_msgs/Image on /camera/image_raw. image_processor consumes that and
publishes /vision/detections (+ /vision/markers); the three policy nodes consume
it and emit /look_at, /servo/command and /cmd_vel respectively. Each behaviour is
launched as its own node so it can be enabled/disabled or retuned independently;
drop the policy nodes you don't want.

All of them also subscribe /vision/config (std_msgs/String JSON), so a web /
rosbridge client can override the parameters below at runtime (including
capture_fps, which sets the webrtc_camera frame rate).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    color = LaunchConfiguration("color")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    hfov_deg = LaunchConfiguration("hfov_deg")
    capture_fps = LaunchConfiguration("capture_fps")

    common = {
        "image_width": image_width,
        "image_height": image_height,
        "hfov_deg": hfov_deg,
    }

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="off",
                              description="off | aruco | color | face | qr"),
        DeclareLaunchArgument("color", default_value="red"),
        DeclareLaunchArgument("image_width", default_value="640"),
        DeclareLaunchArgument("image_height", default_value="480"),
        DeclareLaunchArgument("hfov_deg", default_value="60.0"),
        DeclareLaunchArgument("capture_fps", default_value="5.0",
                              description="WebRTC frame rate fed into vision (1-30)"),

        Node(
            package="smabo_brain_ros",
            executable="webrtc_camera_node",
            name="webrtc_camera",
            output="screen",
            parameters=[{
                "image_topic": "camera/image_raw",
                "capture_fps": capture_fps,
            }],
        ),
        Node(
            package="smabo_brain_ros",
            executable="image_processor_node",
            name="image_processor",
            output="screen",
            parameters=[{
                "enabled": True,
                "mode": mode,
                "color": color,
                "hfov_deg": hfov_deg,
                "input_topic": "camera/image_raw",
                "throttle_hz": 30.0,
            }],
        ),
        Node(
            package="smabo_brain_ros",
            executable="gaze_policy_node",
            name="gaze_policy",
            output="screen",
            parameters=[common],
        ),
        Node(
            package="smabo_brain_ros",
            executable="neck_policy_node",
            name="neck_policy",
            output="screen",
            parameters=[{**common, "pan_joint": "head_pan"}],
        ),
        Node(
            package="smabo_brain_ros",
            executable="drive_policy_node",
            name="drive_policy",
            output="screen",
            parameters=[common],
        ),
    ])
