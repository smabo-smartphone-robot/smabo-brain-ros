"""Speech recognition: speech_recognizer_node.

smabo-app sends a recorded utterance on /speech/audio (audio_common_msgs/
AudioData); this node transcribes it with brain.stt and publishes the text on
/speech/recognized (std_msgs/String), mirroring smabo-brain's WS relay.

Engine / model / language are ROS arguments. Default engine is vosk; pass
stt_engine:=whisper for faster-whisper. For an offline vosk model, point
stt_model at its directory (otherwise vosk fetches a small model by language).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stt_engine = LaunchConfiguration("stt_engine")
    stt_model = LaunchConfiguration("stt_model")
    stt_language = LaunchConfiguration("stt_language")

    return LaunchDescription([
        DeclareLaunchArgument("stt_engine", default_value="vosk",
                              description="vosk | whisper (faster-whisper)"),
        DeclareLaunchArgument("stt_model", default_value="",
                              description="vosk model dir / whisper size (e.g. small)"),
        DeclareLaunchArgument("stt_language", default_value="ja"),

        Node(
            package="smabo_brain_ros",
            executable="speech_recognizer_node",
            name="speech_recognizer",
            output="screen",
            parameters=[{
                "stt_engine": stt_engine,
                "stt_model": stt_model,
                "stt_language": stt_language,
            }],
        ),
    ])
