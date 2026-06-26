"""Strip source prefixes (/web, /esp32, /app) into canonical ROS topics.

Non-ROS clients publish with a source prefix over rosbridge (smabo-web → /web,
smabo-esp32 → /esp32, smabo-app → /app); the ROS graph (Nav2, MoveIt, odom_node)
expects canonical names. This launch starts one ``topic_tools relay`` per topic
to do the mapping — the ROS-native equivalent of the inline strip smabo-brain's
WebSocket relay does (brain/relay.py).

The prefix strings come from ``brain.topics.SOURCE_PREFIXES`` so the convention
lives in exactly one place (smabo-brain), which smabo-brain-ros imports — see the
package README. Outbound topics (/odom, /scan, /map, /plan …) need no relay:
clients subscribe to the canonical names directly over rosbridge.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

try:
    from brain.topics import SOURCE_PREFIXES
except Exception:  # brain not importable (e.g. lint w/o PYTHONPATH) — fallback
    SOURCE_PREFIXES = {"web": "/web", "app": "/app", "esp32": "/esp32"}

# canonical topics each client publishes (prefix is prepended by the client).
# Camera video is NOT here: smabo-app sends it over WebRTC (P2P) to
# webrtc_camera_node, not as a prefixed rosbridge topic — only the small
# /webrtc/* signaling rides rosbridge and is relayed below.
_PREFIXED_PUBLISH = {
    "web":   ["/cmd_vel", "/servo/command", "/initialpose", "/goal_pose",
              "/speech/say", "/expression", "/look_at", "/ping",
              "/vision/config",
              "/webrtc/preview", "/webrtc/web_answer", "/webrtc/web_ice"],
    "esp32": ["/wheel_vel", "/joint_states", "/scan", "/pong"],
    "app":   ["/imu/data", "/gps/fix", "/speech/recognized",
              "/webrtc/offer", "/webrtc/app_ice"],
}


def _relay(name: str, src: str, dst: str) -> Node:
    return Node(
        package="topic_tools",
        executable="relay",
        name=name,
        arguments=[src, dst],
        output="log",
    )


def generate_launch_description():
    nodes = []
    for client, topics in _PREFIXED_PUBLISH.items():
        prefix = SOURCE_PREFIXES[client]
        for canonical in topics:
            src = f"{prefix}{canonical}"
            safe = canonical.strip("/").replace("/", "_")
            nodes.append(_relay(f"relay_{client}_{safe}", src, canonical))
    return LaunchDescription(nodes)
