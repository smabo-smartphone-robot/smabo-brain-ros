"""Strip source prefixes (/web, /esp32, /app) into canonical ROS topics.

Non-ROS clients publish with a source prefix over rosbridge (smabo-web → /web,
smabo-esp32 → /esp32, smabo-app → /app); the ROS graph (Nav2, MoveIt, odom_node)
expects canonical names. This launch starts one ``topic_tools relay`` per topic
to do the mapping — the ROS-native equivalent of the inline strip smabo-brain
does (brain/server.py).

The tables come from ``brain.topics`` (``SOURCE_PREFIXES`` and ``PUBLISH``), the
single definition of the protocol in smabo-brain, which this package wraps.
Outbound topics (/odom, /scan, /map, /plan …) need no relay: clients subscribe
to the canonical names directly over rosbridge.

Topics the ESP32 stamps (RESTAMP) go through ``restamp_relay`` instead, which
replaces the ESP32's since-boot stamp with ROS time — MoveIt and AMCL reject
stamps that far from now.

Camera video is NOT relayed: smabo-app sends it over WebRTC to
webrtc_camera_node; only the small /webrtc/* signaling rides rosbridge.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from brain.topics import PUBLISH, SOURCE_PREFIXES

# (client, canonical topic) whose header stamp is the sender's uptime, not ROS time
RESTAMP = {("esp32", "/joint_states"), ("esp32", "/scan")}


def _relay(name: str, src: str, dst: str) -> Node:
    return Node(
        package="topic_tools",
        executable="relay",
        name=name,
        arguments=[src, dst],
        output="log",
    )


def generate_launch_description():
    nodes, restamp = [], []
    for client, topics in PUBLISH.items():
        prefix = SOURCE_PREFIXES[client]
        for canonical, msg_type in topics.items():
            src = f"{prefix}{canonical}"
            if (client, canonical) in RESTAMP:
                restamp.append(f"{src}:{canonical}:{msg_type}")
                continue
            safe = canonical.strip("/").replace("/", "_")
            nodes.append(_relay(f"relay_{client}_{safe}", src, canonical))
    nodes.append(Node(
        package="smabo_brain_ros",
        executable="restamp_relay",
        name="restamp_relay",
        parameters=[{"relays": restamp}],
        output="log",
    ))
    return LaunchDescription(nodes)
