"""Protocol check for smabo-brain-ros: ``/brain/info`` out, ``/hello`` in.

smabo-brain answers each client's ``/hello`` itself (brain/hello.py). Over
rosbridge there is no per-client reply, so this node publishes ``/brain/info``
(std_msgs/String, ``{"protocol", "brain", "version"}``) every 5 s and right after
each ``/hello`` (relayed from ``/web/hello``, ``/app/hello``, ``/esp32/hello``),
and logs clients whose protocol number differs. Clients compare the number
themselves and warn on their side.
"""

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from brain.topics import PROTOCOL_VERSION

BRAIN_NAME = "smabo-brain-ros"
BRAIN_VERSION = "0.2.0"   # keep in step with setup.py / package.xml
INFO_PERIOD_S = 5.0


def info_json() -> str:
    return json.dumps({"protocol": PROTOCOL_VERSION, "brain": BRAIN_NAME, "version": BRAIN_VERSION})


def check_hello(data: str) -> str | None:
    """A warning for a ``/hello`` with another protocol number, else None."""
    try:
        hello = json.loads(data)
    except ValueError:
        return None
    if not isinstance(hello, dict) or hello.get("protocol") == PROTOCOL_VERSION:
        return None
    return (f"{hello.get('client')} {hello.get('version')} speaks protocol "
            f"{hello.get('protocol')}; {BRAIN_NAME} speaks {PROTOCOL_VERSION}")


class ProtocolNode(Node):
    def __init__(self) -> None:
        super().__init__("protocol_node")
        self._pub = self.create_publisher(String, "/brain/info", 10)
        self.create_subscription(String, "/hello", self._on_hello, 10)
        self.create_timer(INFO_PERIOD_S, self._publish_info)
        self._publish_info()

    def _publish_info(self) -> None:
        self._pub.publish(String(data=info_json()))

    def _on_hello(self, msg: String) -> None:
        warning = check_hello(msg.data)
        if warning:
            self.get_logger().warning(warning)
        else:
            self.get_logger().info(f"hello: {msg.data}")
        self._publish_info()


def main() -> None:
    rclpy.init()
    node = ProtocolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
