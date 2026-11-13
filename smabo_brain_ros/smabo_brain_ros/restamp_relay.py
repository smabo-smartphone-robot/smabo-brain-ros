"""Relay prefixed ESP32 topics to their canonical names, re-stamped with ROS time.

smabo-esp32 stamps its messages with ``ticks_ms`` (milliseconds since boot): it
has no clock in common with the ROS host. A plain ``topic_tools relay`` passes
that stamp through, and the ROS graph then sees data from "1970 + uptime":
MoveIt rejects every execution ("couldn't receive full current joint state
within 1s") and AMCL / slam_toolbox drop the scans.

This node subscribes to ``<prefix><topic>`` and republishes on ``<topic>`` with
``header.stamp`` set to the time the message arrived (the same thing the plain
relay would deliver, apart from the stamp). One node handles every topic given
in ``relays`` as ``"src:dst:type"`` strings.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, LaserScan

# message types with a std_msgs/Header that this relay knows
TYPES = {
    "sensor_msgs/msg/JointState": JointState,
    "sensor_msgs/msg/LaserScan": LaserScan,
}


def parse_relay(spec: str) -> tuple[str, str, str]:
    """``"/esp32/scan:/scan:sensor_msgs/msg/LaserScan"`` -> (src, dst, type)."""
    src, dst, msg_type = spec.split(":", 2)
    if msg_type not in TYPES:
        raise ValueError(f"restamp_relay: unsupported type {msg_type!r} for {src}")
    return src, dst, msg_type


class RestampRelay(Node):
    def __init__(self) -> None:
        super().__init__("restamp_relay")
        self.declare_parameter("relays", [""])
        specs = [s for s in self.get_parameter("relays").value if s]
        self._keep = []
        for spec in specs:
            src, dst, msg_type = parse_relay(spec)
            cls = TYPES[msg_type]
            # reliable, like the plain relay: it matches both reliable and
            # best-effort (sensor data) subscribers
            pub = self.create_publisher(cls, dst, 10)
            self._keep.append(self.create_subscription(cls, src, self._forward(pub), 10))
            self.get_logger().info(f"restamp {src} -> {dst} ({msg_type})")

    def _forward(self, pub):
        def cb(msg) -> None:
            msg.header.stamp = self.get_clock().now().to_msg()
            pub.publish(msg)
        return cb


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RestampRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
