"""Odometry node: wraps smabo-brain's pure integrator as a ROS 2 node.

Subscribes to raw wheel velocities (``smabo_interfaces/WheelVel``) and publishes
``nav_msgs/Odometry`` plus the ``odom -> base_link`` tf, reusing
``brain.odometry.Odometry`` so the integration math lives in exactly one place
(smabo-brain).

The pure core returns plain pose/twist values; this node supplies ROS time and
builds the ROS messages / tf.  The WebSocket transport in smabo-brain does the
same with wall-clock time (see ``brain/relay.py``) — that separation is what
lets brain-ros wrap brain by simply importing its core.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry as OdometryMsg
from tf2_ros import TransformBroadcaster

try:
    from brain.odometry import (
        Odometry as OdometryCore,
        build_pose_covariance,
        build_twist_covariance,
    )
except ImportError as e:  # pragma: no cover
    raise ImportError(
        "Could not import 'brain' (smabo-brain). Make it importable in this "
        "environment, e.g.  pip install -e /path/to/smabo-brain  or add the "
        "smabo-brain repo root to PYTHONPATH."
    ) from e

from smabo_interfaces.msg import WheelVel


class OdomNode(Node):
    """Integrate /wheel_vel into /odom (+ tf) using smabo-brain's core."""

    def __init__(self) -> None:
        super().__init__("odom")

        # Parameters mirror the ESP32 config keys the core consumes, so the
        # exact same update_config() path is reused.
        self.declare_parameter("wheel_separation", 0.15)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("input_topic", "wheel_vel")
        self.declare_parameter("output_topic", "odom")
        self.declare_parameter("pose_xx", 0.001)
        self.declare_parameter("pose_yy", 0.001)
        self.declare_parameter("pose_aa", 0.001)
        self.declare_parameter("twist_vv", 0.001)
        self.declare_parameter("twist_ww", 0.001)

        val = lambda name: self.get_parameter(name).value  # noqa: E731
        self._publish_tf = val("publish_tf")

        self._core = OdometryCore()
        self._core.update_config({
            "dc": {"wheel_separation": val("wheel_separation")},
            "encoder": {
                "odom_frame": val("odom_frame"),
                "base_frame": val("base_frame"),
                "covariance": {
                    "pose_xx": val("pose_xx"),
                    "pose_yy": val("pose_yy"),
                    "pose_aa": val("pose_aa"),
                    "twist_vv": val("twist_vv"),
                    "twist_ww": val("twist_ww"),
                },
            },
        })

        self._pub = self.create_publisher(OdometryMsg, val("output_topic"), 10)
        self._tf = TransformBroadcaster(self) if self._publish_tf else None
        self.create_subscription(WheelVel, val("input_topic"), self._on_wheel, 10)

        self.get_logger().info("smabo-brain-ros odom node started")

    def _on_wheel(self, msg: WheelVel) -> None:
        r = self._core.integrate(msg.left, msg.right, msg.dt)
        if r is None:
            return

        stamp = self.get_clock().now().to_msg()   # ROS time, not wall clock
        q = Quaternion(z=math.sin(r["theta"] / 2.0), w=math.cos(r["theta"] / 2.0))

        odom = OdometryMsg()
        odom.header.stamp = stamp
        odom.header.frame_id = r["odom_frame"]
        odom.child_frame_id = r["base_frame"]
        odom.pose.pose.position.x = r["x"]
        odom.pose.pose.position.y = r["y"]
        odom.pose.pose.orientation = q
        odom.pose.covariance = build_pose_covariance(r["cov"])
        odom.twist.twist.linear.x = r["vx"]
        odom.twist.twist.angular.z = r["wz"]
        odom.twist.covariance = build_twist_covariance(r["cov"])
        self._pub.publish(odom)

        if self._tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = r["odom_frame"]
            t.child_frame_id = r["base_frame"]
            t.transform.translation.x = r["x"]
            t.transform.translation.y = r["y"]
            t.transform.rotation = q
            self._tf.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
