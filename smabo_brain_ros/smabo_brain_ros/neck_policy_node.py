"""Neck policy node: /vision/detections → /servo/command (JointTrajectory).

Projects the target to a direction and turns it into a neck-servo command using
``brain.vision.direction_to_servo``, which only emits joints that are both
configured and actually present on the robot. By default it drives ``head_pan``
only (head tilt is not a default joint); declare ``tilt_joint`` to enable tilt.
"""

import rclpy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from brain import vision

from .vision_common import PolicyNode, stamp_to_dict


class NeckPolicyNode(PolicyNode):
    def __init__(self) -> None:
        super().__init__("neck_policy")
        self.declare_parameter("output_topic", "servo/command")
        self.declare_parameter("pan_joint", "head_pan")
        self.declare_parameter("tilt_joint", "")
        self.declare_parameter("pan_sign", 1.0)
        self.declare_parameter("tilt_sign", 1.0)
        self.declare_parameter("servo_gain", 1.0)
        # Joints actually available on the robot; empty = don't restrict.
        self.declare_parameter("joints_available", [""])
        self._pub = self.create_publisher(
            JointTrajectory, self.get_parameter("output_topic").value, 10)
        self.init_cfg()
        self.get_logger().info("smabo-brain-ros neck_policy node started")

    def _joints_available(self):
        vals = [j for j in self.get_parameter("joints_available").value if j]
        return set(vals) if vals else None

    def on_detections(self, dets, target, img_wh, stamp) -> None:
        if target is None:
            return
        direction = vision.bbox_to_direction(
            target["cx"], target["cy"], img_wh[0], img_wh[1], self._cfg.hfov_deg)
        servo = vision.direction_to_servo(
            direction, self._cfg, stamp_to_dict(stamp), self._joints_available())
        if servo is None:
            return
        jt = JointTrajectory()
        jt.header.stamp = stamp
        jt.joint_names = list(servo["joint_names"])
        sp = servo["points"][0]
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in sp["positions"]]
        tfs = sp["time_from_start"]
        pt.time_from_start = Duration(sec=int(tfs["sec"]), nanosec=int(tfs["nanosec"]))
        jt.points = [pt]
        self._pub.publish(jt)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NeckPolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
