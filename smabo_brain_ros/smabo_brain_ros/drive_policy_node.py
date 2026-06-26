"""Drive policy node: /vision/detections → /cmd_vel (geometry_msgs/Twist).

Makes the mobile base follow the target: turn to keep it centered and
approach/recede to hold a target apparent size, via ``brain.vision.to_cmd_vel``.
Publishes a stop twist when there is no target so the base never runs away.
"""

import rclpy
from geometry_msgs.msg import Twist

from brain import vision

from .vision_common import PolicyNode


class DrivePolicyNode(PolicyNode):
    def __init__(self) -> None:
        super().__init__("drive_policy")
        self.declare_parameter("output_topic", "cmd_vel")
        self.declare_parameter("drive_target_area_frac", 0.10)
        self.declare_parameter("drive_k_ang", 1.5)
        self.declare_parameter("drive_k_lin", 2.0)
        self.declare_parameter("drive_max_ang", 1.0)
        self.declare_parameter("drive_max_lin", 0.20)
        self.declare_parameter("drive_deadzone", 0.02)
        self._pub = self.create_publisher(
            Twist, self.get_parameter("output_topic").value, 10)
        self.init_cfg()
        self.get_logger().info("smabo-brain-ros drive_policy node started")

    def on_detections(self, dets, target, img_wh, stamp) -> None:
        tw = vision.to_cmd_vel(target, img_wh, self._cfg)
        msg = Twist()
        msg.linear.x = float(tw["linear"]["x"])
        msg.angular.z = float(tw["angular"]["z"])
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DrivePolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
