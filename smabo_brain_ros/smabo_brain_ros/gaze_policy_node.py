"""Gaze policy node: /vision/detections → /look_at (geometry_msgs/PoseStamped).

Picks the target detection and projects its pixel position to a look-at
direction (REP-103: x forward, y left, z up) using ``brain.vision`` so the math
matches smabo-brain's relay. smabo-app drives the eyes from /look_at.
"""

import rclpy
from geometry_msgs.msg import PoseStamped

from brain import vision

from .vision_common import PolicyNode


class GazePolicyNode(PolicyNode):
    def __init__(self) -> None:
        super().__init__("gaze_policy")
        self.declare_parameter("output_topic", "look_at")
        self._pub = self.create_publisher(
            PoseStamped, self.get_parameter("output_topic").value, 10)
        self.init_cfg()
        self.get_logger().info("smabo-brain-ros gaze_policy node started")

    def on_detections(self, dets, target, img_wh, stamp) -> None:
        if target is None:
            return
        x, y, z = vision.bbox_to_direction(
            target["cx"], target["cy"], img_wh[0], img_wh[1], self._cfg.hfov_deg)
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        self._pub.publish(ps)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazePolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
