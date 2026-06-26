"""Shared helpers for the smabo-brain-ros vision nodes.

Like the other nodes in this package, these wrap smabo-brain's pure logic
(``brain.vision``) and only add the ROS transport. The detection math, the
pixel→direction projection and the gaze/neck/drive policies all live in
``brain.vision`` so the WebSocket relay (smabo-brain) and these ROS nodes stay
behaviourally identical — the relay applies the same functions over rosbridge
JSON, these apply them over typed ROS messages.

Configuration follows the two-tier scheme documented in design.md: each node
exposes the settings as ROS 2 parameters, *and* subscribes ``/vision/config``
(``std_msgs/String`` carrying JSON) so web/rosbridge clients can override them
at runtime with the very same partial-merge semantics (``vision.merge_config``).
"""

import json

from rclpy.node import Node

from brain import vision
from brain.vision import VisionConfig, merge_config


def stamp_to_dict(stamp) -> dict:
    """builtin_interfaces/Time → the {'sec','nanosec'} dict the pure funcs use."""
    return {"sec": int(stamp.sec), "nanosec": int(stamp.nanosec)}


def vision_config_from_params(node: Node) -> VisionConfig:
    """Build a VisionConfig from whichever vision parameters ``node`` declared.

    Parameters that a given node did not declare fall back to VisionConfig's
    built-in defaults, so each node only needs to declare the keys it cares
    about (e.g. the gaze node ignores the drive gains).
    """
    def g(name, default):
        return node.get_parameter(name).value if node.has_parameter(name) else default

    tm = g("target_marker_id", "")
    raw = {
        "enabled": g("enabled", True),
        "mode": g("mode", "off"),
        "color": g("color", "red"),
        "color_rgb": g("color_rgb", ""),       # hex "#RRGGBB" / "" = use named color
        "color_hue_tol": g("color_hue_tol", 12),
        "color_s_min": g("color_s_min", 70),
        "color_v_min": g("color_v_min", 60),
        "min_area_frac": g("min_area_frac", 0.0008),
        "speak": g("speak", False),
        "aruco_dict": g("aruco_dict", vision.DEFAULT_ARUCO_DICT),
        "target_marker_id": (tm if tm not in (None, "") else None),
        "hfov_deg": g("hfov_deg", vision.DEFAULT_HFOV_DEG),
        "target_joints": {
            "pan": g("pan_joint", "head_pan"),
            "tilt": g("tilt_joint", ""),
            "pan_sign": g("pan_sign", 1.0),
            "tilt_sign": g("tilt_sign", 1.0),
            "gain": g("servo_gain", 1.0),
        },
        "drive": {
            "target_area_frac": g("drive_target_area_frac", 0.10),
            "k_ang": g("drive_k_ang", 1.5),
            "k_lin": g("drive_k_lin", 2.0),
            "max_ang": g("drive_max_ang", 1.0),
            "max_lin": g("drive_max_lin", 0.20),
            "deadzone": g("drive_deadzone", 0.02),
        },
    }
    return VisionConfig(raw)


def ros_detections_to_dicts(msg) -> list:
    """vision_msgs/Detection2DArray → brain.vision detection dicts (for policies)."""
    out = []
    for d in msg.detections:
        bbox = d.bbox
        cls, score = "", 1.0
        if d.results:
            hyp = d.results[0].hypothesis
            cls, score = hyp.class_id, hyp.score
        out.append(vision._det(
            cls, score,
            bbox.center.position.x, bbox.center.position.y,
            bbox.size_x, bbox.size_y,
        ))
    return out


class PolicyNode(Node):
    """Base for the gaze/neck/drive policy nodes.

    Subscribes ``/vision/detections`` and ``/vision/config``, reconstructs the
    detection list, picks the target and projects it to a direction, then hands
    off to ``on_detections`` which each subclass turns into its behaviour
    message. Image dimensions come from parameters (a plain Detection2DArray
    does not carry the source image size; the WS path adds a non-standard hint
    instead — see design.md).
    """

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.declare_parameter("detections_topic", "vision/detections")
        self.declare_parameter("config_topic", "vision/config")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("hfov_deg", vision.DEFAULT_HFOV_DEG)
        self.declare_parameter("target_marker_id", "")
        self.declare_parameter("frame_id", "base_link")
        self._cfg: VisionConfig | None = None  # built by subclass via init_cfg()

        # Imported lazily so this module imports without rclpy message pkgs when
        # only the pure helpers above are needed (e.g. unit tests).
        from vision_msgs.msg import Detection2DArray
        from std_msgs.msg import String
        self.create_subscription(
            Detection2DArray,
            self.get_parameter("detections_topic").value,
            self._on_detections, 10)
        self.create_subscription(
            String,
            self.get_parameter("config_topic").value,
            self._on_config, 10)

    def init_cfg(self) -> None:
        """Call at the end of the subclass __init__ (after all params declared)."""
        self._cfg = vision_config_from_params(self)

    @property
    def frame_id(self) -> str:
        return self.get_parameter("frame_id").value

    def image_wh(self):
        return (int(self.get_parameter("image_width").value),
                int(self.get_parameter("image_height").value))

    def _on_config(self, msg) -> None:
        if self._cfg is None:
            return
        try:
            patch = json.loads(msg.data)
        except Exception:
            self.get_logger().warning("vision/config: data is not valid JSON; ignored")
            return
        if not isinstance(patch, dict):
            return
        self._cfg = VisionConfig(merge_config(self._cfg.to_dict(), patch))

    def _on_detections(self, msg) -> None:
        if self._cfg is None:
            return
        dets = ros_detections_to_dicts(msg)
        target = vision.pick_target(dets, self._cfg.target_marker_id)
        stamp = self.get_clock().now().to_msg()
        self.on_detections(dets, target, self.image_wh(), stamp)

    # subclasses implement this
    def on_detections(self, dets, target, img_wh, stamp) -> None:
        raise NotImplementedError
