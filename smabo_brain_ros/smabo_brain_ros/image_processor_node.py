"""Image processor node: wraps smabo-brain's vision detector as a ROS 2 node.

Subscribes a **native ``sensor_msgs/Image``** (``bgr8``) on ``/camera/image_raw``
— published by ``webrtc_camera_node``, which terminates smabo-app's WebRTC video
and republishes frames on the DDS bus (camera deliberately bypasses rosbridge so
real-time is preserved; see design.md). Runs the configured detector from
``brain.vision`` and publishes ``vision_msgs/Detection2DArray`` on
``/vision/detections`` plus the recognized AR ids / QR contents as
``std_msgs/String`` on ``/vision/markers``. When ``speak`` is on it forwards a
*changed* string to ``/speech/say`` (brain-side dedup), mirroring smabo-brain's
relay pipeline.

CV is heavy, so frames are processed on a single worker thread that always picks
the latest frame and drops the rest (rclpy publishers are thread-safe). The
policy stages (gaze/neck/drive) live in separate nodes that consume
``/vision/detections`` — see ``gaze_policy_node`` / ``neck_policy_node`` /
``drive_policy_node``.
"""

import json
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    BoundingBox2D,
    ObjectHypothesisWithPose,
)

from brain import vision
from brain.vision import VisionConfig, merge_config

from .vision_common import vision_config_from_params


class ImageProcessorNode(Node):
    def __init__(self) -> None:
        super().__init__("image_processor")

        # Detector configuration (also overridable at runtime via /vision/config).
        self.declare_parameter("enabled", True)
        self.declare_parameter("mode", "off")
        self.declare_parameter("color", "red")
        self.declare_parameter("color_rgb", "")        # hex "#RRGGBB" / "" = named color
        self.declare_parameter("color_hue_tol", 12)
        self.declare_parameter("color_s_min", 70)
        self.declare_parameter("color_v_min", 60)
        self.declare_parameter("min_area_frac", 0.0008)
        self.declare_parameter("speak", False)
        self.declare_parameter("aruco_dict", vision.DEFAULT_ARUCO_DICT)
        self.declare_parameter("target_marker_id", "")
        self.declare_parameter("hfov_deg", vision.DEFAULT_HFOV_DEG)
        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("input_topic", "camera/image_raw")
        self.declare_parameter("detections_topic", "vision/detections")
        self.declare_parameter("markers_topic", "vision/markers")
        self.declare_parameter("config_topic", "vision/config")
        self.declare_parameter("speech_topic", "speech/say")
        self.declare_parameter("throttle_hz", 10.0)

        self._frame_id = self.get_parameter("frame_id").value
        hz = float(self.get_parameter("throttle_hz").value)
        self._min_interval = 1.0 / hz if hz > 0 else 0.0
        self._cfg: VisionConfig = vision_config_from_params(self)
        self._last_spoken: str | None = None
        self._last_t = 0.0

        self._det_pub = self.create_publisher(
            Detection2DArray, self.get_parameter("detections_topic").value, 10)
        self._mark_pub = self.create_publisher(
            String, self.get_parameter("markers_topic").value, 10)
        self._say_pub = self.create_publisher(
            String, self.get_parameter("speech_topic").value, 10)

        self.create_subscription(
            Image, self.get_parameter("input_topic").value,
            self._on_image, 10)
        self.create_subscription(
            String, self.get_parameter("config_topic").value,
            self._on_config, 10)

        # Worker thread: process the latest frame only, drop the rest.
        self._latest = None  # (bgr ndarray) or None
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._running = True
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

        self.get_logger().info("smabo-brain-ros image_processor node started")

    # ------------------------------------------------------------------ subs
    def _on_image(self, msg: Image) -> None:
        # Reconstruct the BGR ndarray from the raw Image (bgr8, no decode).
        if msg.encoding not in ("bgr8", "rgb8") or msg.height == 0 or msg.width == 0:
            return
        try:
            bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            if msg.encoding == "rgb8":
                bgr = bgr[:, :, ::-1]
        except Exception:
            return
        with self._cv:
            self._latest = bgr
            self._cv.notify()

    def _on_config(self, msg: String) -> None:
        try:
            patch = json.loads(msg.data)
        except Exception:
            self.get_logger().warning("vision/config: data is not valid JSON; ignored")
            return
        if isinstance(patch, dict):
            self._cfg = VisionConfig(merge_config(self._cfg.to_dict(), patch))

    # --------------------------------------------------------------- worker
    def _loop(self) -> None:
        while self._running:
            with self._cv:
                while self._running and self._latest is None:
                    self._cv.wait()
                if not self._running:
                    return
                bgr = self._latest
                self._latest = None
            cfg = self._cfg
            if not cfg.enabled or cfg.mode == "off":
                continue
            now = time.monotonic()
            if (now - self._last_t) < self._min_interval:
                continue
            self._last_t = now
            try:
                self._process(bgr, cfg)
            except Exception:
                self.get_logger().error("vision processing failed", throttle_duration_sec=5.0)

    def _process(self, bgr, cfg: VisionConfig) -> None:
        if bgr is None:
            return
        detections, strings, _img_wh = vision.run_detector(bgr, cfg)
        stamp = self.get_clock().now().to_msg()

        arr = Detection2DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self._frame_id
        for d in detections:
            det = Detection2D()
            det.header = arr.header
            bb = BoundingBox2D()
            bb.center.position.x = float(d["cx"])
            bb.center.position.y = float(d["cy"])
            bb.center.theta = 0.0
            bb.size_x = float(d["w"])
            bb.size_y = float(d["h"])
            det.bbox = bb
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(d["class_id"])
            hyp.hypothesis.score = float(d["score"])
            det.results.append(hyp)
            arr.detections.append(det)
        self._det_pub.publish(arr)

        if strings:
            joined = ",".join(strings)
            self._mark_pub.publish(String(data=joined))
            if cfg.speak and joined != self._last_spoken:
                self._last_spoken = joined
                self._say_pub.publish(String(data=joined))

    def destroy_node(self) -> bool:
        self._running = False
        with self._cv:
            self._cv.notify_all()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImageProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
