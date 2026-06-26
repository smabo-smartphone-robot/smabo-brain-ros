"""WebRTC camera ingestion node: terminate smabo-app's WebRTC video in ROS.

Why this exists: rosbridge is a JSON-over-WebSocket bridge, so pushing camera
frames through it as ``sensor_msgs/CompressedImage`` (base64 JSON) is slow and
jittery. Instead this node is a **WebRTC peer** (reusing ``brain.webrtc_hub``)
that receives the phone's video directly (P2P, hardware codec), pulls frames and
republishes them as a **native ``sensor_msgs/Image``** (``/camera/image_raw``)
on the DDS bus. Vision / Nav2 consume that with native ROS transport — fast.

Only the small WebRTC **signaling** (SDP / ICE) rides rosbridge, as plain
``std_msgs/String`` topics:

    app → /webrtc/offer        node → /webrtc/answer
    app → /webrtc/app_ice      (node's ICE is bundled in the answer SDP)
    web → /webrtc/preview {on} node → /webrtc/web_offer
    web → /webrtc/web_answer
    web → /webrtc/web_ice

The hub is asyncio-based while rclpy is not, so an asyncio loop runs on a
background thread; ROS subscription callbacks schedule the hub coroutines onto
it with ``run_coroutine_threadsafe``. rclpy publishers are thread-safe, so the
frame / signaling publishes happen straight from the asyncio thread.

aiortc is imported lazily by ``brain.webrtc_hub``; without it the node logs a
warning and idles (the rest of the graph still runs).
"""

import asyncio
import json
import threading

import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from brain.webrtc_hub import WebRtcHub

# Single logical web preview peer (rosbridge has no per-client addressing, and a
# preview is shown by one viewer at a time). Used as the hub's web "ws" key.
_WEB_PEER = "web"


class WebRtcCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("webrtc_camera")

        self.declare_parameter("frame_id", "camera")
        self.declare_parameter("image_topic", "camera/image_raw")
        self.declare_parameter("config_topic", "vision/config")
        self.declare_parameter("capture_fps", 5.0)

        self._frame_id = self.get_parameter("frame_id").value

        self._img_pub = self.create_publisher(
            Image, self.get_parameter("image_topic").value, 10)
        # Signaling publishers (node → app / web), created up-front.
        self._sig_pubs = {
            "/webrtc/answer":    self.create_publisher(String, "webrtc/answer", 10),
            "/webrtc/web_offer": self.create_publisher(String, "webrtc/web_offer", 10),
        }

        # asyncio loop on a background thread (aiortc lives here).
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        self._hub = WebRtcHub(
            process_frame=self._publish_frame,
            send_app=self._send_app,
            send_web=self._send_web,
        )
        self._hub.set_fps(float(self.get_parameter("capture_fps").value))

        # Signaling subscriptions (app / web → node).
        self.create_subscription(String, "webrtc/offer",      self._on_offer, 10)
        self.create_subscription(String, "webrtc/app_ice",    self._on_app_ice, 10)
        self.create_subscription(String, "webrtc/preview",    self._on_preview, 10)
        self.create_subscription(String, "webrtc/web_answer", self._on_web_answer, 10)
        self.create_subscription(String, "webrtc/web_ice",    self._on_web_ice, 10)
        self.create_subscription(
            String, self.get_parameter("config_topic").value, self._on_config, 10)

        if not self._hub.available:
            self.get_logger().warning(
                "aiortc が無いため WebRTC カメラ取り込みは無効です。"
                "pip install aiortc を実行してください。")
        self.get_logger().info("smabo-brain-ros webrtc_camera node started")

    # --------------------------------------------------------------- asyncio
    def _run_loop(self) -> None:
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _submit(self, coro) -> None:
        asyncio.run_coroutine_threadsafe(coro, self._loop)

    @staticmethod
    def _data(msg: String) -> dict:
        try:
            d = json.loads(msg.data)
            return d if isinstance(d, dict) else {}
        except Exception:
            return {}

    # ----------------------------------------------------- hub callbacks
    async def _publish_frame(self, bgr) -> None:
        """Hub hands us a BGR ndarray → publish it as sensor_msgs/Image."""
        if bgr is None or bgr.ndim != 3:
            return
        h, w = bgr.shape[:2]
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.height = int(h)
        msg.width = int(w)
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(w * 3)
        msg.data = np.ascontiguousarray(bgr).tobytes()
        self._img_pub.publish(msg)

    async def _send_app(self, frame: dict) -> None:
        self._publish_signaling(frame)

    async def _send_web(self, _ws, frame: dict) -> None:
        self._publish_signaling(frame)

    def _publish_signaling(self, frame: dict) -> None:
        topic = frame.get("topic")
        data = (frame.get("msg") or {}).get("data", "")
        pub = self._sig_pubs.get(topic)
        if pub is not None:
            pub.publish(String(data=data))

    # ------------------------------------------------------------ ROS subs
    def _on_offer(self, msg: String) -> None:
        self._submit(self._hub.handle_app_offer(self._data(msg)))

    def _on_app_ice(self, msg: String) -> None:
        self._submit(self._hub.add_app_ice(self._data(msg)))

    def _on_preview(self, msg: String) -> None:
        on = bool(self._data(msg).get("on"))
        if on:
            self._submit(self._hub.start_web_preview(_WEB_PEER))
        else:
            self._submit(self._hub.stop_web_preview(_WEB_PEER))

    def _on_web_answer(self, msg: String) -> None:
        self._submit(self._hub.handle_web_answer(_WEB_PEER, self._data(msg)))

    def _on_web_ice(self, msg: String) -> None:
        self._submit(self._hub.add_web_ice(_WEB_PEER, self._data(msg)))

    def _on_config(self, msg: String) -> None:
        try:
            patch = json.loads(msg.data)
        except Exception:
            return
        if isinstance(patch, dict) and "capture_fps" in patch:
            self._hub.set_fps(patch["capture_fps"])

    # --------------------------------------------------------------- teardown
    def destroy_node(self) -> bool:
        try:
            self._submit(self._hub.close_app())
        except Exception:
            pass
        self._loop.call_soon_threadsafe(self._loop.stop)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebRtcCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
