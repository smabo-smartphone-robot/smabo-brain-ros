"""Speech recognizer node: wraps smabo-brain's STT as a ROS 2 node.

smabo-app handles only the wake word; on wake it records the utterance and
publishes it on ``/speech/audio`` (audio_common_msgs/AudioData = 16 kHz mono
WAV bytes). This node transcribes that with ``brain.stt`` and publishes the
text as ``std_msgs/String`` on ``/speech/recognized`` (shown in smabo-web) —
exactly like smabo-brain's WebSocket relay, reusing the same pure logic.

STT is CPU-heavy, so audio is transcribed on a worker thread (rclpy publishers
are thread-safe). The engine (vosk default, faster-whisper optional) and model
come from ROS parameters; both libraries are imported lazily by ``brain.stt``,
so without them the node simply logs that STT is disabled.
"""

import threading

import rclpy
from rclpy.node import Node

from audio_common_msgs.msg import AudioData
from std_msgs.msg import String

from brain.stt import SttEngine, wav_to_samples


class SpeechRecognizerNode(Node):
    def __init__(self) -> None:
        super().__init__("speech_recognizer")

        self.declare_parameter("stt_engine", "vosk")        # vosk | whisper
        self.declare_parameter("stt_model", "")             # vosk dir / whisper size
        self.declare_parameter("stt_language", "ja")
        self.declare_parameter("audio_topic", "speech/audio")
        self.declare_parameter("recognized_topic", "speech/recognized")

        self._stt = SttEngine(
            engine=self.get_parameter("stt_engine").value,
            model=self.get_parameter("stt_model").value,
            language=self.get_parameter("stt_language").value,
        )

        self._pub = self.create_publisher(
            String, self.get_parameter("recognized_topic").value, 10)
        self.create_subscription(
            AudioData, self.get_parameter("audio_topic").value, self._on_audio, 10)

        # Worker thread: transcribe queued utterances in order (FIFO), off the
        # executor thread. The backlog is capped so a slow engine can't grow
        # memory without bound.
        self._queue: list[bytes] = []
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._running = True
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()

        if not self._stt.available:
            self.get_logger().warning(
                "STT が無効です（/speech/audio は無視されます）。vosk もしくは "
                "faster-whisper を導入してください。")
        self.get_logger().info("smabo-brain-ros speech_recognizer node started")

    def _on_audio(self, msg: AudioData) -> None:
        raw = bytes(msg.data)
        if not raw:
            return
        with self._cv:
            self._queue.append(raw)
            if len(self._queue) > 8:
                self._queue.pop(0)
            self._cv.notify()

    def _loop(self) -> None:
        while self._running:
            with self._cv:
                while self._running and not self._queue:
                    self._cv.wait()
                if not self._running:
                    return
                raw = self._queue.pop(0)
            samples, sr = wav_to_samples(raw)
            if samples is None or samples.size == 0:
                continue
            try:
                text = self._stt.transcribe(samples, sr)
            except Exception:
                self.get_logger().error("STT failed", throttle_duration_sec=5.0)
                continue
            if text:
                self.get_logger().info("STT: %r", text)
                self._pub.publish(String(data=text))

    def destroy_node(self) -> bool:
        self._running = False
        with self._cv:
            self._cv.notify_all()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpeechRecognizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
