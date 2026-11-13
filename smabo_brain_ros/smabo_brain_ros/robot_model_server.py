"""Robot model server: serve the URDF / SRDF / meshes to rosbridge clients.

smabo-web renders the robot itself (three.js) and talks to the ROS graph only
through rosbridge (WebSocket), which cannot serve files. This node exposes one
service, ``smabo_interfaces/srv/GetFile`` on ``/smabo/get_file``:

    uri = "robot_description"           → URDF (from robot_state_publisher)
    uri = "robot_description_semantic"  → SRDF (from move_group)
    uri = "package://<pkg>/<path>"      → mesh file under <pkg>'s share directory

Both descriptions are latched (transient-local) topics, so the node subscribes
with matching QoS and keeps the latest copy. package:// is restricted to mesh
extensions inside the package share directory so no other file can be read.
"""

import array
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from smabo_interfaces.srv import GetFile

_MESH_EXTS = (".stl", ".dae", ".obj")
_MAX_BYTES = 20 * 1024 * 1024


class RobotModelServer(Node):
    def __init__(self) -> None:
        super().__init__("robot_model_server")
        self._docs: dict[str, str] = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        for topic in ("robot_description", "robot_description_semantic"):
            self.create_subscription(
                String, topic, lambda m, t=topic: self._docs.__setitem__(t, m.data), latched)
        self.create_service(GetFile, "smabo/get_file", self._on_get_file)
        self.get_logger().info("smabo-brain-ros robot_model_server started (/smabo/get_file)")

    def _on_get_file(self, req, res):
        uri = req.uri.strip()
        try:
            if uri in ("robot_description", "robot_description_semantic"):
                doc = self._docs.get(uri)
                if doc is None:
                    raise FileNotFoundError(f"{uri} has not been published yet")
                data = doc.encode("utf-8")
            elif uri.startswith("package://"):
                data = self._read_package_file(uri)
            else:
                raise ValueError(f"unsupported uri: {uri}")
        except Exception as e:  # report every failure to the client instead of raising
            res.success, res.message = False, str(e)
            return res
        res.success, res.message, res.data = True, "", array.array("B", data)
        return res

    @staticmethod
    def _read_package_file(uri: str) -> bytes:
        pkg, _, rel = uri[len("package://"):].partition("/")
        if not pkg or not rel or not rel.lower().endswith(_MESH_EXTS):
            raise ValueError(f"only mesh files can be served: {uri}")
        try:
            share = os.path.abspath(get_package_share_directory(pkg))
        except PackageNotFoundError:
            raise FileNotFoundError(f"package not found: {pkg}")
        # Lexical check only (normpath, not realpath): with `colcon build
        # --symlink-install` every share file is a symlink into src/, so resolving
        # links would reject all meshes. ".." traversal is still caught here.
        path = os.path.normpath(os.path.join(share, rel))
        if not path.startswith(share + os.sep):
            raise ValueError(f"path escapes the package: {uri}")
        if os.path.getsize(path) > _MAX_BYTES:
            raise ValueError(f"file too large: {uri}")
        with open(path, "rb") as f:
            return f.read()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotModelServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
