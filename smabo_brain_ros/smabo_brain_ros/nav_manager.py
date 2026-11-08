"""Navigation mode manager: build a map with SLAM, save it, navigate on it.

Nav2's navigation servers (planner, controller, behaviours, bt_navigator) run
all the time (bringup). What changes is who provides the ``map`` frame and the
``/map`` topic, and this node starts / stops that part as a child process:

  mapping     slam_toolbox (online async): /scan + /odom -> /map, map -> odom
  navigation  map_server + AMCL (nav2_bringup localization_launch.py) on a
              saved map: /map from the file, map -> odom from AMCL
  off         neither (e.g. to drive by hand with only /odom)

smabo-web drives it over rosbridge (the Nav tab):

  /web/nav/command → /nav/command  std_msgs/String, JSON:
      {"op": "mapping"}                      start SLAM (a new, empty map)
      {"op": "save", "name": "room_1"}       save the SLAM map as <maps_dir>/room_1.{yaml,pgm}
      {"op": "navigate", "name": "room_1"}   localise on a saved map (AMCL)
      {"op": "stop"}                         neither
  /nav/state  std_msgs/String, JSON (on change and every second):
      {"mode", "map", "maps": [{name, saved, resolution}], "busy", "message", "maps_dir"}

When switching straight from SLAM to navigation on the map just made, the
robot's pose in the SLAM map is handed to AMCL as /initialpose, so it can
navigate right away. Otherwise set the initial pose in the Nav tab.

Maps live in ``maps_dir`` (parameter, else $SMABO_MAPS_DIR, else ~/smabo-maps;
docker-compose mounts the host's ~/smabo-maps there).
"""

from __future__ import annotations

import json
import os
import queue
import signal
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .nav_maps import check_name, list_maps, maps_dir, yaml_path

MODES = ("off", "mapping", "navigation")


class NavManager(Node):
    def __init__(self) -> None:
        super().__init__("nav_manager")
        self.declare_parameter("maps_dir", "")
        self.declare_parameter("initial_mode", "navigation")   # off | mapping | navigation
        self.declare_parameter("initial_map", "")               # yaml path or saved map name
        self.declare_parameter("nav2_params", "")
        self.declare_parameter("slam_params", "")
        val = lambda n: self.get_parameter(n).value  # noqa: E731

        self.maps_dir = maps_dir(val("maps_dir") or None)
        os.makedirs(self.maps_dir, exist_ok=True)
        self.nav2_params = val("nav2_params")
        self.slam_params = val("slam_params")

        self.mode = "off"
        self.map = ""            # name (saved map) or yaml path of the map in use
        self.busy = False
        self.message = ""
        self._proc: subprocess.Popen | None = None
        self._last_slam_pose = None
        self._commands: queue.Queue[dict] = queue.Queue()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(String, "nav/state", latched)
        self._initpose_pub = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 1)
        self.create_subscription(String, "nav/command", self._on_command, 10)
        self._tf = Buffer(cache_time=Duration(seconds=10))
        self._tf_listener = TransformListener(self._tf, self)
        self.create_timer(1.0, self._publish_state)

        first = {"mapping": {"op": "mapping"}, "off": {"op": "stop"}}.get(
            val("initial_mode"), {"op": "navigate", "path": val("initial_map")})
        self._commands.put(first)
        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(f"maps in {self.maps_dir}")

    # ------------------------------------------------------------------ #
    # state
    # ------------------------------------------------------------------ #
    def _publish_state(self) -> None:
        if self._proc is not None and self._proc.poll() is not None and not self.busy:
            self._set(message=f"{self.mode} stopped unexpectedly (exit {self._proc.returncode})",
                      mode="off", map="")
            self._proc = None
            return
        state = {"mode": self.mode, "map": self.map, "maps": list_maps(self.maps_dir),
                 "busy": self.busy, "message": self.message, "maps_dir": self.maps_dir}
        self._state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def _set(self, **kw) -> None:
        for k, v in kw.items():
            setattr(self, k, v)
        if "message" in kw and kw["message"]:
            self.get_logger().info(kw["message"])
        self._publish_state()

    # ------------------------------------------------------------------ #
    # commands (handled one at a time off the executor thread)
    # ------------------------------------------------------------------ #
    def _on_command(self, msg: String) -> None:
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            self._set(message="nav/command: not JSON")
            return
        if not isinstance(cmd, dict) or cmd.get("op") not in ("mapping", "save", "navigate", "stop"):
            self._set(message='nav/command: op must be mapping, save, navigate or stop')
            return
        if self.busy:
            self._set(message="busy; try again in a moment")
            return
        self._commands.put(cmd)

    def _worker(self) -> None:
        while rclpy.ok():
            cmd = self._commands.get()
            self._set(busy=True)
            try:
                getattr(self, "_do_" + cmd["op"])(cmd)
            except ValueError as e:
                self._set(message=str(e))
            except Exception as e:  # keep the manager alive whatever happens
                self.get_logger().error(f"{cmd}: {e!r}")
                self._set(message=f"{cmd['op']} failed: {e}")
            finally:
                self._set(busy=False)

    def _do_stop(self, _cmd: dict) -> None:
        self._stop_child()
        self._set(mode="off", map="", message="navigation off")

    def _do_mapping(self, _cmd: dict) -> None:
        self._stop_child()
        args = ["ros2", "run", "slam_toolbox", "async_slam_toolbox_node", "--ros-args",
                "-r", "__node:=slam_toolbox"]
        if self.slam_params:
            args += ["--params-file", self.slam_params]
        self._start_child(args)
        self._set(mode="mapping", map="", message="SLAM started: drive around to build the map")

    def _do_save(self, cmd: dict) -> None:
        if self.mode != "mapping":
            raise ValueError("save works while building a map (SLAM)")
        name = check_name(str(cmd.get("name", "")))
        base = os.path.join(self.maps_dir, name)
        r = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", base, "--ros-args",
             "-p", "save_map_timeout:=10.0", "-p", "map_subscribe_transient_local:=true"],
            capture_output=True, text=True, timeout=40)
        if r.returncode != 0 or not os.path.exists(base + ".yaml"):
            tail = (r.stderr or r.stdout).strip().splitlines()[-1:] or ["no output"]
            raise RuntimeError(f"map_saver failed: {tail[0]}")
        self._set(map=name, message=f"saved map '{name}'")

    def _do_navigate(self, cmd: dict) -> None:
        if cmd.get("path"):
            path = cmd["path"]
            if not os.path.isabs(path) and not path.endswith(".yaml"):
                path = yaml_path(self.maps_dir, path)
            label = os.path.splitext(os.path.basename(path))[0]
        else:
            label = check_name(str(cmd.get("name", "")))
            path = yaml_path(self.maps_dir, label)
        if not os.path.exists(path):
            raise ValueError(f"no such map: {path}")
        # Coming from SLAM on this very map: remember where the robot is so AMCL
        # can start from there instead of waiting for an initial pose.
        handover = self._robot_in_map() if self.mode == "mapping" and self.map == label else None
        self._stop_child()
        args = ["ros2", "launch", "nav2_bringup", "localization_launch.py", f"map:={path}", "autostart:=true"]
        if self.nav2_params:
            args.append(f"params_file:={self.nav2_params}")
        self._start_child(args)
        self._set(mode="navigation", map=label, message=f"navigating on '{label}' (AMCL)")
        if handover is not None:
            if self._wait_for_amcl(25.0):
                self._send_initial_pose(*handover)
                self._set(message=f"navigating on '{label}': initial pose taken over from SLAM")
            else:
                self._set(message=f"navigating on '{label}': set the initial pose")
        else:
            self._set(message=f"navigating on '{label}': set the initial pose")

    # ------------------------------------------------------------------ #
    # child process
    # ------------------------------------------------------------------ #
    def _start_child(self, args: list[str]) -> None:
        self.get_logger().info("starting: " + " ".join(args))
        self._proc = subprocess.Popen(args, start_new_session=True)

    def _stop_child(self) -> None:
        p, self._proc = self._proc, None
        if p is None or p.poll() is not None:
            return
        for sig, wait in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 5.0)):
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                return
            try:
                p.wait(timeout=wait)
                break
            except subprocess.TimeoutExpired:
                continue
        time.sleep(1.0)        # let the old map -> odom publisher go before the next one starts

    # ------------------------------------------------------------------ #
    # SLAM -> AMCL pose handover
    # ------------------------------------------------------------------ #
    def _robot_in_map(self):
        try:
            t = self._tf.lookup_transform("map", "base_link", Time(), timeout=Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().warning(f"no map -> base_link from SLAM ({e}); set the initial pose by hand")
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return tr.x, tr.y, q.z, q.w

    def _wait_for_amcl(self, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            # AMCL subscribes to /initialpose once it is active
            if self.count_subscribers("initialpose") > 0 and self.count_publishers("map") > 0:
                time.sleep(1.0)
                return True
            time.sleep(0.5)
        return False

    def _send_initial_pose(self, x: float, y: float, qz: float, qw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        # stamp 0 = "latest": AMCL looks up odom -> base_link at this time
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = qz, qw
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05
        cov[35] = 0.03
        msg.pose.covariance = cov
        for _ in range(3):           # in case the first one lands before AMCL is listening
            self._initpose_pub.publish(msg)
            time.sleep(0.5)

    def destroy_node(self) -> bool:
        self._stop_child()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
