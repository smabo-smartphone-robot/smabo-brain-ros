"""Mode manager: start / stop the navigation or the motion-planning stack.

bringup starts only the base (rosbridge, relays, robot_state_publisher, odom,
robot_model_server, …). The two heavy stacks are started on request, one at a
time, as a child ``ros2 launch``:

  nav   smabo_bringup nav.launch.py   Nav2 servers + nav_manager (SLAM / AMCL)
  plan  smabo_bringup plan.launch.py  MoveIt2 move_group + servo_trajectory_bridge

Each mode drives a different robot, so starting one also swaps the URDF that
robot_state_publisher publishes (it accepts a new ``robot_description`` while
running), before the stack starts and reads it:

  nav   the DG01D-E mobile base with the LD19 and the breadboard, no arm
  plan  the arm

smabo-web drives it over rosbridge (the Navigation and Motion Plan tabs):

  /web/mode/command → /mode/command  std_msgs/String, JSON:
      {"op": "start", "mode": "nav" | "plan"}   (stops the other one first)
      {"op": "stop"}
  /mode/state  std_msgs/String, JSON (latched; on change and every second):
      {"mode": "none" | "nav" | "plan",
       "status": "stopped" | "starting" | "running" | "stopping" | "error",
       "message", "model": {xacro args of the URDF in use}}
"""

from __future__ import annotations

import json
import os
import queue
import subprocess
import threading
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from .child_process import start_group, stop_group

# the robot each mode drives (smabo.urdf.xacro args)
MODELS: dict[str, dict[str, str]] = {
    "nav": {"arm": "false", "hand": "false", "breadboard": "true", "mount": "dg01d_e"},
    "plan": {"arm": "true", "hand": "false", "breadboard": "false", "mount": "none"},
}
LAUNCH = {"nav": "nav.launch.py", "plan": "plan.launch.py"}
# a node that is up once the stack is ready to use
READY_NODE = {"nav": "bt_navigator", "plan": "move_group"}
START_TIMEOUT_S = 90.0


def parse_command(text: str) -> dict:
    """The command in a /mode/command message; ValueError when it is not one."""
    try:
        cmd = json.loads(text)
    except ValueError:
        raise ValueError("mode/command: not JSON") from None
    if not isinstance(cmd, dict) or cmd.get("op") not in ("start", "stop"):
        raise ValueError("mode/command: op must be start or stop")
    if cmd["op"] == "start" and cmd.get("mode") not in MODELS:
        raise ValueError("mode/command: mode must be nav or plan")
    return cmd


def xacro_args(model: dict[str, str]) -> list[str]:
    return [f"{k}:={v}" for k, v in model.items()]


def build_urdf(model: dict[str, str]) -> str:
    """The URDF of smabo.urdf.xacro with these args (RuntimeError when xacro fails)."""
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("smabo_description"), "urdf",
                        "smabo.urdf.xacro")
    r = subprocess.run(["xacro", path, *xacro_args(model)], capture_output=True,
                       text=True, timeout=60)
    if r.returncode != 0:
        raise RuntimeError(f"xacro failed: {(r.stderr or r.stdout).strip()[-200:]}")
    return r.stdout


class ModeManager(Node):
    def __init__(self) -> None:
        super().__init__("mode_manager")
        self.declare_parameter("initial_mode", "none")    # none | nav | plan
        self.declare_parameter("slam", False)             # nav: start in SLAM (mapping) mode
        self.declare_parameter("map", "")                 # nav: map to start AMCL on
        self.declare_parameter("maps_dir", "")
        self.declare_parameter("sim", False)              # plan: simulate the arm
        self.declare_parameter("rsp_node", "robot_state_publisher")
        val = lambda n: self.get_parameter(n).value  # noqa: E731
        # ros2 launch rejects an empty "name:=" — leave those out (the launch file's default applies)
        self._launch_args = {
            "nav": [a for a in (f"slam:={str(bool(val('slam'))).lower()}", f"map:={val('map')}",
                                f"maps_dir:={val('maps_dir')}") if not a.endswith(":=")],
            "plan": [f"sim:={str(bool(val('sim'))).lower()}"],
        }

        self.mode = "none"
        self.status = "stopped"
        self.message = ""
        self.model: dict[str, str] = {}
        self._proc: subprocess.Popen | None = None
        self._commands: queue.Queue[dict] = queue.Queue()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(String, "mode/state", latched)
        self.create_subscription(String, "mode/command", self._on_command, 10)
        self._rsp = self.create_client(SetParameters, f"/{val('rsp_node')}/set_parameters")
        self.create_timer(1.0, self._publish_state)

        if val("initial_mode") in MODELS:
            self._commands.put({"op": "start", "mode": val("initial_mode")})
        threading.Thread(target=self._worker, daemon=True).start()
        self._publish_state()

    # ------------------------------------------------------------------ #
    # state
    # ------------------------------------------------------------------ #
    def _publish_state(self) -> None:
        p = self._proc
        if p is not None and p.poll() is not None and self.status in ("starting", "running"):
            self._proc = None
            self._set(mode="none", status="error",
                      message=f"{self.mode} stopped unexpectedly (exit {p.returncode})")
            return
        state = {"mode": self.mode, "status": self.status, "message": self.message,
                 "model": self.model}
        self._state_pub.publish(String(data=json.dumps(state, ensure_ascii=False)))

    def _set(self, **kw) -> None:
        for k, v in kw.items():
            setattr(self, k, v)
        if kw.get("message"):
            self.get_logger().info(kw["message"])
        self._publish_state()

    # ------------------------------------------------------------------ #
    # commands (one at a time, off the executor thread)
    # ------------------------------------------------------------------ #
    def _on_command(self, msg: String) -> None:
        try:
            cmd = parse_command(msg.data)
        except ValueError as e:
            self._set(message=str(e))
            return
        if self.status in ("starting", "stopping"):
            self._set(message="busy; try again in a moment")
            return
        self._commands.put(cmd)

    def _worker(self) -> None:
        while rclpy.ok():
            cmd = self._commands.get()
            try:
                if cmd["op"] == "stop":
                    self._stop()
                else:
                    self._start(cmd["mode"])
            except Exception as e:  # keep the manager alive whatever happens
                self.get_logger().error(f"{cmd}: {e!r}")
                self._stop_child()
                self._set(mode="none", status="error", message=f"{cmd['op']} failed: {e}")

    def _stop(self) -> None:
        if self._proc is None:
            self._set(mode="none", status="stopped", message="nothing running")
            return
        name = self.mode
        self._set(status="stopping", message=f"stopping {name}")
        self._stop_child()
        self._set(mode="none", status="stopped", message=f"{name} stopped")

    def _start(self, mode: str) -> None:
        if self._proc is not None:
            if self.mode == mode and self.status == "running":
                self._set(message=f"{mode} is already running")
                return
            self._stop()
        self._set(mode=mode, status="starting", message=f"starting {mode}: switching the robot model")
        model = MODELS[mode]
        self._set_robot_description(model)
        self._set(model=dict(model), message=f"starting {mode}")
        args = ["ros2", "launch", "smabo_bringup", LAUNCH[mode],
                *xacro_args(model), *self._launch_args[mode]]
        self.get_logger().info("starting: " + " ".join(args))
        self._proc = start_group(args)
        end = time.monotonic() + START_TIMEOUT_S
        while time.monotonic() < end:
            if self._proc is None or self._proc.poll() is not None:
                raise RuntimeError(f"{mode} exited while starting")
            if READY_NODE[mode] in self.get_node_names():
                self._set(status="running", message=f"{mode} running")
                return
            time.sleep(0.5)
        raise RuntimeError(f"{mode} did not come up within {START_TIMEOUT_S:.0f} s")

    # ------------------------------------------------------------------ #
    # robot model: run xacro, hand the URDF to robot_state_publisher
    # ------------------------------------------------------------------ #
    def _set_robot_description(self, model: dict[str, str]) -> None:
        urdf = build_urdf(model)
        if not self._rsp.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("robot_state_publisher is not running")
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name="robot_description",
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=urdf))]
        done = threading.Event()
        future = self._rsp.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(10.0) or future.result() is None:
            raise RuntimeError("robot_state_publisher did not answer")
        res = future.result().results[0]
        if not res.successful:
            raise RuntimeError(f"robot_state_publisher refused the model: {res.reason}")

    # ------------------------------------------------------------------ #
    # child process
    # ------------------------------------------------------------------ #
    def _stop_child(self) -> None:
        p, self._proc = self._proc, None
        stop_group(p, sigint_wait=15.0)

    def destroy_node(self) -> bool:
        self._stop_child()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModeManager()
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
