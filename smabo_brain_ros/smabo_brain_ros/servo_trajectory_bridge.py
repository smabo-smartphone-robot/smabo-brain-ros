"""MoveIt execution endpoint for the smabo servo arm.

Exposes a ``control_msgs/FollowJointTrajectory`` action server (the controller
MoveIt's moveit_controllers.yaml points at) and republishes each planned
trajectory on ``/servo/command`` as ``trajectory_msgs/JointTrajectory`` — which
is exactly the shape smabo-esp32 expects over rosbridge (see design.md §4-5 and
smabo-web ``Arm.tsx``).  The ESP32 does the time-based following itself, so this
node simply forwards the trajectory and reports success when its duration
elapses.

With ``simulate:=true`` (no real robot) the node also publishes ``/joint_states``
from the commanded points, giving MoveIt a current robot state to plan from.
Leave it ``false`` on hardware, where the ESP32 publishes ``/joint_states``.
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# All servo joints from smabo-esp32 config (used only for the simulated
# /joint_states stream so MoveIt has a complete state).
_ALL_JOINTS = [
    "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4",
    "head_pan", "left_hand", "right_hand",
]


def _dur_to_sec(d: Duration) -> float:
    return d.sec + d.nanosec * 1e-9


class ServoTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__("servo_trajectory_bridge")

        self.declare_parameter("action_name", "smabo_arm_controller/follow_joint_trajectory")
        self.declare_parameter("command_topic", "/servo/command")
        self.declare_parameter("simulate", False)
        self.declare_parameter("joint_states_rate", 20.0)

        action_name = self.get_parameter("action_name").value
        command_topic = self.get_parameter("command_topic").value
        self._simulate = bool(self.get_parameter("simulate").value)

        # Reentrant group so the /joint_states timer keeps firing while a goal
        # is executing (the execute callback sleeps through the trajectory).
        self._cb_group = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(JointTrajectory, command_topic, 10)

        self._server = ActionServer(
            self,
            FollowJointTrajectory,
            action_name,
            execute_callback=self._execute,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )

        # Simulated joint state (only published when simulate:=true).
        self._positions = {j: 0.0 for j in _ALL_JOINTS}
        if self._simulate:
            self._js_pub = self.create_publisher(JointState, "/joint_states", 10)
            rate = float(self.get_parameter("joint_states_rate").value) or 20.0
            self.create_timer(
                1.0 / rate, self._publish_joint_states, callback_group=self._cb_group
            )

        self.get_logger().info(
            f"servo_trajectory_bridge ready (action='{action_name}', "
            f"command='{command_topic}', simulate={self._simulate})"
        )

    def _publish_joint_states(self) -> None:
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = list(self._positions.keys())
        js.position = list(self._positions.values())
        self._js_pub.publish(js)

    def _execute(self, goal_handle):
        traj: JointTrajectory = goal_handle.request.trajectory

        # Forward the whole trajectory to the ESP32 (it time-follows the points).
        self._cmd_pub.publish(traj)
        self.get_logger().info(
            f"forwarded trajectory: {len(traj.joint_names)} joints, "
            f"{len(traj.points)} points -> {self.get_parameter('command_topic').value}"
        )

        total = _dur_to_sec(traj.points[-1].time_from_start) if traj.points else 0.0
        start = time.monotonic()
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(traj.joint_names)

        # Advance through the points for the trajectory duration, optionally
        # driving the simulated joint state.
        idx = 0
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            elapsed = time.monotonic() - start
            while idx < len(traj.points) and \
                    _dur_to_sec(traj.points[idx].time_from_start) <= elapsed:
                pt = traj.points[idx]
                if self._simulate:
                    for name, pos in zip(traj.joint_names, pt.positions):
                        self._positions[name] = pos
                idx += 1

            goal_handle.publish_feedback(feedback)
            if elapsed >= total:
                break
            time.sleep(0.02)

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoTrajectoryBridge()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
