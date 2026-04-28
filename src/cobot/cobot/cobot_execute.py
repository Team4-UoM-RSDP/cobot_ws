"""
cobot_execute.py

A ROS2 node that executes incoming joint and gripper commands on the MyCobot 280 Pi,
and publishes the cobot's current state.

Author: Team 4
Date: 2026-04
"""

import math
import time
from typing import List, Optional, Tuple, cast

import rclpy
from control_msgs.action import FollowJointTrajectory
from pymycobot import PI_BAUD, PI_PORT, MyCobot280
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.logging import get_logger
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

_logger = get_logger(__name__)


class CobotExecute(Node):
    def __init__(self) -> None:
        super().__init__("cobot_execute")

        # Declare parameters
        self.declare_parameter("port", PI_PORT)
        self.declare_parameter("baud", str(PI_BAUD))
        self.declare_parameter("joint_cmd_topic", "joint_cmd")
        self.declare_parameter("joint_state_topic", "joint_state")
        self.declare_parameter("joint_speed", 35)  # 0-100%, 100% = 160°/s
        self.declare_parameter("gripper_cmd_topic", "gripper_cmd")
        self.declare_parameter("gripper_state_topic", "gripper_state")
        self.declare_parameter("gripper_speed", 100)  # 0-100%
        self.declare_parameter("joint_state_hz", 10.0)
        self.declare_parameter("joint_cmd_hz", 0.0)

        # Get parameters
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().string_value
        joint_cmd_topic = (
            self.get_parameter("joint_cmd_topic").get_parameter_value().string_value
        )
        joint_state_topic = (
            self.get_parameter("joint_state_topic").get_parameter_value().string_value
        )
        self.joint_speed = (
            self.get_parameter("joint_speed").get_parameter_value().integer_value
        )
        gripper_cmd_topic = (
            self.get_parameter("gripper_cmd_topic").get_parameter_value().string_value
        )
        gripper_state_topic = (
            self.get_parameter("gripper_state_topic").get_parameter_value().string_value
        )
        self.gripper_speed = (
            self.get_parameter("gripper_speed").get_parameter_value().integer_value
        )
        joint_state_hz = (
            self.get_parameter("joint_state_hz").get_parameter_value().double_value
        )
        joint_cmd_hz = (
            self.get_parameter("joint_cmd_hz").get_parameter_value().double_value
        )

        # Initialise pymcobot
        self.mc = MyCobot280(port=port, baudrate=baud)
        time.sleep(0.05)
        if self.mc.get_fresh_mode() == 0:
            self.mc.set_fresh_mode(1)
            time.sleep(0.05)
        try:
            # Initial checks
            power_status = self.mc.is_power_on()
            if power_status != 1:
                self.get_logger().warn(
                    f"is_power_on() returned: {power_status}. Should be 1."
                )
            controller_status = self.mc.is_controller_connected()
            if controller_status != 1:
                self.get_logger().warn(
                    f"is_controller_connected() returned: {controller_status}. Should be 1."
                )
            servo_status = self.mc.is_all_servo_enable()
            if servo_status != 1:
                self.get_logger().warn(
                    f"is_all_servo_enable() returned: {servo_status}. Should be 1."
                )

            # Read initial angles
            initial_angles = self.mc.get_angles()
            if initial_angles and len(initial_angles) >= 6:
                self.get_logger().info(
                    f"Connected to MyCobot 280 Pi. Initial joint angles: {initial_angles[:6]}"
                )
            else:
                self.get_logger().warn(
                    f"get_angles() returned unexpected data: {initial_angles}"
                )
        except Exception as e:
            self.get_logger().error(f"Failed to connect to the MyCobot 280 Pi: {e}")

        # Declare joint names (must match URDF) and load limits
        self.joint_names = [
            "link1_to_link2",
            "link2_to_link3",
            "link3_to_link4",
            "link4_to_link5",
            "link5_to_link6",
            "link6_to_link6_flange",
        ]
        self.joint_limits_deg = self._load_joint_limits_deg()

        # Subscribe to /gripper_cmd_topic
        self.last_gripper_state = None
        self.create_subscription(Int8, gripper_cmd_topic, self.gripper_cmd_callback, 10)

        # Subscribe to /joint_cmd_topic with rate limiting
        self.last_joint_cmd_time = time.time()
        self.joint_cmd_period = 1.0 / joint_cmd_hz if joint_cmd_hz > 0 else 0.0
        self.create_subscription(
            JointState, joint_cmd_topic, self.joint_cmd_callback, 10
        )

        # Publish to /joint_state_topic
        joint_state_period = 1.0 / joint_state_hz if joint_state_hz > 0 else 0.1
        self.create_timer(joint_state_period, self.joint_state_callback)
        self.joint_state_pub = self.create_publisher(JointState, joint_state_topic, 10)

        # Publish to /gripper_state_topic
        self.gripper_state_pub = self.create_publisher(Int8, gripper_state_topic, 10)

        # Action server to follow MoveIt trajectories
        self.trajectory_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "arm_controller/follow_joint_trajectory",
            execute_callback=self.trajectory_execute_callback,
        )

    def joint_cmd_callback(self, msg: JointState) -> None:
        """Executes when a message is received on /joint_cmd_topic."""
        if self.joint_cmd_period > 0:
            current_time = time.time()
            if current_time - self.last_joint_cmd_time < self.joint_cmd_period:
                return
            self.last_joint_cmd_time = current_time
        state_dict = dict(zip(msg.name, msg.position))
        angles = [
            round(math.degrees(state_dict[j]), 3)
            for j in self.joint_names
            if j in state_dict
        ]
        if len(angles) == 6:
            angles = self._clamp_angles_deg(angles)
            try:
                self.mc.send_angles(angles, self.joint_speed)
            except Exception as e:
                self.get_logger().error(f"Send failed: {e}")

    def gripper_cmd_callback(self, msg: Int8) -> None:
        """Executes when a message is received on /gripper_cmd_topic."""
        if msg.data == self.last_gripper_state:
            return
        try:
            self.mc.set_gripper_state(msg.data, self.gripper_speed)
            self.last_gripper_state = msg.data
            self.gripper_state_pub.publish(Int8(data=msg.data))
        except Exception as e:
            self.get_logger().error(f"Gripper failed: {e}")

    def joint_state_callback(self) -> None:
        """Publishes the current joint states at a fixed rate."""
        try:
            if (
                self.mc.is_controller_connected() != 1
                or self.mc.is_power_on() != 1
                or self.mc.is_all_servo_enable() != 1
            ):
                return
            res = cast(List[float], self.mc.get_angles())
            if not res or len(res) < 6:
                return
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.name = self.joint_names
            msg.position = [math.radians(float(a)) for a in res[:6]]
            msg.velocity = [0.0] * 6
            msg.effort = [0.0] * 6
            self.joint_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Feedback error: {e}")

    def trajectory_execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> FollowJointTrajectory.Result:
        """Executes when a trajectory goal is received from MoveIt."""
        traj = goal_handle.request.trajectory
        res = FollowJointTrajectory.Result()
        if not traj.points:
            goal_handle.succeed()
            return res
        for i, point in enumerate(traj.points):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return res
            angles = self._clamp_angles_deg(
                [math.degrees(p) for p in point.positions[:6]]
            )
            try:
                self.mc.send_angles(angles, self.joint_speed)
            except Exception as e:
                self.get_logger().error(f"Error sending angles: {e}")
                res.error_code = -1
                goal_handle.abort()
                return res
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = self.joint_names
            feedback.desired = point
            goal_handle.publish_feedback(feedback)
            if i < len(traj.points) - 1:
                t1, t0 = traj.points[i + 1].time_from_start, point.time_from_start
                delay = (t1.sec + t1.nanosec / 1e9) - (t0.sec + t0.nanosec / 1e9)
                if delay > 0:
                    time.sleep(min(delay, 5.0))
        res.error_code = 0
        goal_handle.succeed()
        return res

    def _load_joint_limits_deg(self) -> List[Tuple[float, float]]:
        """Loads joint limits provided by pymycobot."""
        try:
            limits = []
            for i in range(1, 7):
                limits.append(
                    (self.mc.get_joint_min_angle(i), self.mc.get_joint_max_angle(i))
                )
            return limits
        except Exception as e:
            self.get_logger().error(f"Error loading joint limits: {e}")
            self.get_logger().warn("Proceeding without joint limits.")
            return [(float("-inf"), float("inf"))] * 6

    def _clamp_angles_deg(self, angles: List[float]) -> List[float]:
        """Clamps angles to the joint limits."""
        clamped = []
        for i, a in enumerate(angles):
            min_l, max_l = self.joint_limits_deg[i]
            if a < min_l:
                clamped.append(min_l)
            elif a > max_l:
                clamped.append(max_l)
            else:
                clamped.append(a)
        return clamped


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the cobot_execute node."""
    try:
        rclpy.init(args=args)
        cobot_execute = CobotExecute()
        rclpy.spin(cobot_execute)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        _logger.error(f"Error spinning node: {e}", exc_info=True)


if __name__ == "__main__":
    main()
