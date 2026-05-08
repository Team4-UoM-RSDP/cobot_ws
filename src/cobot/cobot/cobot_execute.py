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
from sensor_msgs.msg import JointStates
from std_msgs.msg import Int8

_logger = get_logger(__name__)


class CobotExecute(Node):
    def __init__(self) -> None:
        super().__init__("cobot_execute")

        # Declare ROS2 parameters
        self.declare_parameter("port", PI_PORT)
        self.declare_parameter("baud", str(PI_BAUD))
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("joint_state_hz", 10.0)
        self.declare_parameter("joint_cmd_topic", "joint_cmd")
        self.declare_parameter("joint_cmd_hz", 0.0)
        self.declare_parameter("joint_speed", 35)
        self.declare_parameter("gripper_state_topic", "gripper_state")
        self.declare_parameter("gripper_cmd_topic", "gripper_cmd")
        self.declare_parameter("gripper_speed", 80)
        self.declare_parameter("start_stop_topic", "cobot_start_stop")
        self.declare_parameter("cobot_stopped", 0)

        # Get ROS2 parameters
        port: str = self.get_parameter("port").get_parameter_value().string_value
        baud: str = self.get_parameter("baud").get_parameter_value().string_value
        joint_states_topic: str = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        joint_state_hz: float = (
            self.get_parameter("joint_state_hz").get_parameter_value().double_value
        )
        joint_cmd_topic: str = (
            self.get_parameter("joint_cmd_topic").get_parameter_value().string_value
        )
        joint_cmd_hz: float = (
            self.get_parameter("joint_cmd_hz").get_parameter_value().double_value
        )
        self.joint_speed: int = (
            self.get_parameter("joint_speed").get_parameter_value().integer_value
        )
        gripper_state_topic: str = (
            self.get_parameter("gripper_state_topic").get_parameter_value().string_value
        )
        gripper_cmd_topic: str = (
            self.get_parameter("gripper_cmd_topic").get_parameter_value().string_value
        )
        self.gripper_speed: int = (
            self.get_parameter("gripper_speed").get_parameter_value().integer_value
        )
        start_stop_topic: str = (
            self.get_parameter("start_stop_topic").get_parameter_value().string_value
        )
        self.cobot_stopped: int = (
            self.get_parameter("cobot_stopped").get_parameter_value().integer_value
        )

        # Log configuration
        self.get_logger().info(f"Port: {port}, Baudrate: {baud}")
        self.get_logger().info(f"Publishing joint states: /{joint_states_topic}")
        self.get_logger().info(f"Subscribing to joint commands: /{joint_cmd_topic}")
        self.get_logger().info(f"Joint speed: {self.joint_speed}%")
        self.get_logger().info(f"Publishing gripper state: /{gripper_state_topic}")
        self.get_logger().info(f"Subscribing to gripper commands: /{gripper_cmd_topic}")
        self.get_logger().info(f"Gripper speed set to: {self.gripper_speed}%")

        # Initialise pymcobot
        self.mc = MyCobot280(port=port, baudrate=baud)
        time.sleep(0.05)
        if self.mc.get_fresh_mode() == 0:
            self.mc.set_fresh_mode(1)
            time.sleep(0.05)
        try:
            # Wait until servos are ready before proceeding
            start_time = time.time()
            servo_status = self.mc.is_all_servo_enable()
            while servo_status != 1 and time.time() - start_time < 5.0:
                self.get_logger().info("Waiting for servos...")
                time.sleep(0.1)
                servo_status = self.mc.is_all_servo_enable()
            if servo_status == 1:
                self.get_logger().info("Servos are enabled.")
                # Read initial angles
                initial_angles = self.mc.get_angles()
                if initial_angles and len(initial_angles) >= 6:
                    self.get_logger().info(
                        f"Connected to MyCobot 280 Pi. Initial joint angles: {initial_angles[:6]}"
                    )
                else:
                    self.get_logger().warning(
                        f"get_angles() returned unexpected data: {initial_angles}"
                    )
            else:
                self.get_logger().error(
                    f"Timeout waiting for servos: is_all_servo_enable = {servo_status}"
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
            "gripper_controller",
        ]
        self.joint_limits_deg = self._load_joint_limits_deg()
        
        # Gripper joint angle limits (radians)
        self.gripper_min_angle = -0.7
        self.gripper_max_angle = 0.15

        # Subscribe to /gripper_cmd_topic
        self.gripper_state = None
        self.create_subscription(Int8, gripper_cmd_topic, self.gripper_cmd_callback, 10)

        # Subscribe to /joint_cmd_topic with rate limiting
        self.last_joint_cmd_time = time.time()
        self.joint_cmd_period = 1.0 / joint_cmd_hz if joint_cmd_hz > 0 else 0.0
        self.create_subscription(
            JointState, joint_cmd_topic, self.joint_cmd_callback, 10
        )

        # Subscribe to /start_stop_topic to stop the cobot immediately
        self.create_subscription(
            Int8,
            start_stop_topic,
            self.start_stop_callback,
            10,
        )

        # Publish to /joint_states at a fixed rate
        joint_state_period = 1.0 / joint_state_hz if joint_state_hz > 0 else 0.1
        self.create_timer(joint_state_period, self.joint_state_callback)
        self.joint_state_pub = self.create_publisher(JointState, joint_states_topic, 10)

        # Publish to /gripper_state when the gripper state changes
        self.gripper_state_pub = self.create_publisher(Int8, gripper_state_topic, 10)

        # Action server to follow MoveIt trajectories
        self.trajectory_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "arm_controller/follow_joint_trajectory",
            execute_callback=self.trajectory_execute_callback,
        )

    def start_stop_callback(self, msg: Int8) -> None:
        """Handles  commands from /cobot_start_stop."""
        if msg.data == 1 and self.cobot_stopped != 1:
            self.mc.stop()
        elif msg.data == 0 and self.cobot_stopped != 0:
            self.mc.resume()
        else:
            self.get_logger().warn(
                f"Received invalid start/stop command: {msg.data}. Current state: {self.cobot_stopped}"
            )

    def joint_cmd_callback(self, msg: JointState) -> None:
        """Executes when a message is received on /joint_cmd_topic."""
        # Limit joint command rate to set frequency
        if self.joint_cmd_period > 0:
            current_time = time.time()
            if current_time - self.last_joint_cmd_time < self.joint_cmd_period:
                return
            self.last_joint_cmd_time = current_time
        # Convert JointState message to a dictionary
        state_dict = dict(zip(msg.name, msg.position))
        # Extract angles and convert to degrees
        angles = [
            round(math.degrees(state_dict[j]), 3)
            for j in self.joint_names
            if j in state_dict
        ]
        # Clamp angles to joint limits and send to the cobot
        if len(angles) == 6:
            angles = self._clamp_angles_deg(angles)
            try:
                self.mc.send_angles(angles, self.joint_speed)
            except Exception as e:
                self.get_logger().error(f"Failed to execute joint commands: {e}")

    def gripper_cmd_callback(self, msg: Int8) -> None:
        """Executes when a message is received on /gripper_cmd_topic."""
        # Ignore redundant or invalid commands
        self.get_logger().info(f"Received gripper command: {msg.data}")
        self.get_logger().info(
            f"Current gripper position: {self.mc.get_gripper_value()}"
        )
        if msg.data == self.gripper_state:
            return
        if msg.data not in (0, 100):
            self.get_logger().warn(
                f"Invalid gripper command: {msg.data}. Expected 0 (close) or 100 (open)."
            )
            return
        try:
            # Send the command (100=open, 0=closed)
            self.mc.set_gripper_value(
                gripper_value=msg.data,
                speed=self.gripper_speed,
                gripper_type=1,
                is_torque=1,
            )
            self.get_logger().info(f"Gripper command sent: {msg.data}")
            # If closing, wait for gripper to settle and detect grasp
            if msg.data == 0:
                # Wait for gripper to start moving
                time.sleep(2.0)
                # Wait for gripper to stop moving (object grasped or fully closed)
                while cast(int, self.mc.is_gripper_moving()) == 1:
                    time.sleep(0.1)
                # If gripper didn't fully close, assume a block is grasped
                grasp_val = cast(int, self.mc.get_gripper_value())
                if grasp_val > 10:
                    self.get_logger().info(f"Object grasped at position: {grasp_val}")
                    # Maintain grasp position
                    self.mc.set_gripper_value(
                        gripper_value=grasp_val,
                        speed=self.gripper_speed,
                        gripper_type=1,
                        is_torque=1,
                    )
            else:
                time.sleep(1.0)
            self.gripper_state = msg.data
            # Publish the new gripper state
            self.gripper_state_pub.publish(Int8(data=self.gripper_state))
        except Exception as e:
            self.get_logger().error(f"Failed to execute gripper commands: {e}")

    def joint_state_callback(self) -> None:
        """Publishes the current joint states at a fixed rate."""
        # Return if not all servos are ready
        try:
            if self.mc.is_all_servo_enable() != 1:
                return
            # Get current angles
            res = cast(List[float], self.mc.get_angles())
            if not res or len(res) < 6:
                return
            # Get gripper position and convert to joint angle
            gripper_value = cast(int, self.mc.get_gripper_value())
            # Map gripper value (0-100) to joint angle (-0.7 to 0.15 radians)
            gripper_angle = self.gripper_min_angle + (gripper_value / 100.0) * (
                self.gripper_max_angle - self.gripper_min_angle
            )
            # Publish as a JointState message
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.name = self.joint_names
            msg.position = [math.radians(float(a)) for a in res[:6]] + [gripper_angle]
            msg.velocity = [0.0] * 7
            msg.effort = [0.0] * 7
            self.joint_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Feedback error: {e}")

    def trajectory_execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> FollowJointTrajectory.Result:
        """Executes when a trajectory goal is received from MoveIt."""
        traj = goal_handle.request.trajectory
        res = FollowJointTrajectory.Result()
        # Succeed if trajectory is empty
        if not traj.points:
            goal_handle.succeed()
            return res
        # Execute sequentially through trajectory points
        for i, point in enumerate(traj.points):
            # Check for cancellation at each point
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return res
            # Convert trajectory points to joint angles and clamp
            angles = self._clamp_angles_deg(
                [math.degrees(p) for p in point.positions[:6]]
            )
            # Send joint angles to the cobot
            try:
                self.mc.send_angles(angles, self.joint_speed)
            except Exception as e:
                self.get_logger().error(f"Failed to execute trajectory: {e}")
                res.error_code = -1
                goal_handle.abort()
                return res
            # Publish action feedback
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = self.joint_names[:6]  # Only arm joints from MoveIt
            feedback.desired = point
            goal_handle.publish_feedback(feedback)
            if i < len(traj.points) - 1:
                t1, t0 = traj.points[i + 1].time_from_start, point.time_from_start
                delay = (t1.sec + t1.nanosec / 1e9) - (t0.sec + t0.nanosec / 1e9)
                if delay > 0:
                    time.sleep(min(delay, 5.0))
        # Succeed after executing all trajectory points
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
        # Fallback to infinite limits
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
        _logger.error(f"Error spinning node: {e}")


if __name__ == "__main__":
    main()
