"""
cobot_execute.py

A ROS2 node that executes incoming joint and gripper commands on the MyCobot 280 Pi,
and publishes the cobot's current state.

Author: Team 4
Date: 2026-04
"""

import math
import time
from typing import List, Optional, Tuple

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
    """The main ROS2 node for executing commands"""

    def __init__(self) -> None:
        super().__init__("cobot_execute")

        # Declare port and baudrate provided by pymycobot
        self.declare_parameter("port", PI_PORT)
        self.declare_parameter("baud", str(PI_BAUD))

        # Declare topics
        self.declare_parameter(
            "joint_commands_topic", "joint_commands"
        )  # Joint angle commands
        self.declare_parameter(
            "joint_states_topic", "joint_states"
        )  # Current joint angles
        self.declare_parameter(
            "joint_speed", 35
        )  # Speed for joint movements (0-100%), 100% = 160°/s

        self.declare_parameter(
            "gripper_command_topic", "gripper_command"
        )  # Commands received to open/close gripper
        self.declare_parameter(
            "gripper_state_topic", "gripper_state"
        )  # Current gripper state
        self.declare_parameter(
            "gripper_speed", 100
        )  # Speed for gripper movement (0-100%)

        self.declare_parameter(
            "feedback_rate_hz", 10.0
        )  # Rate to publish joint feedback
        self.declare_parameter(
            "joint_commands_rate_hz", 0.0
        )  # Rate limiting for incoming joint commands (0 = no limit)

        # Get declared parameters
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().string_value
        joint_commands_topic = (
            self.get_parameter("joint_commands_topic")
            .get_parameter_value()
            .string_value
        )
        joint_states_topic = (
            self.get_parameter("joint_states_topic").get_parameter_value().string_value
        )
        self.joint_speed = (
            self.get_parameter("joint_speed").get_parameter_value().integer_value
        )
        gripper_command_topic = (
            self.get_parameter("gripper_command_topic")
            .get_parameter_value()
            .string_value
        )
        gripper_state_topic = (
            self.get_parameter("gripper_state_topic").get_parameter_value().string_value
        )
        self.gripper_speed = (
            self.get_parameter("gripper_speed").get_parameter_value().integer_value
        )
        feedback_rate = (
            self.get_parameter("feedback_rate_hz").get_parameter_value().double_value
        )
        joint_commands_rate = (
            self.get_parameter("joint_commands_rate_hz")
            .get_parameter_value()
            .double_value
        )

        # Log configuration
        self.get_logger().info("port:%s, baud:%s" % (port, baud))

        self.get_logger().info(
            "Subscribing to joint commands on: %s" % joint_commands_topic
        )
        if joint_commands_rate != 0:
            self.get_logger().info(
                "Rate limiting incoming joint commands at: %.2f Hz"
                % joint_commands_rate
            )
        self.get_logger().info(
            "Subscribing to gripper command on: %s" % gripper_command_topic
        )
        self.get_logger().info("Publishing joint states to: %s" % (joint_states_topic))
        self.get_logger().info("Publishing gripper state to: %s" % gripper_state_topic)
        self.get_logger().info(
            "Publishing feedback at frequency: %.2f Hz" % feedback_rate
        )

        self.get_logger().info("Joint speed set to: %d, (max: 100)" % self.joint_speed)
        self.get_logger().info(
            "Gripper speed set to: %d, (max: 100)" % self.gripper_speed
        )

        # Initialise connection to myCobot 280 pi
        self.mc = MyCobot280(port=port, baudrate=baud)
        time.sleep(0.05)
        if self.mc.get_fresh_mode() == 0:
            self.mc.set_fresh_mode(1)
            time.sleep(0.05)

        try:
            # Check Atom controller is powered on
            power_status = self.mc.is_power_on()
            if power_status != 1:
                self.get_logger().warn(
                    "is_power_on returned: {power_status}. Should be 1."
                )
            else:
                self.get_logger().info("is_power_on returned: {power_status}. Success")

            # Check connection to Atom controller
            controller_status = self.mc.is_controller_connected()
            if controller_status != 1:
                self.get_logger().warn(
                    "is_controller_connected returned: {controller_status}. Should be 1."
                )
            else:
                self.get_logger().info(
                    "is_controller_connected returned: {controller_status}. Success"
                )

            # Check all servos are enabled
            servo_status = self.mc.is_all_servo_enable()
            if servo_status != 1:
                self.get_logger().warn(
                    "is_all_servo_enable returned: {servo_status}. Should be 1."
                )
            else:
                self.get_logger().info(
                    "is_all_servo_enable returned: {servo_status}. Success"
                )

            # Try to read initial angles
            initial_angles = self.mc.get_angles()
            if initial_angles and len(initial_angles) >= 6:
                self.get_logger().info(
                    "Successfully connected to MyCobot 280 Pi. Initial joint angles: {}".format(
                        initial_angles[:6]
                    )
                )
            else:
                self.get_logger().warn(
                    "get_angles() returned unexpected data: {}".format(initial_angles)
                )

        # Catch and log any exceptions
        except Exception as e:
            self.get_logger().error("Failed to connect to MyCobot 280 Pi: {}".format(e))

        # Joint names (must match URDF used in MoveIt2)
        self.joint_names = [
            "link1_to_link2",
            "link2_to_link3",
            "link3_to_link4",
            "link4_to_link5",
            "link5_to_link6",
            "link6_to_link6_flange",
        ]

        # Load joint limits provided by pymycobot
        self.joint_limits_deg = self._load_joint_limits_deg()

        # Rate limiting for incoming joint commands
        self.last_sync_time = time.time()
        self.min_sync_interval = (
            1.0 / joint_commands_rate if joint_commands_rate > 0 else 0.0
        )

        # Subscribe to /joint_commands_topic
        self.joint_commands_sub = self.create_subscription(
            msg_type=JointState,
            topic=joint_commands_topic,
            callback=self.joint_commands_callback,
            qos_profile=10,
        )
        self.joint_commands_sub

        # Create publisher for /joint_states_topic
        self.joint_feedback_pub = self.create_publisher(
            JointState, joint_states_topic, 10
        )

        # Timer for publishing to /joint_states_topic
        feedback_interval = 1.0 / feedback_rate if feedback_rate > 0 else 0.1
        self.feedback_timer = self.create_timer(
            feedback_interval, self.publish_joint_feedback
        )

        # Gripper state tracking
        self.last_gripper_state = None
        self.current_gripper_state = None

        # Subscribe to /gripper_command_topic
        self.gripper_command_sub = self.create_subscription(
            msg_type=Int8,
            topic=gripper_command_topic,
            callback=self.gripper_command_callback,
            qos_profile=10,
        )
        self.gripper_command_sub

        # Create publisher for current /gripper_state_topic
        self.gripper_state_pub = self.create_publisher(Int8, gripper_state_topic, 10)

        # Create action server for executing MoveIt2 trajectories
        self.trajectory_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "arm_controller/follow_joint_trajectory",
            execute_callback=self.trajectory_execute_callback,
        )
        self.get_logger().info(
            "Follow Joint Trajectory action server started on 'arm_controller/follow_joint_trajectory'"
        )

    def joint_commands_callback(self, msg: JointState) -> None:
        """Process incoming joint state commands from MoveIt2.

        Args:
            msg: JointState message containing desired joint positions.
        """
        # Rate limiting check
        current_time = time.time()
        if self.min_sync_interval > 0:
            if current_time - self.last_sync_time < self.min_sync_interval:
                return  # Skip this message
            self.last_sync_time = current_time

        # Create dictionary mapping joint names to positions
        joint_state_dict = {name: msg.position[i] for i, name in enumerate(msg.name)}

        # Extract angles in correct order
        data_list = []
        missing_joints = []
        for joint in self.joint_names:
            if joint in joint_state_dict:
                radians_to_angles = round(math.degrees(joint_state_dict[joint]), 3)
                data_list.append(radians_to_angles)
            else:
                missing_joints.append(joint)

        # Only send if we have all 6 joints
        if len(data_list) == 6:
            clamped_angles, had_clamp = self._clamp_angles_deg(data_list)
            if had_clamp:
                self.get_logger().warn(
                    "Clamped joint angles to limits. Command: {} -> {}".format(
                        data_list, clamped_angles
                    ),
                    throttle_duration_sec=2.0,
                )
            self.get_logger().debug("Sending angles: {}".format(clamped_angles))
            try:
                self.mc.send_angles(clamped_angles, self.joint_speed)
            except Exception as e:
                self.get_logger().error("Failed to send angles to robot: {}".format(e))
        elif missing_joints:
            self.get_logger().warn(
                "Missing joints in message: {}. Available: {}".format(
                    missing_joints, list(joint_state_dict.keys())
                ),
                throttle_duration_sec=5.0,
            )

    def gripper_command_callback(self, msg: Int8) -> None:
        """Handle incoming gripper command messages (0 = open, 1 = close).

        Args:
            msg: Int8 message where 0 = open, 1 = close.
        """
        gripper_state = msg.data
        if gripper_state == self.last_gripper_state:
            return  # No change needed

        try:
            # Set gripper state: 0 = open, 1 = close
            self.mc.set_gripper_state(gripper_state, self.gripper_speed)
            self.last_gripper_state = gripper_state
            self.current_gripper_state = gripper_state

            state_name = "open" if gripper_state == 0 else "close"
            self.get_logger().info(
                "Gripper state set to {} ({})".format(gripper_state, state_name)
            )

            # Publish current state
            state_msg = Int8()
            state_msg.data = gripper_state
            self.gripper_state_pub.publish(state_msg)
        except Exception as e:
            self.get_logger().error(
                "Failed to send gripper command: {}".format(e),
                throttle_duration_sec=2.0,
            )

    def publish_joint_feedback(self) -> None:
        """Read actual joint angles from hardware and publish as feedback."""
        try:
            # Check hardware status before reading
            controller_ok = self.mc.is_controller_connected()
            if controller_ok != 1:
                self.get_logger().warn(
                    "Controller not connected (status: {})".format(controller_ok),
                    throttle_duration_sec=5.0,
                )
                return

            power_ok = self.mc.is_power_on()
            if power_ok != 1:
                self.get_logger().warn(
                    "Power is not on (status: {})".format(power_ok),
                    throttle_duration_sec=5.0,
                )
                return

            servos_ok = self.mc.is_all_servo_enable()
            if servos_ok != 1:
                self.get_logger().warn(
                    "Not all servos are enabled (status: {})".format(servos_ok),
                    throttle_duration_sec=5.0,
                )
                return

            # Read current angles from robot (in degrees)
            angles_result = self.mc.get_angles()

            # Ensure we have a valid list
            if angles_result is None:
                self.get_logger().warn(
                    "get_angles() returned None - hardware may not be responding",
                    throttle_duration_sec=5.0,
                )
                return

            angles_deg: List = (
                list(angles_result) if isinstance(angles_result, (list, tuple)) else []
            )
            if len(angles_deg) < 6:
                self.get_logger().warn(
                    "get_angles() returned {} angles, expected 6: {}".format(
                        len(angles_deg), angles_deg
                    ),
                    throttle_duration_sec=5.0,
                )
                return

            # Convert to radians
            angles_rad = [math.radians(float(angle)) for angle in angles_deg[:6]]

            # Create and publish JointState message
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.header.frame_id = "world"
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = angles_rad
            joint_state_msg.velocity = [0.0] * 6  # Not available from hardware
            joint_state_msg.effort = [0.0] * 6  # Not available from hardware

            self.joint_feedback_pub.publish(joint_state_msg)
            self.get_logger().debug(
                "Published joint feedback: {}".format(angles_deg[:6])
            )
        except Exception as e:
            self.get_logger().warn(
                "Error reading joint feedback: {}".format(e), throttle_duration_sec=5.0
            )

    def _load_joint_limits_deg(self) -> List[Tuple[float, float]]:
        """Loads min and max angle limits in degrees for each joint."""
        limits = []
        try:
            for joint_index in range(1, 7):
                max_angle = self.mc.get_joint_max_angle(joint_index)
                time.sleep(0.05)
                min_angle = self.mc.get_joint_min_angle(joint_index)
                time.sleep(0.05)
                limits.append((min_angle, max_angle))
            self.get_logger().info("Loaded joint angle limits: {}".format(limits))
        except Exception as e:
            self.get_logger().warn(
                "Failed to read joint limits: {}. Proceeding without limits.".format(e)
            )
            limits = [(float("-inf"), float("inf"))] * 6
        return limits

    def _clamp_angles_deg(self, angles_deg: List[float]) -> Tuple[List[float], bool]:
        """Clamp desired joint angles to safe limits."""
        clamped = []
        had_clamp = False
        for i, angle in enumerate(angles_deg):
            min_lim, max_lim = self.joint_limits_deg[i]
            if angle < min_lim:
                clamped.append(min_lim)
                had_clamp = True
            elif angle > max_lim:
                clamped.append(max_lim)
                had_clamp = True
            else:
                clamped.append(angle)
        return clamped, had_clamp

    def trajectory_execute_callback(
        self, goal_handle: ServerGoalHandle
    ) -> FollowJointTrajectory.Result:
        """Execute trajectory goal.

        Args:
            goal_handle: The goal handle containing the trajectory.

        Returns:
            FollowJointTrajectory.Result with error code.
        """
        trajectory = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()

        self.get_logger().info(
            "Executing trajectory with %d points" % len(trajectory.points)
        )

        # Guard against empty trajectory
        if not trajectory.points:
            self.get_logger().warn("Received empty trajectory")
            goal_handle.succeed()
            return result

        # Execute each waypoint in the trajectory
        for i, point in enumerate(trajectory.points):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Trajectory execution cancelled by client")
                goal_handle.canceled()
                return result

            # Convert radians to degrees
            angles_deg = [math.degrees(angle) for angle in point.positions[:6]]

            # Clamp angles to limits
            clamped_angles, _ = self._clamp_angles_deg(angles_deg)

            self.get_logger().debug("Sending waypoint %d: %s" % (i, clamped_angles))
            try:
                self.mc.send_angles(clamped_angles, self.joint_speed)
            except Exception as e:
                self.get_logger().error("Failed to send angles to robot: %s" % str(e))
                result.error_code = -1
                goal_handle.abort()
                return result

            # Publish feedback with current point
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = self.joint_names
            feedback.desired = point
            goal_handle.publish_feedback(feedback)

            # Wait for trajectory time per point if specified
            if i < len(trajectory.points) - 1:
                time_from_start = trajectory.points[i + 1].time_from_start
                current_time = trajectory.points[i].time_from_start
                wait_time = (time_from_start.sec + time_from_start.nanosec / 1e9) - (
                    current_time.sec + current_time.nanosec / 1e9
                )
                if wait_time > 0:
                    self.get_logger().debug(
                        "Waiting %.2f seconds for next waypoint" % wait_time
                    )
                    time.sleep(min(wait_time, 5.0))  # Cap wait time at 5 seconds

        self.get_logger().info("Trajectory execution completed successfully")
        result.error_code = 0  # Success
        goal_handle.succeed()
        return result


def main(args: Optional[List[str]] = None) -> None:
    """Main entry point for the cobot_execute node.

    Args:
        args: Command line arguments passed to ROS2.
    """
    try:
        rclpy.init(args=args)
        cobot_execute = CobotExecute()
        rclpy.spin(cobot_execute)
    except KeyboardInterrupt:
        _logger.info("Received keyboard interrupt, shutting down")
    except Exception as e:
        _logger.error(f"Node encountered an error: {e}", exc_info=True)


if __name__ == "__main__":
    main()
