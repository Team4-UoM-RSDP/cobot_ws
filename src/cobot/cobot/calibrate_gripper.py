"""
calibrate_gripper.py

A ROS2 node that calibrates the Elephant Robotics adaptive gripper.
The gripper must be fully closed before running.

Author: Team 4
Date: 2026-04
"""

import time
from typing import List, Optional

import rclpy
from pymycobot import PI_BAUD, PI_PORT, MyCobot280
from rclpy.logging import get_logger
from rclpy.node import Node

_logger = get_logger(__name__)


class CalibrateGripper(Node):
    def __init__(self) -> None:
        super().__init__("cobot_execute")

        # Declare ROS2 parameters
        self.declare_parameter("port", PI_PORT)
        self.declare_parameter("baud", str(PI_BAUD))
        self.declare_parameter("gripper_speed", 10)

        # Get ROS2 parameters
        port: str = self.get_parameter("port").get_parameter_value().string_value
        baud: str = self.get_parameter("baud").get_parameter_value().string_value

        # Log configuration
        self.get_logger().info(f"Port: {port}, Baudrate: {baud}")

        # Initialise pymcobot
        self.mc = MyCobot280(port=port, baudrate=baud)
        time.sleep(0.05)
        if self.mc.get_fresh_mode() == 0:
            self.mc.set_fresh_mode(1)
            time.sleep(0.05)
        try:
            # Gripper must be closed. This sets current position to 0.
            self.mc.set_gripper_calibration()
            time.sleep(3)
            # Open the gripper
            self.mc.set_gripper_value(100, 10, 1, 0)
            time.sleep(3)
            # Initialise gripper
            self.mc.init_gripper()
            time.sleep(3)
            self.get_logger().info("Calibrated gripper.")
        except Exception as e:
            self.get_logger().error(f"Failed to calibrate gripper: {e}")


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the calibrate_gripper node."""
    try:
        rclpy.init(args=args)
        calibrate_gripper = CalibrateGripper()
        rclpy.spin(calibrate_gripper)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        _logger.error(f"Error spinning node: {e}")


if __name__ == "__main__":
    main()
