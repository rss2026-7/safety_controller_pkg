#!/usr/bin/env python3
"""Example node that drives the car forward at 1 m/s.

Run alongside the safety controller to demonstrate the mux architecture:

    Terminal 1:  ros2 run safety_controller_pkg safety_controller_pkg
    Terminal 2:  ros2 run safety_controller_pkg example_forward

The safety node will override this with a stop command whenever an
obstacle is detected, then release control once the obstacle clears.
"""
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


class ExampleForward(Node):
    def __init__(self):
        super().__init__('example_forward')
        self.SPEED = 1.0  # m/s

        self.nav_pub = self.create_publisher(
            AckermannDriveStamped, "/vesc/high_level/input/nav_0", 10
        )
        self.create_timer(0.1, self.drive_callback)  # 10 Hz

        self.get_logger().info(
            f'ExampleForward started — publishing {self.SPEED} m/s '
            'to /vesc/high_level/input/nav_0'
        )

    def drive_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.SPEED
        msg.drive.steering_angle = 0.0
        self.nav_pub.publish(msg)


def main():
    rclpy.init()
    node = ExampleForward()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
