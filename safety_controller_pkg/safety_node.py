#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    """Pure safety override — monitors LIDAR and brakes when obstacles are too close.

    Publishes ONLY to /vesc/low_level/input/safety (stop commands).
    Does NOT set any drive speed — that is the job of your navigation code
    (wall follower, example_forward, etc.) on /vesc/high_level/input/nav_*.

    When no obstacle is detected, this node publishes nothing, so the
    low-level mux naturally passes through whatever navigation commands
    are active.
    """

    def __init__(self):
        super().__init__('safety_node')
        self.SAFE_DISTANCE = 0.4  # meters
        self.CONE_HALF_ANGLE = 2.007  # 115 deg → 230 deg total

        # Safety stop overrides navigation via the low-level mux
        self.safety_pub = self.create_publisher(
            AckermannDriveStamped, "/vesc/low_level/input/safety", 10
        )
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        self.get_logger().info(
            'SafetyNode started — publishing stop commands to '
            '/vesc/low_level/input/safety when obstacles < '
            f'{self.SAFE_DISTANCE}m'
        )

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        mask = (
            (np.abs(angles) < self.CONE_HALF_ANGLE)
            & (ranges > 0.1)
            & np.isfinite(ranges)
        )
        front_ranges = ranges[mask]

        if len(front_ranges) == 0:
            return

        min_dist = np.min(front_ranges)
        if min_dist < self.SAFE_DISTANCE:
            self.get_logger().warn(
                f'OBSTACLE at {min_dist:.3f}m — braking!'
            )
            self.send_stop_command()

    def send_stop_command(self):
        stop = AckermannDriveStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.drive.speed = 0.0
        stop.drive.steering_angle = 0.0
        self.safety_pub.publish(stop)


def main():
    rclpy.init()
    node = SafetyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
