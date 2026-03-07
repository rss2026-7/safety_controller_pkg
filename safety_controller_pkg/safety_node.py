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
        self.SAFE_DISTANCE = 0.25  # meters
        self.CONE_HALF_ANGLE = 2.007  # 115 deg → 230 deg total
        self.LASER_TIMEOUT = 0.5  # seconds without a scan → laser disconnected

        self.last_scan_time = None
        self.laser_disconnected = False
        self.startup_time = self.get_clock().now()

        # Safety stop overrides navigation via the low-level mux
        self.safety_pub = self.create_publisher(
            AckermannDriveStamped, "/vesc/low_level/input/safety", 10
        )
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        # Periodically check whether laser messages are still arriving
        self.create_timer(0.1, self.check_laser_timeout)

        self.get_logger().info(
            'SafetyNode started — publishing stop commands to '
            '/vesc/low_level/input/safety when obstacles < '
            f'{self.SAFE_DISTANCE}m'
        )

    def check_laser_timeout(self):
        now = self.get_clock().now()
        if self.last_scan_time is None:
            elapsed = (now - self.startup_time).nanoseconds / 1e9
            if elapsed <= self.LASER_TIMEOUT:
                return  # grace period — still starting up
        else:
            elapsed = (now - self.last_scan_time).nanoseconds / 1e9
        if elapsed > self.LASER_TIMEOUT:
            if not self.laser_disconnected:
                self.laser_disconnected = True
                self.get_logger().error(
                    '\033[91mLASER DISCONNECTED — no /scan messages for '
                    f'{elapsed:.2f}s — STOPPING ROBOT\033[0m'
                )
            self.send_stop_command()
        elif self.laser_disconnected:
            self.laser_disconnected = False
            self.get_logger().info('\033[92mLaser reconnected\033[0m')

    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
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
                f'\033[93mOBSTACLE at {min_dist:.3f}m — braking!\033[0m'
            )
            self.send_stop_command()
        elif min_dist < self.SAFE_DISTANCE * 2:
            self.get_logger().info(
                f'\033[93mWall distance: {min_dist:.3f}m\033[0m'
            )
        else:
            self.get_logger().info(
                f'\033[97mWall distance: {min_dist:.3f}m\033[0m'
            )

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
