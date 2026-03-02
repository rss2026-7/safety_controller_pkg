#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.SAFE_DISTANCE = 1
        
        self.breach_pub = self.create_publisher(Bool, "/safety_breach", 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # 1. Apply mask
        front_mask = (angles > -0.17) & (angles < 0.17) & (ranges > 0.1) & (np.isfinite(ranges))
        front_ranges = ranges[front_mask]

        # 2. Determine state (Default to False)
        collision_detected = False
        if len(front_ranges) > 0:
            if np.min(front_ranges) < self.SAFE_DISTANCE:
                collision_detected = True

        # 3. Create and publish the message ONCE
        breach_msg = Bool()
        breach_msg.data = collision_detected
        self.breach_pub.publish(breach_msg)

        # 4. Trigger the stop command if true
        if collision_detected:
            self.get_logger().warn(f"FRONT OBSTACLE DETECTED! Braking!")
            self.send_stop_command()

    def send_stop_command(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.drive_pub.publish(msg)

def main():
    rclpy.init()
    node = SafetyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
