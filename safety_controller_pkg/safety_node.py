#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.SAFE_DISTANCE = 0.4 # IN METERS
        self.CONE_HALF_ANGLE = 2.007  # 115 deg → 230 deg total
        self.DRIVE_SPEED = 1.0

        # Drive forward via high-level navigation mux
        self.nav_pub = self.create_publisher(AckermannDriveStamped, "/vesc/high_level/input/nav_0", 10)
        # Safety stop overrides navigation
        self.safety_pub = self.create_publisher(AckermannDriveStamped, "/vesc/low_level/input/safety", 10)
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        # Debug: monitor joystick to see RB state
        self.create_subscription(Joy, "/vesc/joy", self.joy_callback, 10)
        self._rb_held = False
        self._joy_received = False

        # 10Hz timer to drive forward
        self._drive_count = 0
        self.create_timer(0.1, self.drive_callback)

        self.get_logger().info('SafetyNode started — waiting for /vesc/joy and /scan...')

    def joy_callback(self, msg):
        if not self._joy_received:
            self.get_logger().info(f'[DEBUG] First joy msg received! buttons={list(msg.buttons)}, axes={[round(a,2) for a in msg.axes]}')
            self._joy_received = True

        # Logitech: RB = buttons[5], LB = buttons[4]
        rb = msg.buttons[5] if len(msg.buttons) > 5 else 0
        lb = msg.buttons[4] if len(msg.buttons) > 4 else 0

        if rb and not self._rb_held:
            self.get_logger().info('[DEBUG] RB PRESSED — autonomous mode active')
        elif not rb and self._rb_held:
            self.get_logger().info('[DEBUG] RB RELEASED — autonomous mode inactive')

        self._rb_held = bool(rb)

        # Also log LB for reference
        if lb:
            self.get_logger().info('[DEBUG] LB held (teleop mode — overrides everything)')

    def drive_callback(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = self.DRIVE_SPEED
        msg.drive.steering_angle = 0.0
        self.nav_pub.publish(msg)

        # Log every 2 seconds (every 20th call at 10Hz)
        self._drive_count += 1
        if self._drive_count % 20 == 0:
            self.get_logger().info(f'[DEBUG] Publishing drive: speed={self.DRIVE_SPEED} to /vesc/high_level/input/nav_0 (rb_held={self._rb_held})')

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # 1. Apply mask — 230 degree cone (±115 deg)
        mask = (np.abs(angles) < self.CONE_HALF_ANGLE) & (ranges > 0.1) & (np.isfinite(ranges))
        front_ranges = ranges[mask]

        # 2. Determine state (Default to False)
        collision_detected = False
        if len(front_ranges) > 0:
            min_dist = np.min(front_ranges)
            self.get_logger().info(f'[DEBUG] min_distance={min_dist:.3f}m (threshold={self.SAFE_DISTANCE}m)')
            if min_dist < self.SAFE_DISTANCE:
                collision_detected = True

        # 3. Trigger the stop command if obstacle detected
        if collision_detected:
            self.get_logger().warn(f"FRONT OBSTACLE DETECTED! Braking!")
            self.send_stop_command()

    def send_stop_command(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        self.safety_pub.publish(msg)

def main():
    rclpy.init()
    node = SafetyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
