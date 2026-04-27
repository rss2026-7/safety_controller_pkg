#!/usr/bin/env python3
"""
Advanced Safety Controller for MIT RACECAR

Designed for Final Challenge 2026:
  - Part A (Race): Velocity-dependent braking, narrow dynamic cone, lane-aware
  - Part B (Boating School): Graduated response, external overrides, reversing support

Key improvements over basic safety controller:
  1. Velocity-dependent stopping distance (physics-based)
  2. Dynamic cone angle that narrows at high speed
  3. Steering-aware obstacle weighting
  4. Direction-aware scanning (forward/reverse)
  5. Graduated response zones (DANGER/CAUTION/CLEAR)
  6. External override interface for perception integration
  7. Configurable modes (race vs city)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyZone:
    """Safety zone classification for graduated response."""
    CLEAR = 0      # No action needed, nav has full control
    CAUTION = 1    # Reduce speed proportionally
    DANGER = 2     # Emergency stop
    CRITICAL = 3   # Already too close, maximum braking


class SafetyNode(Node):
    """Advanced safety controller with velocity-aware braking and multiple modes.

    Modes:
        'race':  Narrow cone, high-speed optimized, ignores external overrides
        'city':  Wide cone, graduated response, listens to perception systems

    Topics Subscribed:
        /scan                    - LaserScan for obstacle detection
        /odom                    - Odometry for current velocity
        /vesc/sensors/core       - VESC telemetry (backup velocity source)
        /safety/external_stop    - Bool, external stop request (Part B)
        /safety/max_speed        - Float32, external speed limit (Part B)

    Topics Published:
        /vesc/low_level/input/safety  - AckermannDriveStamped stop/slow commands
        /safety/status                - Float32, current safety zone (for debugging)

    Parameters:
        mode           - 'race' or 'city' (default: 'city')
        brake_accel    - Braking deceleration in m/s² (default: 6.0)
        min_distance   - Minimum stopping distance in m (default: 0.20)
        reaction_time  - Reaction time margin in seconds (default: 0.05)
        max_cone_angle - Maximum half-cone angle in rad (default: 1.5, ~86°)
        min_cone_angle - Minimum half-cone angle in rad (default: 0.4, ~23°)
        caution_multiplier - Multiplier for caution zone (default: 2.0)
    """

    def __init__(self):
        super().__init__('safety_node')

        # ===== Declare and get parameters =====
        self.declare_parameter('mode', 'city')
        self.declare_parameter('brake_accel', 6.0)
        self.declare_parameter('min_distance', 0.35)        # CRITICAL threshold floor (m)
        self.declare_parameter('reaction_time', 0.05)
        self.declare_parameter('max_cone_angle', 1.5)
        self.declare_parameter('min_cone_angle', 0.4)
        self.declare_parameter('caution_multiplier', 2.857) # CAUTION = stopping × this (~1.0m at v=0)
        self.declare_parameter('brake_speed', 0.0)          # Target speed during CRITICAL hard stop
        self.declare_parameter('laser_timeout', 0.5)
        self.declare_parameter('max_speed', 4.0)
        self.declare_parameter('wheelbase', 0.325)
        # Distance from LIDAR origin to the front bumper. The launched MIT
        # racecar_simulator is a point car with no chassis or collision body,
        # and its lidar sits 0.275m forward of base_link — so for the sim
        # this offset is effectively zero. Override on the real car.
        self.declare_parameter('lidar_to_bumper', 0.0)
        self.declare_parameter('safety_buffer', 0.05)  # Margin beyond physics

        self.mode = self.get_parameter('mode').value
        self.brake_accel = self.get_parameter('brake_accel').value
        self.min_distance = self.get_parameter('min_distance').value
        self.reaction_time = self.get_parameter('reaction_time').value
        self.max_cone_angle = self.get_parameter('max_cone_angle').value
        self.min_cone_angle = self.get_parameter('min_cone_angle').value
        self.caution_multiplier = self.get_parameter('caution_multiplier').value
        self.laser_timeout = self.get_parameter('laser_timeout').value
        self.max_speed = self.get_parameter('max_speed').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.lidar_to_bumper = self.get_parameter('lidar_to_bumper').value
        self.safety_buffer = self.get_parameter('safety_buffer').value
        self.brake_speed = self.get_parameter('brake_speed').value

        # ===== State variables =====
        self.current_speed = 0.0          # m/s, from odometry
        self.current_steering = 0.0       # rad, from nav drive commands
        self.current_nav_speed = 0.0      # m/s, last commanded nav speed
        self.last_scan_time = None
        self.laser_disconnected = False
        self.startup_time = self.get_clock().now()

        # External override state (for Part B integration)
        self.external_stop_requested = False
        self.external_speed_limit = None  # None = no limit

        # Current safety state
        self.current_zone = SafetyZone.CLEAR
        self.closest_obstacle_dist = float('inf')
        self.closest_obstacle_angle = 0.0

        # ===== Publishers =====
        self.safety_pub = self.create_publisher(
            AckermannDriveStamped, "/vesc/low_level/input/safety", 10
        )
        self.status_pub = self.create_publisher(
            Float32, "/safety/status", 10
        )

        # ===== Subscribers =====
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Odometry, "/vesc/odom", self.odom_callback, 10)

        # External override topics (Part B integration)
        self.create_subscription(Bool, "/safety/external_stop", self.external_stop_callback, 10)
        self.create_subscription(Float32, "/safety/max_speed", self.speed_limit_callback, 10)

        # Also listen to drive commands to know current steering intent
        self.create_subscription(
            AckermannDriveStamped, "/vesc/high_level/input/nav_0",
            self.nav_callback, 10
        )

        # ===== Timers =====
        self.create_timer(0.1, self.check_laser_timeout)
        self.create_timer(0.05, self.publish_status)

        # ===== Logging =====
        self.get_logger().info(
            f'\033[96m╔══════════════════════════════════════════════════════════╗\033[0m'
        )
        self.get_logger().info(
            f'\033[96m║  SAFETY CONTROLLER v2.0 - Mode: {self.mode.upper():^6}                  ║\033[0m'
        )
        self.get_logger().info(
            f'\033[96m║  Brake accel: {self.brake_accel:.1f} m/s²  |  Min dist: {self.min_distance:.2f}m           ║\033[0m'
        )
        self.get_logger().info(
            f'\033[96m║  Cone: {math.degrees(self.min_cone_angle):.0f}°-{math.degrees(self.max_cone_angle):.0f}° (speed-dependent)              ║\033[0m'
        )
        self.get_logger().info(
            f'\033[96m╚══════════════════════════════════════════════════════════╝\033[0m'
        )

    # =========================================================================
    # PHYSICS CALCULATIONS
    # =========================================================================

    def compute_stopping_distance(self, speed: float) -> float:
        """Compute physics-based stopping distance.

        Formula: d = v² / (2a) + v * t_reaction + lidar_to_bumper + safety_buffer

        ``lidar_to_bumper`` is 0 for the launched MIT racecar_simulator (point
        car); set it nonzero on the real car to account for the chassis
        forward of the lidar.
        """
        speed = abs(speed)

        # Base: account for LIDAR-to-bumper offset + safety buffer
        base = self.lidar_to_bumper + self.safety_buffer

        if speed < 0.1:
            return max(self.min_distance, base)

        physics_dist = (speed ** 2) / (2 * self.brake_accel)
        reaction_dist = speed * self.reaction_time
        total = physics_dist + reaction_dist + base

        return max(self.min_distance, total)

    def compute_cone_angle(self, speed: float) -> float:
        """Compute dynamic cone half-angle based on speed.

        At low speed: wide cone for maneuvering
        At high speed: narrow cone for straight-line racing

        This prevents false triggers from cars in adjacent lanes on curves.
        """
        speed = abs(speed)

        # Linear interpolation based on speed
        # At 0 m/s: max_cone_angle (wide)
        # At max_speed: min_cone_angle (narrow)
        speed_ratio = min(1.0, speed / self.max_speed)
        angle = self.max_cone_angle - speed_ratio * (self.max_cone_angle - self.min_cone_angle)

        return angle

    def compute_steering_weight(self, obstacle_angle: float, steering: float) -> float:
        """Weight obstacle danger by how much it's in our projected path.

        If turning left (steering > 0), obstacles on the left are more dangerous.
        Obstacles on the right of our turn are less likely to be hit.

        Returns a weight 0.0 to 1.0 where 1.0 = directly in path.
        """
        if abs(steering) < 0.01:
            # Going straight - symmetric weighting
            return 1.0

        # Compute approximate turn radius and project path
        if abs(steering) > 0.001:
            turn_radius = self.wheelbase / math.tan(abs(steering))
        else:
            turn_radius = float('inf')

        # If steering left (positive), our path curves left
        # Obstacles at angles matching our turn direction are more dangerous
        turn_direction = 1.0 if steering > 0 else -1.0

        # Angle difference from our turn center
        # Obstacles in the direction we're turning get higher weight
        angle_alignment = obstacle_angle * turn_direction

        if angle_alignment > 0:
            # Obstacle is in the direction we're turning - higher danger
            weight = 1.0
        else:
            # Obstacle is opposite to turn direction - lower danger
            # But still dangerous if close to centerline
            weight = max(0.3, 1.0 - abs(obstacle_angle) / self.max_cone_angle)

        return weight

    def get_scan_angles(self, speed: float) -> tuple:
        """Get the forward cone half-angle for the current speed.

        Returns ``(-cone, +cone)``. Reverse-direction scanning is handled
        directly in ``scan_callback`` because a 270° lidar cannot see
        directly behind, so we instead use the rear-most rays available
        at the FOV edges.
        """
        cone_angle = self.compute_cone_angle(speed)
        return (-cone_angle, cone_angle)

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def odom_callback(self, msg: Odometry):
        """Update current velocity from odometry."""
        self.current_speed = msg.twist.twist.linear.x
        # Could also extract angular velocity if needed

    def nav_callback(self, msg: AckermannDriveStamped):
        """Track the commanded nav drive command (steering + speed)."""
        self.current_steering = msg.drive.steering_angle
        self.current_nav_speed = msg.drive.speed

    def external_stop_callback(self, msg: Bool):
        """Handle external stop requests (e.g., from traffic light detector)."""
        if self.mode == 'race':
            return  # Ignore external overrides in race mode

        self.external_stop_requested = msg.data
        if msg.data:
            self.get_logger().warn('\033[95mExternal stop requested\033[0m')

    def speed_limit_callback(self, msg: Float32):
        """Handle external speed limit (e.g., approaching intersection)."""
        if self.mode == 'race':
            return  # Ignore external overrides in race mode

        if msg.data < 0:
            self.external_speed_limit = None  # Clear limit
        else:
            self.external_speed_limit = msg.data
            self.get_logger().info(f'\033[95mExternal speed limit: {msg.data:.2f} m/s\033[0m')

    def check_laser_timeout(self):
        """Stop the robot if LIDAR data stops arriving."""
        now = self.get_clock().now()

        if self.last_scan_time is None:
            elapsed = (now - self.startup_time).nanoseconds / 1e9
            if elapsed <= self.laser_timeout:
                return  # Grace period at startup
        else:
            elapsed = (now - self.last_scan_time).nanoseconds / 1e9

        if elapsed > self.laser_timeout:
            if not self.laser_disconnected:
                self.laser_disconnected = True
                self.get_logger().error(
                    f'\033[91m╔═══════════════════════════════════════════╗\033[0m'
                )
                self.get_logger().error(
                    f'\033[91m║  LASER DISCONNECTED - EMERGENCY STOP      ║\033[0m'
                )
                self.get_logger().error(
                    f'\033[91m║  No /scan for {elapsed:.2f}s                       ║\033[0m'
                )
                self.get_logger().error(
                    f'\033[91m╚═══════════════════════════════════════════╝\033[0m'
                )
            self.current_zone = SafetyZone.CRITICAL
            self.send_stop_command()
        elif self.laser_disconnected:
            self.laser_disconnected = False
            self.get_logger().info('\033[92mLaser reconnected\033[0m')

    def scan_callback(self, msg: LaserScan):
        """Main safety logic - process LIDAR scan and take action."""
        self.last_scan_time = self.get_clock().now()

        # Handle external stop first
        if self.external_stop_requested:
            self.current_zone = SafetyZone.CRITICAL
            self.send_stop_command()
            return

        # Parse scan data
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        # Get dynamic parameters based on current state
        stopping_dist = self.compute_stopping_distance(self.current_speed)
        caution_dist = stopping_dist * self.caution_multiplier
        cone_angle = self.compute_cone_angle(self.current_speed)

        # Build angle mask based on direction of travel.
        if self.current_speed >= -0.1:  # Forward or stopped: scan front cone.
            angle_mask = (angles >= -cone_angle) & (angles <= cone_angle)
        else:
            # Reverse: the MIT lidar has a 270° FOV (±~2.355 rad), so it
            # cannot see directly behind. Use the rear-most rays available
            # — a cone of width ``cone_angle`` along each FOV edge.
            fov_edge = max(abs(msg.angle_min), abs(msg.angle_max))
            rear_threshold = max(0.0, fov_edge - cone_angle)
            angle_mask = np.abs(angles) >= rear_threshold

        # Filter valid readings
        valid_mask = (
            angle_mask &
            (ranges > 0.05) &  # Minimum range (avoid self-reflections)
            (ranges < msg.range_max) &
            np.isfinite(ranges)
        )

        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        if len(valid_ranges) == 0:
            self.current_zone = SafetyZone.CLEAR
            return

        # Apply steering-based weighting
        if self.mode == 'race' and abs(self.current_steering) > 0.05:
            # In race mode with active steering, weight obstacles by path
            weights = np.array([
                self.compute_steering_weight(a, self.current_steering)
                for a in valid_angles
            ])
            # Effective distance = actual distance / weight
            # Higher weight = obstacle more in path = effectively closer
            effective_ranges = valid_ranges / np.maximum(weights, 0.1)
        else:
            effective_ranges = valid_ranges

        # Find closest obstacle
        min_idx = np.argmin(effective_ranges)
        min_effective_dist = effective_ranges[min_idx]
        min_actual_dist = valid_ranges[min_idx]
        min_angle = valid_angles[min_idx]

        self.closest_obstacle_dist = min_actual_dist
        self.closest_obstacle_angle = min_angle

        # CRITICAL: inside the near edge of the CAUTION zone — hold a hard
        # stop. Never autonomously reverse, otherwise we ping-pong
        # (forward → reverse → cone flips → forward).
        if min_actual_dist < stopping_dist:
            self.current_zone = SafetyZone.CRITICAL
            self.get_logger().error(
                f'\033[91;1m[CRITICAL] Obstacle at {min_actual_dist:.2f}m '
                f'(threshold {stopping_dist:.2f}m, speed {self.current_speed:.2f} m/s) — HOLDING STOP\033[0m'
            )
            self.send_stop_command()

        # CAUTION: scale the nav-commanded speed by a distance-based factor.
        # At the far edge of CAUTION (progress=0): factor = 1.0 → pass-through.
        # At the near edge       (progress=1):     factor = 0.1 → 10% of nav.
        # We scale the nav speed (not max_speed) so we never overdrive what
        # the user actually asked for. Always publish — otherwise the mux's
        # 0.5s safety_timeout releases nav and the car re-accelerates.
        elif min_effective_dist < caution_dist:
            self.current_zone = SafetyZone.CAUTION
            span = max(1e-6, caution_dist - stopping_dist)
            progress = (caution_dist - min_effective_dist) / span
            progress = max(0.0, min(1.0, progress))
            factor = 1.0 - 0.9 * progress  # 1.0 → 0.1
            max_allowed_speed = self.current_nav_speed * factor

            if self.external_speed_limit is not None:
                max_allowed_speed = min(max_allowed_speed, self.external_speed_limit)

            if self.current_nav_speed > max_allowed_speed + 0.05:
                self.get_logger().info(
                    f'\033[93m[CAUTION] Obstacle at {min_actual_dist:.2f}m — '
                    f'limiting {self.current_nav_speed:.2f}→{max_allowed_speed:.2f} m/s\033[0m'
                )
            self.send_speed_limit_command(max_allowed_speed)

        else:
            self.current_zone = SafetyZone.CLEAR
            # Apply external speed limit even in clear zone
            if self.external_speed_limit is not None and self.current_speed > self.external_speed_limit:
                self.send_speed_limit_command(self.external_speed_limit)
            # Otherwise, don't publish - let nav commands through

    # =========================================================================
    # COMMAND PUBLISHING
    # =========================================================================

    def send_stop_command(self):
        """Send a hard-brake command.

        Targets ``brake_speed`` (slightly negative by default) so the motor
        actively brakes instead of being commanded to coast to 0. The value
        is small enough that the car does not meaningfully reverse.
        """
        stop = AckermannDriveStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = "base_link"
        stop.drive.speed = self.brake_speed
        stop.drive.acceleration = -self.brake_accel
        stop.drive.steering_angle = self.current_steering
        self.safety_pub.publish(stop)

    def send_speed_limit_command(self, max_speed: float):
        """Send speed limit command for graduated response."""
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = max_speed
        cmd.drive.steering_angle = self.current_steering  # Maintain steering
        self.safety_pub.publish(cmd)

    def publish_status(self):
        """Publish current safety status for debugging/visualization."""
        status = Float32()
        status.data = float(self.current_zone)
        self.status_pub.publish(status)


# =============================================================================
# LEGACY COMPATIBILITY
# =============================================================================
# These class attributes allow the basic_sim to read parameters without
# instantiating parameters (since the mock doesn't support declare_parameter)

SafetyNode.SAFE_DISTANCE = 0.20  # Legacy: will be overridden by compute_stopping_distance
SafetyNode.CONE_HALF_ANGLE = 1.5  # Legacy: will be overridden by compute_cone_angle


def main():
    rclpy.init()
    node = SafetyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
