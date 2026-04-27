#!/usr/bin/env python3
"""
Advanced Safety Controller for MIT RACECAR

All tunables live in a single YAML config (``config/safety.yaml``) — there
are NO code-side defaults. The node refuses to start if any required key
is missing.

Key features:
  1. Velocity-dependent stopping distance (physics-based)
  2. Dynamic cone angle that narrows at high speed
  3. Steering-aware obstacle weighting (feature-flagged)
  4. Direction-aware scanning (forward/reverse, feature-flagged)
  5. Graduated response zones: CLEAR / CAUTION / AVOIDANCE / CRITICAL
  6. External override interface for perception integration (feature-flagged)
"""

import math
import os
import yaml

import rclpy
from rclpy.node import Node
import numpy as np
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyZone:
    """Safety zone classification for graduated response."""
    CLEAR = 0      # No action; nav has full control
    CAUTION = 1    # Speed scaled down linearly
    AVOIDANCE = 2  # Speed-limited and actively steering away
    CRITICAL = 3   # Already too close, hard brake


class SafetyNode(Node):
    """Advanced safety controller. All behavior controlled by YAML config.

    Topics Subscribed:
        /scan                    - LaserScan for obstacle detection
        /odom, /vesc/odom        - Odometry for current velocity
        /vesc/high_level/input/nav_0 - Nav drive commands (steering intent)
        /safety/external_stop    - Bool, external stop request
        /safety/max_speed        - Float32, external speed limit

    Topics Published:
        /vesc/low_level/input/safety - AckermannDriveStamped stop/limit
        /safety/status               - Float32, current SafetyZone value
    """

    def __init__(self):
        super().__init__('safety_node')

        # ===== Load the bundled YAML config (the ground truth) =====
        # We load it directly so the package works with `ros2 run` as well
        # as `ros2 launch`. ROS-level overrides (via `--params-file` or
        # `-p key:=value`) still win — declare_parameter uses the YAML
        # value as the *default*, and rclpy applies any override on top.
        self._cfg = self._load_bundled_config()

        # ===== Read every parameter, screaming on any missing key =====
        self.cone_max_angle    = self._req('cone.max_angle')
        self.cone_min_angle    = self._req('cone.min_angle')
        self.cone_shrink_speed = self._req('cone.shrink_speed')

        self.wheelbase       = self._req('vehicle.wheelbase')
        self.lidar_to_bumper = self._req('vehicle.lidar_to_bumper')

        self.brake_accel   = self._req('physics.brake_accel')
        self.reaction_time = self._req('physics.reaction_time')
        self.safety_buffer = self._req('physics.safety_buffer')

        self.respect_external_stop    = self._req('features.respect_external_stop')
        self.respect_external_speed   = self._req('features.respect_external_speed')
        self.steering_aware_detection = self._req('features.steering_aware_detection')
        self.reverse_scanning         = self._req('features.reverse_scanning')

        self.caution_multiplier   = self._req('caution.distance_multiplier')
        self.caution_start_factor = self._req('caution.start_factor')
        self.caution_end_factor   = self._req('caution.end_factor')
        self.caution_escalate     = self._req('caution.escalate_below_factor')

        self.avoidance_enabled       = self._req('avoidance.enabled')
        self.avoidance_max_steering  = self._req('avoidance.max_steering_angle')
        head_on = self._req('avoidance.head_on_direction')
        if head_on not in ('left', 'right'):
            raise ValueError(
                f"avoidance.head_on_direction must be 'left' or 'right', got {head_on!r}"
            )
        # Steering convention: positive = left turn.
        self.head_on_sign = 1.0 if head_on == 'left' else -1.0

        self.min_distance           = self._req('critical.min_distance')
        self.brake_speed            = self._req('critical.brake_speed')
        self.critical_steer_away    = self._req('critical.steer_away')
        self.critical_max_steering  = self._req('critical.max_steering_angle')
        self.brake_p_gain           = self._req('critical.brake_p_gain')
        self.stop_speed_threshold   = self._req('critical.stop_speed_threshold')
        self.brake_max_command      = self._req('critical.brake_max_command')

        self.laser_timeout = self._req('laser_timeout')

        # ===== State variables =====
        self.current_speed = 0.0
        self.current_steering = 0.0
        self.current_nav_speed = 0.0
        self.last_scan_time = None
        self.laser_disconnected = False
        self.startup_time = self.get_clock().now()

        self.external_stop_requested = False
        self.external_speed_limit = None  # None = no limit

        self.current_zone = SafetyZone.CLEAR
        self.closest_obstacle_dist = float('inf')
        self.closest_obstacle_angle = 0.0

        # ===== Publishers =====
        self.safety_pub = self.create_publisher(
            AckermannDriveStamped, "/vesc/low_level/input/safety", 10
        )
        self.status_pub = self.create_publisher(Float32, "/safety/status", 10)

        # ===== Subscribers =====
        self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Odometry, "/vesc/odom", self.odom_callback, 10)
        self.create_subscription(Bool, "/safety/external_stop", self.external_stop_callback, 10)
        self.create_subscription(Float32, "/safety/max_speed", self.speed_limit_callback, 10)
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
            f'\033[96m║  SAFETY CONTROLLER — config-driven                       ║\033[0m'
        )
        self.get_logger().info(
            f'\033[96m║  Brake accel: {self.brake_accel:.1f} m/s²  |  Min dist: {self.min_distance:.2f}m           ║\033[0m'
        )
        self.get_logger().info(
            f'\033[96m║  Cone: {math.degrees(self.cone_min_angle):.0f}°-{math.degrees(self.cone_max_angle):.0f}° (speed-dependent)              ║\033[0m'
        )
        self.get_logger().info(
            f'\033[96m╚══════════════════════════════════════════════════════════╝\033[0m'
        )

    # =========================================================================
    # PARAMETER ACCESS
    # =========================================================================

    def _load_bundled_config(self) -> dict:
        """Load the YAML config installed alongside the package.

        Returns the dict under ``safety_node.ros__parameters``. Raises with a
        clear message if the file is missing, malformed, or has the wrong
        top-level shape.
        """
        path = os.path.join(
            get_package_share_directory('safety_controller_pkg'),
            'config',
            'safety.yaml',
        )
        self.get_logger().info(f'\033[96mLoading safety config: {path}\033[0m')
        try:
            with open(path) as f:
                doc = yaml.safe_load(f)
        except FileNotFoundError:
            raise RuntimeError(
                f"Bundled safety config not found at {path}. "
                f"Did `colcon build` install the config/ directory? "
                f"Check setup.py data_files."
            )
        except yaml.YAMLError as e:
            raise RuntimeError(f"safety.yaml is malformed: {e}")

        params = (doc or {}).get('safety_node', {}).get('ros__parameters')
        if not isinstance(params, dict):
            raise RuntimeError(
                f"safety.yaml at {path} is missing the "
                f"'safety_node:\\n  ros__parameters:' top-level structure."
            )
        return params

    def _req(self, name: str):
        """Read a required parameter from the YAML, screaming if missing.

        The YAML is the ground truth — there are no code-side defaults.
        The value is also declared as a ROS parameter, so callers can
        introspect or override it via `--params-file` / `-p key:=value`.
        """
        parts = name.split('.')
        node = self._cfg
        for part in parts:
            if not isinstance(node, dict) or part not in node:
                raise RuntimeError(
                    f"Missing required safety-controller key '{name}' in "
                    f"safety.yaml. Every key under safety_node.ros__parameters "
                    f"must be present; there are no code-side defaults."
                )
            node = node[part]
        yaml_value = node

        # Declare so rclpy's override mechanism still applies cleanly.
        if not self.has_parameter(name):
            self.declare_parameter(name, yaml_value)
        return self.get_parameter(name).value

    # =========================================================================
    # PHYSICS CALCULATIONS
    # =========================================================================

    def compute_stopping_distance(self, speed: float) -> float:
        """Compute physics-based stopping distance.

        d = v² / (2a) + v · t_reaction + lidar_to_bumper + safety_buffer
        Floored at ``critical.min_distance``.
        """
        speed = abs(speed)
        base = self.lidar_to_bumper + self.safety_buffer

        if speed < 0.1:
            return max(self.min_distance, base)

        physics_dist = (speed ** 2) / (2 * self.brake_accel)
        reaction_dist = speed * self.reaction_time
        total = physics_dist + reaction_dist + base
        return max(self.min_distance, total)

    def compute_cone_angle(self, speed: float) -> float:
        """Speed-dependent cone half-angle. Wide at v=0, narrow at high v."""
        speed = abs(speed)
        speed_ratio = min(1.0, speed / self.cone_shrink_speed)
        return self.cone_max_angle - speed_ratio * (self.cone_max_angle - self.cone_min_angle)

    def compute_steering_weight(self, obstacle_angle: float, steering: float) -> float:
        """Weight obstacle danger by how much it's in our projected path."""
        if abs(steering) < 0.01:
            return 1.0

        turn_direction = 1.0 if steering > 0 else -1.0
        angle_alignment = obstacle_angle * turn_direction

        if angle_alignment > 0:
            return 1.0
        return max(0.3, 1.0 - abs(obstacle_angle) / self.cone_max_angle)

    def get_scan_angles(self, speed: float) -> tuple:
        """Forward cone half-angle for current speed; ``(-cone, +cone)``."""
        cone_angle = self.compute_cone_angle(speed)
        return (-cone_angle, cone_angle)

    # =========================================================================
    # CALLBACKS
    # =========================================================================

    def odom_callback(self, msg: Odometry):
        self.current_speed = msg.twist.twist.linear.x

    def nav_callback(self, msg: AckermannDriveStamped):
        self.current_steering = msg.drive.steering_angle
        self.current_nav_speed = msg.drive.speed

    def external_stop_callback(self, msg: Bool):
        if not self.respect_external_stop:
            return
        self.external_stop_requested = msg.data
        if msg.data:
            self.get_logger().warn('\033[95mExternal stop requested\033[0m')

    def speed_limit_callback(self, msg: Float32):
        if not self.respect_external_speed:
            return
        if msg.data < 0:
            self.external_speed_limit = None
        else:
            self.external_speed_limit = msg.data
            self.get_logger().info(f'\033[95mExternal speed limit: {msg.data:.2f} m/s\033[0m')

    def check_laser_timeout(self):
        """Emergency stop if /scan stops arriving."""
        now = self.get_clock().now()

        if self.last_scan_time is None:
            elapsed = (now - self.startup_time).nanoseconds / 1e9
            if elapsed <= self.laser_timeout:
                return
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
            self.stop()
        elif self.laser_disconnected:
            self.laser_disconnected = False
            self.get_logger().info('\033[92mLaser reconnected\033[0m')

    def scan_callback(self, msg: LaserScan):
        """Main safety logic — process scan, classify zone, take action."""
        self.last_scan_time = self.get_clock().now()

        if self.external_stop_requested:
            self.current_zone = SafetyZone.CRITICAL
            self.stop()
            return

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        stopping_dist = self.compute_stopping_distance(self.current_speed)
        caution_dist = stopping_dist * self.caution_multiplier
        cone_angle = self.compute_cone_angle(self.current_speed)

        # Build angle mask. In reverse with reverse_scanning enabled, use
        # the rear-most rays available (270° lidar can't see directly behind).
        if self.current_speed >= -0.1 or not self.reverse_scanning:
            angle_mask = (angles >= -cone_angle) & (angles <= cone_angle)
        else:
            fov_edge = max(abs(msg.angle_min), abs(msg.angle_max))
            rear_threshold = max(0.0, fov_edge - cone_angle)
            angle_mask = np.abs(angles) >= rear_threshold

        valid_mask = (
            angle_mask &
            (ranges > 0.05) &
            (ranges < msg.range_max) &
            np.isfinite(ranges)
        )
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]

        if len(valid_ranges) == 0:
            self.current_zone = SafetyZone.CLEAR
            return

        # Steering-aware weighting (path projection).
        if self.steering_aware_detection and abs(self.current_steering) > 0.05:
            weights = np.array([
                self.compute_steering_weight(a, self.current_steering)
                for a in valid_angles
            ])
            effective_ranges = valid_ranges / np.maximum(weights, 0.1)
        else:
            effective_ranges = valid_ranges

        min_idx = np.argmin(effective_ranges)
        min_effective_dist = effective_ranges[min_idx]
        min_actual_dist = valid_ranges[min_idx]
        min_angle = valid_angles[min_idx]

        self.closest_obstacle_dist = min_actual_dist
        self.closest_obstacle_angle = min_angle

        # ─── CRITICAL ─── inside stopping distance: hard brake.
        if min_actual_dist < stopping_dist:
            self.current_zone = SafetyZone.CRITICAL
            if self.critical_steer_away:
                avoidance = self.compute_avoidance_steering(self.critical_max_steering)
                self.get_logger().error(
                    f'\033[91;1m[CRITICAL] Obstacle at {min_actual_dist:.2f}m, bearing '
                    f'{math.degrees(min_angle):+.0f}° (threshold {stopping_dist:.2f}m, '
                    f'speed {self.current_speed:.2f} m/s) — STOP, steer {math.degrees(avoidance):+.0f}°\033[0m'
                )
                self.stop(avoidance)
            else:
                self.get_logger().error(
                    f'\033[91;1m[CRITICAL] Obstacle at {min_actual_dist:.2f}m, bearing '
                    f'{math.degrees(min_angle):+.0f}° (threshold {stopping_dist:.2f}m, '
                    f'speed {self.current_speed:.2f} m/s) — STOP\033[0m'
                )
                self.stop()
            return

        # ─── CAUTION / AVOIDANCE ─── speed-scale within caution range.
        if min_effective_dist < caution_dist:
            span = max(1e-6, caution_dist - stopping_dist)
            progress = (caution_dist - min_effective_dist) / span
            progress = max(0.0, min(1.0, progress))
            factor = self.caution_start_factor - (
                self.caution_start_factor - self.caution_end_factor
            ) * progress
            max_allowed_speed = self.current_nav_speed * factor

            if self.external_speed_limit is not None:
                max_allowed_speed = min(max_allowed_speed, self.external_speed_limit)

            if factor < self.caution_escalate and self.avoidance_enabled:
                self.current_zone = SafetyZone.AVOIDANCE
                avoidance = self.compute_avoidance_steering(self.avoidance_max_steering)
                self.get_logger().warn(
                    f'\033[95m[AVOIDANCE] Obstacle at {min_actual_dist:.2f}m, bearing '
                    f'{math.degrees(min_angle):+.0f}° — limiting to {max_allowed_speed:.2f} m/s, '
                    f'steer {math.degrees(avoidance):+.0f}°\033[0m'
                )
                self.send_speed_limit_command(max_allowed_speed, avoidance)
            else:
                self.current_zone = SafetyZone.CAUTION
                if self.current_nav_speed > max_allowed_speed + 0.05:
                    self.get_logger().info(
                        f'\033[93m[CAUTION] Obstacle at {min_actual_dist:.2f}m — '
                        f'limiting {self.current_nav_speed:.2f}→{max_allowed_speed:.2f} m/s\033[0m'
                    )
                self.send_speed_limit_command(max_allowed_speed)
            return

        # ─── CLEAR ─── nothing in range.
        self.current_zone = SafetyZone.CLEAR
        if (self.external_speed_limit is not None
                and self.current_speed > self.external_speed_limit):
            self.send_speed_limit_command(self.external_speed_limit)

    # =========================================================================
    # COMMAND PUBLISHING
    # =========================================================================

    def compute_avoidance_steering(self, max_angle: float) -> float:
        """Steering angle that nudges the car away from the closest obstacle.

        ``max_angle`` is the cap (radians); the sign is opposite the
        obstacle's bearing so the front turns away. For dead-center
        obstacles the configured ``head_on_direction`` breaks the tie. In
        reverse the rear-edge rays don't map cleanly to a steer direction,
        so we return the held nav steering unchanged.
        """
        if self.current_speed < -0.1:
            return self.current_steering

        angle = self.closest_obstacle_angle
        if abs(angle) < 0.05:
            direction = self.head_on_sign
        else:
            direction = -1.0 if angle > 0 else 1.0
        return direction * max_angle

    def stop(self, steering: float | None = None):
        """Momentum-aware brake command.

        While the measured wheel speed is above ``stop_speed_threshold`` we
        command the VESC opposite the direction of motion, magnitude
        proportional to current_speed (P-loop), clamped to
        ``brake_max_command``. Once the wheels are below threshold we settle
        onto ``brake_speed``. The sim has no momentum so ``current_speed``
        collapses to ~0 immediately and this becomes a plain stop; the real
        car coasts and needs the active reverse drive to actually halt.
        """
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        if abs(self.current_speed) < self.stop_speed_threshold:
            target_speed = self.brake_speed
        else:
            target_speed = -self.brake_p_gain * self.current_speed
            cap = self.brake_max_command
            target_speed = max(-cap, min(cap, target_speed))

        cmd.drive.speed = target_speed
        cmd.drive.acceleration = -self.brake_accel
        cmd.drive.steering_angle = self.current_steering if steering is None else steering
        self.safety_pub.publish(cmd)

    def send_speed_limit_command(self, max_speed: float, steering: float | None = None):
        """Speed-cap command for graduated response."""
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = max_speed
        cmd.drive.steering_angle = self.current_steering if steering is None else steering
        self.safety_pub.publish(cmd)

    def publish_status(self):
        status = Float32()
        status.data = float(self.current_zone)
        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = SafetyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
