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
  5. Graduated response zones: CLEAR / CAUTION / CRITICAL
  6. External override interface for perception integration (feature-flagged)

Safety only modifies SPEED. Steering is passed through unchanged from
the nav command on every published frame.
"""

import math
import os
import yaml

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import LaserScan, RegionOfInterest
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyZone:
    """Safety zone classification for graduated response."""
    CLEAR = 0      # No action; nav has full control
    CAUTION = 1    # Speed scaled down linearly
    CRITICAL = 2   # Already too close, hard brake


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
        self.momentum_thinness = self._req('cone.momentum_thinness')

        self.wheelbase       = self._req('vehicle.wheelbase')
        self.lidar_to_bumper = self._req('vehicle.lidar_to_bumper')

        self.brake_accel            = self._req('physics.brake_accel')
        self.pessimistic_accel      = self._req('physics.pessimistic_accel')
        self.reaction_time          = self._req('physics.reaction_time')
        self.safety_buffer          = self._req('physics.safety_buffer')
        self.wheel_moving_threshold = self._req('physics.wheel_moving_threshold')

        self.respect_external_stop    = self._req('features.respect_external_stop')
        self.respect_external_speed   = self._req('features.respect_external_speed')
        self.steering_aware_detection = self._req('features.steering_aware_detection')
        self.reverse_scanning         = self._req('features.reverse_scanning')

        self.caution_multiplier   = self._req('caution.distance_multiplier')
        self.caution_start_factor = self._req('caution.start_factor')
        self.caution_end_factor   = self._req('caution.end_factor')

        self.min_distance = self._req('critical.min_distance')
        self.brake_speed  = self._req('critical.brake_speed')

        self.person_critical_enabled  = self._req('person_critical.enabled')
        self.person_image_width       = self._req('person_critical.image_width')
        self.person_image_height      = self._req('person_critical.image_height')
        self.person_area_fraction     = self._req('person_critical.area_fraction')
        self.person_area_threshold_px = (
            self.person_image_width * self.person_image_height
            * self.person_area_fraction
        )

        self.stoplight_latch_sec = self._req('stoplight.latch_sec')

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
        # Latched red-light state: each "red" extends `_stoplight_red_until`
        # by `stoplight_latch_sec`; the safety decision reads the latch via
        # `_stoplight_red_active()`, never `self.stoplight_red` directly.
        # Without the latch a single intervening "none" between two
        # scan_callback ticks would un-stop the car at a real red light.
        self.stoplight_red = False
        self._stoplight_red_until = None
        self.person_too_close = False
        self.person_last_area_px = 0

        self.current_zone = SafetyZone.CLEAR
        self.closest_obstacle_dist = float('inf')
        self.closest_obstacle_angle = 0.0

        # ===== Publishers =====
        self.safety_pub = self.create_publisher(
            AckermannDriveStamped, "/vesc/low_level/input/safety", 10
        )
        self.status_pub = self.create_publisher(Float32, "/safety/status", 10)

        # ===== Subscribers =====
        # Run every callback in a ReentrantCallbackGroup so that a slow
        # perception callback (YOLO/HSV) can never starve scan_callback
        # — that starvation was the architectural reason brief "red"
        # detections sometimes failed to trigger a stop. Pair with the
        # MultiThreadedExecutor in main(). Sensor topics use sensor_data
        # QoS (BEST_EFFORT, depth 1) to match real publishers and avoid
        # silent drops behind a backed-up reliable queue.
        cb_group = ReentrantCallbackGroup()
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback,
            qos_profile_sensor_data, callback_group=cb_group)
        self.create_subscription(
            Odometry, "/odom", self.odom_callback,
            qos_profile_sensor_data, callback_group=cb_group)
        self.create_subscription(
            Odometry, "/vesc/odom", self.odom_callback,
            qos_profile_sensor_data, callback_group=cb_group)
        self.create_subscription(
            Bool, "/safety/external_stop", self.external_stop_callback,
            10, callback_group=cb_group)
        self.create_subscription(
            Float32, "/safety/max_speed", self.speed_limit_callback,
            10, callback_group=cb_group)
        self.create_subscription(
            AckermannDriveStamped, "/vesc/high_level/input/nav_0",
            self.nav_callback,
            qos_profile_sensor_data, callback_group=cb_group)
        self.create_subscription(
            String, "/stoplight/result", self.stoplight_callback,
            10, callback_group=cb_group)
        if self.person_critical_enabled:
            self.create_subscription(
                RegionOfInterest, "/yolo/person/roi", self.person_roi_callback,
                qos_profile_sensor_data, callback_group=cb_group,
            )

        # ===== Timers =====
        self.create_timer(0.1, self.check_laser_timeout)
        self.create_timer(0.05, self.publish_status)
        self.create_timer(1.0, self.log_wheel_speed)

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
        """Speed-dependent cone half-angle. Wide at v=0, narrow at high v.

        When the wheels are actually spinning (above wheel_moving_threshold)
        the result is multiplied by momentum_thinness so the deeper
        momentum-extended cone doesn't fan out wider at the same time.
        """
        speed = abs(speed)
        speed_ratio = min(1.0, speed / self.cone_shrink_speed)
        angle = self.cone_max_angle - speed_ratio * (self.cone_max_angle - self.cone_min_angle)
        if speed > self.wheel_moving_threshold:
            angle *= self.momentum_thinness
        return angle

    def compute_momentum_caution_distance(self, speed: float) -> float:
        """Wheel-speed-driven extension of the CAUTION cone depth.

        Uses pessimistic_accel (the deceleration the F1TENTH chassis
        actually achieves when the VESC is commanded to 0 — much smaller
        than the aspirational brake_accel) to bound how far ahead we
        need to look:

            d = v² / (2·pessimistic_accel) + v·reaction_time + safety_buffer

        Gated on wheel_moving_threshold: returns 0 when the wheels aren't
        spinning, so a parked car (or sim post-stop) doesn't extend the
        cone. The caller takes max() with the existing caution_dist, so
        this can only EXTEND the cone, never shrink it.
        """
        speed = abs(speed)
        if speed < self.wheel_moving_threshold:
            return 0.0
        return (speed ** 2) / (2 * self.pessimistic_accel) \
            + speed * self.reaction_time + self.safety_buffer

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

    def person_roi_callback(self, msg: RegionOfInterest):
        """Track whether YOLO sees a person filling a large area of the frame.

        yolo_node publishes every frame: width=0/height=0 means "not detected
        this frame", so a non-detection naturally clears the flag. Threshold
        is precomputed in pixels at startup from the configured image size.
        """
        area = int(msg.width) * int(msg.height)
        self.person_last_area_px = area
        was_close = self.person_too_close
        self.person_too_close = area >= self.person_area_threshold_px
        if self.person_too_close and not was_close:
            self.get_logger().error(
                f'\033[91;1m[PERSON] Large person bbox '
                f'({msg.width}×{msg.height}={area}px) ≥ '
                f'threshold {self.person_area_threshold_px:.0f}px — CRITICAL stop\033[0m'
            )

    def stoplight_callback(self, msg: String):
        """Latch on "red"; ignore "none" until the latch expires.

        Each fresh "red" extends `_stoplight_red_until` by latch_sec — so
        as long as red is detected at least once per latch window, the
        stop holds. The latch only clears when `_stoplight_red_active()`
        is checked AFTER expiry (in scan_callback), so a single dropped
        frame between two scan ticks can no longer un-stop the car.
        """
        if msg.data == "red":
            was_active = self._stoplight_red_active()
            self.stoplight_red = True
            self._stoplight_red_until = (
                self.get_clock().now()
                + Duration(seconds=self.stoplight_latch_sec)
            )
            if not was_active:
                self.get_logger().error(
                    '\033[91;1m[STOPLIGHT] RED — CRITICAL stop, '
                    f'latched for {self.stoplight_latch_sec:.1f}s\033[0m'
                )

    def _stoplight_red_active(self) -> bool:
        """True iff a "red" message arrived within the last latch window.
        Lazily clears the latch on expiry — there is no separate timer."""
        if not self.stoplight_red:
            return False
        if (self._stoplight_red_until is None
                or self.get_clock().now() >= self._stoplight_red_until):
            self.stoplight_red = False
            self._stoplight_red_until = None
            self.get_logger().info(
                '\033[92m[STOPLIGHT] latch expired — clearing\033[0m'
            )
            return False
        return True

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
            self.send_stop_command()
        elif self.laser_disconnected:
            self.laser_disconnected = False
            self.get_logger().info('\033[92mLaser reconnected\033[0m')

    def scan_callback(self, msg: LaserScan):
        """Main safety logic — process scan, classify zone, take action."""
        self.last_scan_time = self.get_clock().now()

        if self.external_stop_requested:
            self.current_zone = SafetyZone.CRITICAL
            self.send_stop_command()
            return

        if self._stoplight_red_active():
            self.current_zone = SafetyZone.CRITICAL
            self.send_stop_command(0.0)
            return

        if self.person_too_close:
            self.current_zone = SafetyZone.CRITICAL
            self.send_stop_command(0.0)
            return

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

        stopping_dist = self.compute_stopping_distance(self.current_speed)
        caution_dist = max(
            stopping_dist * self.caution_multiplier,
            self.compute_momentum_caution_distance(self.current_speed),
        )
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
            self.get_logger().error(
                f'\033[91;1m[CRITICAL] Obstacle at {min_actual_dist:.2f}m, bearing '
                f'{math.degrees(min_angle):+.0f}° (threshold {stopping_dist:.2f}m, '
                f'speed {self.current_speed:.2f} m/s) — STOP\033[0m'
            )
            self.send_stop_command()
            return

        # ─── CAUTION ─── speed-scale within caution range.
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

    def send_stop_command(self, speed: float = None):
        """Hard-brake command. Targets ``critical.brake_speed`` by default,
        or ``speed`` when an explicit override is supplied (used by the
        stoplight and person-too-close paths to force a clean 0 m/s).

        Steering is passed through unchanged from the latest nav command —
        safety only modifies speed.
        """
        stop = AckermannDriveStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = "base_link"
        stop.drive.speed = self.brake_speed if speed is None else speed
        stop.drive.acceleration = -self.brake_accel
        stop.drive.steering_angle = self.current_steering
        self.safety_pub.publish(stop)

    def send_speed_limit_command(self, max_speed: float):
        """Speed-cap command for graduated response.

        Steering is passed through unchanged from the latest nav command.
        """
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.drive.speed = max_speed
        cmd.drive.steering_angle = self.current_steering
        self.safety_pub.publish(cmd)

    def publish_status(self):
        status = Float32()
        status.data = float(self.current_zone)
        self.status_pub.publish(status)

    def log_wheel_speed(self):
        """1 Hz live readout of wheel-encoder speed and the resulting cone depth."""
        v = self.current_speed
        moving = abs(v) > self.wheel_moving_threshold
        momentum_d = self.compute_momentum_caution_distance(v)
        stopping_d = self.compute_stopping_distance(v)
        existing_d = stopping_d * self.caution_multiplier
        caution_d  = max(existing_d, momentum_d)
        marker = '●' if moving else '○'
        self.get_logger().info(
            f'\033[96m{marker} wheel_speed={v:+.2f} m/s | '
            f'cone={caution_d:.2f}m (existing={existing_d:.2f}m, '
            f'momentum={momentum_d:.2f}m) | '
            f'stopping={stopping_d:.2f}m\033[0m'
        )


def main():
    rclpy.init()
    node = SafetyNode()
    # MultiThreadedExecutor + ReentrantCallbackGroup lets scan_callback
    # run concurrently with the perception callbacks (stoplight, person,
    # external_stop) — single-threaded spin() serialised them, which is
    # what could delay a brake decision behind a slow perception update.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
