"""Run the 2D demo simulator.

Usage:
    python3 -m safety_controller_pkg.basic_sim.run_sim
    # or, from inside basic_sim/
    python3 run_sim.py

What happens on startup:
  1. `ros_mock.install()` registers fake `rclpy`, `sensor_msgs`, `nav_msgs`,
     `std_msgs`, and `ackermann_msgs` modules in sys.modules.
  2. We then `import safety_controller_pkg.safety_node` — the REAL safety
     node code, untouched. Because our mock is API-compatible, it sees
     what it expects.
  3. We construct a `SafetyNode` instance. It subscribes to "/scan", "/odom",
     external override topics, and publishes to "/vesc/low_level/input/safety".
  4. The main loop:
       * reads keyboard (WASD),
       * drives the car physics forward,
       * ray-traces a fake lidar scan,
       * publishes odometry (current speed) on "/odom",
       * publishes the scan on "/scan" — which synchronously fires the safety
         node's callback,
       * if the safety node published a stop on the safety topic since
         last tick, the main loop overrides the car's commanded speed
         (modelling the VESC low-level mux).

New in v2.0:
  * Publishes odometry for velocity-dependent braking
  * Shows current safety zone on HUD
  * Key 'E' toggles external stop (simulates traffic light)
  * Key 'Q' cycles speed limit (simulates approaching intersection)
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import pygame

# ---- Install ROS mocks BEFORE importing the real safety node ----
# This is the critical step. The mock has to be in sys.modules before
# safety_node.py's `import rclpy` statement runs.
from . import ros_mock  # noqa: E402
ros_mock.install()

# Now it is safe to pull in the real safety controller code, unchanged.
from safety_controller_pkg.safety_node import SafetyNode, SafetyZone  # noqa: E402
import rclpy  # noqa: E402  (this is actually our mock)

from .car_physics import CarPhysics, CarParams  # noqa: E402
from .lidar import Lidar, LidarParams  # noqa: E402
from .renderer import Renderer  # noqa: E402
from .world import build_demo_world, load_png_map  # noqa: E402


SCAN_TOPIC = "/scan"
ODOM_TOPIC = "/odom"
SAFETY_TOPIC = "/vesc/low_level/input/safety"
NAV_TOPIC = "/vesc/high_level/input/nav_0"
EXTERNAL_STOP_TOPIC = "/safety/external_stop"
SPEED_LIMIT_TOPIC = "/safety/max_speed"


def build_laserscan(lidar_params: LidarParams, ranges: np.ndarray):
    """Construct a mock LaserScan the way the real racecar simulator does."""
    from sensor_msgs.msg import LaserScan  # our mock
    msg = LaserScan()
    msg.angle_min = lidar_params.angle_min
    msg.angle_max = lidar_params.angle_max
    msg.angle_increment = lidar_params.angle_increment
    msg.range_min = lidar_params.range_min
    msg.range_max = lidar_params.range_max
    msg.ranges = list(ranges.astype(float))
    return msg


def build_odometry(speed: float, steering: float):
    """Construct a mock Odometry message with current velocity."""
    from nav_msgs.msg import Odometry  # our mock
    msg = Odometry()
    msg.twist.twist.linear.x = speed
    msg.twist.twist.angular.z = steering
    return msg


def build_nav_cmd(speed: float, steering: float):
    """Construct a mock nav command (for steering tracking)."""
    from ackermann_msgs.msg import AckermannDriveStamped  # our mock
    msg = AckermannDriveStamped()
    msg.drive.speed = speed
    msg.drive.steering_angle = steering
    return msg


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Safety controller 2D demo sim.")
    p.add_argument("--map", default=None, help="Optional PNG occupancy map.")
    p.add_argument("--resolution", type=float, default=0.05, help="meters/cell for PNG maps.")
    p.add_argument("--no-noise", action="store_true", help="Disable lidar noise.")
    p.add_argument(
        "--scan-hz", type=float, default=40.0,
        help="Rate at which fake lidar scans are published (default 40 Hz).",
    )
    p.add_argument(
        "--mode", type=str, default="city", choices=["city", "race"],
        help="Safety controller mode (default: city).",
    )
    # Autonomous test mode
    p.add_argument(
        "--test", action="store_true",
        help="Run autonomous collision test (full throttle toward wall).",
    )
    p.add_argument(
        "--test-duration", type=float, default=10.0,
        help="Duration of autonomous test in seconds (default: 10).",
    )
    p.add_argument(
        "--start-x", type=float, default=None,
        help="Starting X position (default: 2.0, wall at ~4.0).",
    )
    p.add_argument(
        "--start-y", type=float, default=None,
        help="Starting Y position (default: 6.0).",
    )
    p.add_argument(
        "--min-gap", type=float, default=0.10,
        help="Minimum allowed gap to wall in meters (default: 0.10). Test fails if closer.",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()

    # ------------- world / car / lidar setup -----------------------------
    if args.map:
        world = load_png_map(args.map, resolution=args.resolution)
        start_x, start_y = world.world_width * 0.15, world.world_height * 0.5
    else:
        world = build_demo_world()
        start_x, start_y = 2.0, 6.0

    # Override start position if specified
    if args.start_x is not None:
        start_x = args.start_x
    if args.start_y is not None:
        start_y = args.start_y

    physics = CarPhysics(CarParams())
    physics.reset(start_x, start_y, 0.0)

    # Test mode state
    test_mode = args.test
    test_duration = args.test_duration
    test_min_gap = args.min_gap
    test_collision = False
    test_min_distance = float('inf')
    test_max_speed = 0.0

    lidar_params = LidarParams()
    if args.no_noise:
        lidar_params.std_dev = 0.0
    lidar = Lidar(lidar_params)

    renderer = Renderer()
    renderer.camera.cx = start_x
    renderer.camera.cy = start_y

    # ------------- ROS: spin up the real safety node ---------------------
    rclpy.init()
    safety_node = SafetyNode()

    # Override mode parameter if specified via command line
    safety_node.mode = args.mode
    safety_node._parameters['mode'] = args.mode

    # Create publishers for sim -> safety node communication
    scan_pub = safety_node.create_publisher(
        type(build_laserscan(lidar_params, np.array([1.0]))),
        SCAN_TOPIC,
        10,
    )
    odom_pub = safety_node.create_publisher(
        type(build_odometry(0.0, 0.0)),
        ODOM_TOPIC,
        10,
    )
    nav_pub = safety_node.create_publisher(
        type(build_nav_cmd(0.0, 0.0)),
        NAV_TOPIC,
        10,
    )

    # Publishers for external override simulation
    from std_msgs.msg import Bool, Float32
    external_stop_pub = safety_node.create_publisher(Bool, EXTERNAL_STOP_TOPIC, 10)
    speed_limit_pub = safety_node.create_publisher(Float32, SPEED_LIMIT_TOPIC, 10)

    # ------------- main loop ---------------------------------------------
    clock = pygame.time.Clock()
    sim_time = 0.0
    scan_period = 1.0 / args.scan_hz
    next_scan = 0.0
    show_rays = True
    running = True
    fps_display = 0.0

    # External override state (for testing Part B features)
    external_stop_active = False
    speed_limit_values = [None, 2.0, 1.0, 0.5]  # Cycle through these
    speed_limit_idx = 0

    print("\n" + "="*60)
    if test_mode:
        print("SAFETY CONTROLLER - AUTONOMOUS TEST MODE")
        print("="*60)
        print(f"Start position: ({start_x:.1f}, {start_y:.1f})")
        print(f"Test duration:  {test_duration:.1f} seconds")
        print(f"Min gap requirement: {test_min_gap:.2f}m")
        print("Action: FULL THROTTLE FORWARD (no user input)")
        print("Press Esc to abort")
    else:
        print("SAFETY CONTROLLER v2.0 SIMULATOR")
        print("="*60)
        print("Controls:")
        print("  WASD   - Drive")
        print("  Space  - Handbrake")
        print("  R      - Reset position")
        print("  M      - Toggle lidar rays")
        print("  E      - Toggle external stop (traffic light)")
        print("  Q      - Cycle speed limit (intersection approach)")
        print("  Esc    - Quit")
    print("="*60 + "\n")

    while running:
        # --- events + keys
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    physics.reset(start_x, start_y, 0.0)
                elif event.key == pygame.K_m:
                    show_rays = not show_rays
                elif event.key == pygame.K_e:
                    # Toggle external stop (simulates traffic light)
                    external_stop_active = not external_stop_active
                    msg = Bool()
                    msg.data = external_stop_active
                    external_stop_pub.publish(msg)
                    print(f"External stop: {'ON' if external_stop_active else 'OFF'}")
                elif event.key == pygame.K_q:
                    # Cycle speed limit
                    speed_limit_idx = (speed_limit_idx + 1) % len(speed_limit_values)
                    limit = speed_limit_values[speed_limit_idx]
                    msg = Float32()
                    msg.data = limit if limit is not None else -1.0
                    speed_limit_pub.publish(msg)
                    print(f"Speed limit: {limit if limit else 'None'}")

        keys = pygame.key.get_pressed()
        throttle = 0.0
        steer = 0.0
        handbrake = False

        if test_mode:
            # Autonomous test: full throttle forward, no steering, no handbrake
            throttle = 1.0
            steer = 0.0
            handbrake = False
        else:
            # Manual control
            if keys[pygame.K_w]:
                throttle += 1.0
            if keys[pygame.K_s]:
                throttle -= 1.0
            if keys[pygame.K_a]:
                steer += 1.0  # positive steer = left (positive theta in ROS)
            if keys[pygame.K_d]:
                steer -= 1.0
            handbrake = keys[pygame.K_SPACE]

        # --- advance time
        dt = clock.tick(60) / 1000.0
        dt = min(dt, 0.05)  # clamp giant stalls
        sim_time += dt
        ros_mock.CLOCK.set(sim_time)

        # Publish odometry (every frame, so safety node has current velocity)
        odom_msg = build_odometry(physics.state.speed, physics.state.steering)
        odom_pub.publish(odom_msg)

        # Publish nav command (so safety node knows steering intent)
        nav_msg = build_nav_cmd(
            throttle * physics.params.max_speed,
            steer * physics.params.max_steering_angle
        )
        nav_pub.publish(nav_msg)

        # --- (maybe) publish a new lidar scan
        if sim_time >= next_scan:
            next_scan = sim_time + scan_period
            lx, ly = physics.state.laser_xy(physics.params)
            ranges = lidar.scan(world, lx, ly, physics.state.theta)
            msg = build_laserscan(lidar_params, ranges)
            # Clear the "last safety msg" slot BEFORE publishing, so we
            # can tell whether the safety node reacted to THIS scan.
            ros_mock.BROKER.last_msg.pop(SAFETY_TOPIC, None)
            scan_pub.publish(msg)
        else:
            # Even on non-scan frames, keep whatever ranges we have for drawing
            lx, ly = physics.state.laser_xy(physics.params)
            ranges = lidar.scan(world, lx, ly, physics.state.theta)

        # Drive periodic timers (the safety node's laser-timeout check).
        ros_mock.pump_timers(sim_time)

        # --- read whether the safety node issued a stop/limit
        safety_msg = ros_mock.BROKER.last_msg.get(SAFETY_TOPIC)
        safety_active = False
        override_speed = None
        override_steer = None
        if safety_msg is not None:
            safety_active = True
            override_speed = float(safety_msg.drive.speed)
            # Check if safety is commanding a specific steering (e.g., escape maneuver)
            if hasattr(safety_msg.drive, 'steering_angle') and safety_msg.drive.steering_angle != 0.0:
                override_steer = safety_msg.drive.steering_angle

        if handbrake:
            override_speed = 0.0
            safety_active = True  # visualize it like a safety stop

        # Compute "nav cmd" (what the user is asking for) for HUD
        nav_speed_cmd = throttle * physics.params.max_speed

        # Apply steering override if safety is commanding escape maneuver
        if override_steer is not None:
            steer = override_steer / physics.params.max_steering_angle  # Normalize to [-1, 1]

        # --- step physics (honouring safety override + wall collisions)
        def collides(x: float, y: float, theta: float) -> bool:
            return world.footprint_collides(
                x, y, theta,
                physics.params.length,
                physics.params.width,
                physics.params.rear_overhang,
            )

        def penetration(x: float, y: float, theta: float) -> int:
            return world.footprint_occupancy_count(
                x, y, theta,
                physics.params.length,
                physics.params.width,
                physics.params.rear_overhang,
            )

        collided = physics.step(
            throttle, steer, dt,
            override_speed=override_speed,
            collides=collides,
            penetration=penetration,
        )
        s = physics.state

        # --- camera + draw
        renderer.camera.follow(s.x, s.y)
        renderer.draw_world(world)

        # Compute min-front distance for HUD using the current cone angle
        current_cone = safety_node.compute_cone_angle(s.speed)
        mask = (np.abs(
            lidar_params.angle_min + np.arange(len(ranges)) * lidar_params.angle_increment
        ) < current_cone) & (ranges > 0.1)
        min_front = float(np.min(ranges[mask])) if np.any(mask) else float("inf")

        # --- Test mode tracking ---
        if test_mode:
            test_max_speed = max(test_max_speed, s.speed)
            test_min_distance = min(test_min_distance, min_front)
            if collided:
                test_collision = True
                print(f"\n*** COLLISION at t={sim_time:.2f}s! ***")
                print(f"    Position: ({s.x:.3f}, {s.y:.3f})")
                print(f"    Speed at collision: {s.speed:.2f} m/s")
                print(f"    Min distance seen: {test_min_distance:.3f}m")
                running = False  # End test on collision

            # Check test duration
            if sim_time >= test_duration:
                print(f"\n{'='*60}")
                print("TEST COMPLETE")
                print(f"{'='*60}")
                print(f"Duration: {sim_time:.2f}s")
                print(f"Max speed reached: {test_max_speed:.2f} m/s")
                print(f"Min distance to wall: {test_min_distance:.3f}m")
                print(f"Required min gap: {test_min_gap:.3f}m")
                if test_collision:
                    print("Result: FAILED - Collision occurred!")
                elif test_min_distance < test_min_gap:
                    print(f"Result: FAILED - Got closer than {test_min_gap}m!")
                else:
                    print("Result: PASSED - No collision, maintained safe distance!")
                print(f"{'='*60}")
                running = False

        # Draw safety visualization with dynamic parameters
        stopping_dist = safety_node.compute_stopping_distance(s.speed)
        dynamic_critical = safety_node.compute_critical_distance(s.speed)
        renderer.draw_safety_cone(
            s, physics.params,
            stopping_dist,  # Dynamic stopping distance
            current_cone,   # Dynamic cone angle
            safety_active,
            critical_distance=dynamic_critical,  # Now velocity-dependent!
            zone=safety_node.current_zone,
        )
        if show_rays:
            renderer.draw_lidar(s, physics.params, lidar_params, ranges)
        renderer.draw_car(s, physics.params, safety_active)

        fps_display = 0.9 * fps_display + 0.1 * clock.get_fps()

        # Enhanced HUD with safety zone info
        zone_names = {0: "CLEAR", 1: "CAUTION", 2: "AVOIDANCE", 3: "CRITICAL"}
        zone_name = zone_names.get(safety_node.current_zone, "?")
        renderer.draw_hud_v2(
            s, throttle, min_front, safety_active, nav_speed_cmd, fps_display,
            stopping_dist, current_cone, zone_name,
            external_stop_active, speed_limit_values[speed_limit_idx]
        )
        renderer.flip()

    rclpy.shutdown()
    pygame.quit()

    # Return appropriate exit code for test mode
    if test_mode:
        if test_collision or test_min_distance < test_min_gap:
            return 1  # Test failed
        return 0  # Test passed
    return 0


if __name__ == "__main__":
    sys.exit(main())
