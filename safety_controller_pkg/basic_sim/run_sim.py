"""Run the 2D demo simulator.

Usage:
    python3 -m safety_controller_pkg.basic_sim.run_sim
    # or, from inside basic_sim/
    python3 run_sim.py

What happens on startup:
  1. `ros_mock.install()` registers fake `rclpy`, `sensor_msgs`, and
     `ackermann_msgs` modules in sys.modules.
  2. We then `import safety_controller_pkg.safety_node` — the REAL safety
     node code, untouched. Because our mock is API-compatible, it sees
     what it expects.
  3. We construct a `SafetyNode` instance. It subscribes to "/scan" and
     publishes to "/vesc/low_level/input/safety" via our in-process broker.
  4. The main loop:
       * reads keyboard (WASD),
       * drives the car physics forward,
       * ray-traces a fake lidar scan,
       * publishes it on "/scan" — which synchronously fires the safety
         node's callback,
       * if the safety node published a stop on the safety topic since
         last tick, the main loop overrides the car's commanded speed
         to zero (modelling the VESC low-level mux).
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
from safety_controller_pkg.safety_node import SafetyNode  # noqa: E402
import rclpy  # noqa: E402  (this is actually our mock)

from .car_physics import CarPhysics, CarParams  # noqa: E402
from .lidar import Lidar, LidarParams  # noqa: E402
from .renderer import Renderer  # noqa: E402
from .world import build_demo_world, load_png_map  # noqa: E402


SCAN_TOPIC = "/scan"
SAFETY_TOPIC = "/vesc/low_level/input/safety"


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


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Safety controller 2D demo sim.")
    p.add_argument("--map", default=None, help="Optional PNG occupancy map.")
    p.add_argument("--resolution", type=float, default=0.05, help="meters/cell for PNG maps.")
    p.add_argument("--no-noise", action="store_true", help="Disable lidar noise.")
    p.add_argument(
        "--scan-hz", type=float, default=40.0,
        help="Rate at which fake lidar scans are published (default 40 Hz).",
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

    physics = CarPhysics(CarParams())
    physics.reset(start_x, start_y, 0.0)

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

    # Also give ourselves a publisher so we can shove scans into /scan
    scan_pub = safety_node.create_publisher(
        type(build_laserscan(lidar_params, np.array([1.0]))),
        SCAN_TOPIC,
        10,
    )

    # ------------- main loop ---------------------------------------------
    clock = pygame.time.Clock()
    sim_time = 0.0
    scan_period = 1.0 / args.scan_hz
    next_scan = 0.0
    show_rays = True
    running = True
    fps_display = 0.0

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

        keys = pygame.key.get_pressed()
        throttle = 0.0
        steer = 0.0
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

        # --- read whether the safety node issued a stop
        safety_msg = ros_mock.BROKER.last_msg.get(SAFETY_TOPIC)
        safety_active = False
        override_speed = None
        if safety_msg is not None:
            safety_active = True
            override_speed = float(safety_msg.drive.speed)

        if handbrake:
            override_speed = 0.0
            safety_active = True  # visualize it like a safety stop

        # Compute "nav cmd" (what the user is asking for) for HUD
        nav_speed_cmd = throttle * physics.params.max_speed

        # --- step physics (honouring safety override + wall collisions)
        # The physics step consults `collides` before committing the new
        # pose. If a move would drive the car into a wall it is rejected
        # (but rotation is still allowed, so the user can pivot along a
        # wall), and if the car is already touching a wall the move is
        # only accepted when it reduces penetration — i.e. backing away
        # works, pressing further in does not.
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

        physics.step(
            throttle, steer, dt,
            override_speed=override_speed,
            collides=collides,
            penetration=penetration,
        )
        s = physics.state

        # --- camera + draw
        renderer.camera.follow(s.x, s.y)
        renderer.draw_world(world)

        # Compute min-front distance for HUD
        mask = (np.abs(
            lidar_params.angle_min + np.arange(len(ranges)) * lidar_params.angle_increment
        ) < safety_node.CONE_HALF_ANGLE) & (ranges > 0.1)
        min_front = float(np.min(ranges[mask])) if np.any(mask) else float("inf")

        renderer.draw_safety_cone(
            s, physics.params,
            safety_node.SAFE_DISTANCE,
            safety_node.CONE_HALF_ANGLE,
            safety_active,
        )
        if show_rays:
            renderer.draw_lidar(s, physics.params, lidar_params, ranges)
        renderer.draw_car(s, physics.params, safety_active)

        fps_display = 0.9 * fps_display + 0.1 * clock.get_fps()
        renderer.draw_hud(s, throttle, min_front, safety_active, nav_speed_cmd, fps_display)
        renderer.flip()

    rclpy.shutdown()
    pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
