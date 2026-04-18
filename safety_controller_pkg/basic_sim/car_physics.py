"""Bicycle / Ackermann-style physics with inertia, friction, and a steering
rate limit so driving feels like a real racecar, not an arcade top-down.

Parameters mirror the real racecar (see racecar_simulator/share/params.yaml):

    wheelbase          = 0.325 m
    max_speed          = 4.0   m/s
    max_steering_angle = 0.34  rad  (~19.5 deg)

We additionally model:
  * longitudinal acceleration limit (real ESC + wheels cannot jump to 4 m/s)
  * rolling + viscous friction, so releasing the throttle coasts to a stop
  * reverse thrust when the user commands opposite to current motion
  * steering rate limit so the servo feels mechanical

The kinematic bicycle integration matches `ackermann_kinematics.cpp` from
racecar_simulator exactly, so trajectories are 1:1 with the MIT simulator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional


@dataclass
class CarParams:
    # Matches racecar_simulator params.yaml
    wheelbase: float = 0.325           # meters
    max_speed: float = 4.0             # m/s (capped by the VESC mux)
    max_steering_angle: float = 0.34   # rad

    # Physics tuning — not in the simulator params but chosen to feel real
    max_accel: float = 3.0             # m/s^2 when throttling
    max_brake_accel: float = 6.0       # m/s^2 when braking / reverse
    coast_decel: float = 1.2           # m/s^2 passive friction when no input
    steering_rate: float = 3.4         # rad/s — servo can't snap instantly

    # Distance from base_link (rear-axle midpoint) to the laser. We mount
    # the lidar at the front-axle midpoint — i.e. directly between the
    # front wheels, the "front center wheel area" of the chassis — so the
    # scan originates from the nose of the car rather than its centre.
    # That's `wheelbase` metres forward of base_link. Note: the MIT
    # racecar_simulator ships this as 0.275 m (the real Hokuyo on the 6.141
    # car sits ~5 cm behind the front axle); here we use 0.325 to match the
    # "front center" mounting requested for this basic_sim.
    scan_distance_to_base_link: float = 0.325  # meters (= wheelbase)

    # Physical footprint — used for collision and rendering. Matches the
    # real 1/10-scale racecar (~0.50 m long, ~0.28 m wide). The rectangle
    # is centered in Y on base_link; in X it extends from `-rear_overhang`
    # at the back to `length - rear_overhang` at the front, so the rear
    # axle sits slightly inside the chassis.
    length: float = 0.50           # meters, nose to tail
    width: float = 0.28            # meters, side to side
    rear_overhang: float = 0.05    # meters behind base_link


@dataclass
class CarState:
    x: float = 0.0       # meters, map frame
    y: float = 0.0
    theta: float = 0.0   # rad, 0 = +x
    speed: float = 0.0   # m/s, signed (+ fwd, - reverse)
    steering: float = 0.0  # rad, signed

    def laser_xy(self, params: CarParams) -> tuple[float, float]:
        """Position of the lidar, which sits forward of base_link by
        `scan_distance_to_base_link`."""
        d = params.scan_distance_to_base_link
        return (
            self.x + d * math.cos(self.theta),
            self.y + d * math.sin(self.theta),
        )


class CarPhysics:
    """Integrates car state given (throttle, steer_target) inputs."""

    def __init__(self, params: CarParams | None = None):
        self.params = params or CarParams()
        self.state = CarState()

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self.state = CarState(x=x, y=y, theta=theta)

    def step(
        self,
        throttle: float,       # in [-1, 1], user input
        steer_target: float,   # in [-1, 1], user input
        dt: float,
        override_speed: float | None = None,
        collides: Optional[Callable[[float, float, float], bool]] = None,
        penetration: Optional[Callable[[float, float, float], int]] = None,
    ) -> bool:
        """Advance the sim by `dt` seconds.

        If `override_speed` is provided (e.g. the safety node published a
        stop), the car's target longitudinal velocity is clamped to that
        value instead of being derived from the throttle — this models the
        VESC mux letting the safety stop override nav commands.

        If `collides(x, y, theta) -> bool` is provided, it is consulted
        before the new pose is committed. If the proposed pose would
        collide AND the car is currently free, the translation is rejected
        and longitudinal speed is zeroed (while steering still updates).
        If the car is already touching a wall, a move is only accepted when
        it does not make things worse — this lets the user back the car
        away from a wall they are resting against.

        Returns True if a wall collision rejected the motion this tick.
        """
        p = self.params
        s = self.state
        # Remember where we started so we can roll back if we'd hit a wall.
        x0, y0, theta0 = s.x, s.y, s.theta

        # --- steering servo: slew-rate-limited, capped at max angle --
        steer_goal = max(-1.0, min(1.0, steer_target)) * p.max_steering_angle
        max_delta = p.steering_rate * dt
        s.steering += max(-max_delta, min(max_delta, steer_goal - s.steering))
        s.steering = max(-p.max_steering_angle, min(p.max_steering_angle, s.steering))

        # --- longitudinal dynamics -----------------------------------
        if override_speed is not None:
            # Safety override: aggressively decelerate toward override_speed
            target = override_speed
            if s.speed > target:
                s.speed = max(target, s.speed - p.max_brake_accel * dt)
            elif s.speed < target:
                s.speed = min(target, s.speed + p.max_brake_accel * dt)
        else:
            throttle = max(-1.0, min(1.0, throttle))
            if throttle == 0.0:
                # Coast down toward zero via friction
                if s.speed > 0:
                    s.speed = max(0.0, s.speed - p.coast_decel * dt)
                elif s.speed < 0:
                    s.speed = min(0.0, s.speed + p.coast_decel * dt)
            else:
                target = throttle * p.max_speed
                # Pick accel vs brake gain depending on whether we're
                # accelerating with or fighting current motion.
                same_direction = (target >= 0 and s.speed >= 0) or (
                    target <= 0 and s.speed <= 0
                )
                gain = p.max_accel if same_direction else p.max_brake_accel
                if s.speed < target:
                    s.speed = min(target, s.speed + gain * dt)
                else:
                    s.speed = max(target, s.speed - gain * dt)

        s.speed = max(-p.max_speed, min(p.max_speed, s.speed))

        # --- Ackermann bicycle kinematics ----------------------------
        # Mirrors racecar_simulator/src/ackermann_kinematics.cpp
        if p.wheelbase <= 0:
            return False
        dtheta_dt = s.speed * math.tan(s.steering) / p.wheelbase
        new_theta = theta0 + dtheta_dt * dt
        if abs(dtheta_dt) < 1e-6:
            new_x = x0 + dt * s.speed * math.cos(new_theta)
            new_y = y0 + dt * s.speed * math.sin(new_theta)
        else:
            r = s.speed / dtheta_dt
            new_x = x0 + r * (math.sin(new_theta) - math.sin(theta0))
            new_y = y0 + r * (math.cos(theta0) - math.cos(new_theta))

        # --- collision veto ------------------------------------------
        # If moving to the new pose would overlap a wall, reject the
        # translation and kill longitudinal speed. Rotation-only updates
        # are always allowed. If the car was already embedded in a wall
        # (e.g. spawned inside one, or a wall moved), allow any move that
        # does not increase the penetration count — this lets the user
        # back out.
        collided = False
        if collides is not None:
            was_colliding = collides(x0, y0, theta0)
            will_collide = collides(new_x, new_y, new_theta)
            if will_collide and not was_colliding:
                # Fresh collision — refuse the translation, keep rotation.
                collided = True
                s.x, s.y = x0, y0
                s.theta = new_theta if not collides(x0, y0, new_theta) else theta0
                s.speed = 0.0
            elif will_collide and was_colliding:
                # Already touching a wall. Accept the move only if it
                # does not worsen penetration (measured by occupied
                # footprint-sample count). This lets the user back the
                # car away from a wall they're resting against while
                # rejecting attempts to grind further in.
                if penetration is not None:
                    before = penetration(x0, y0, theta0)
                    after = penetration(new_x, new_y, new_theta)
                    improving = after <= before
                else:
                    # No penetration gradient available — fall back to a
                    # tiny-translation jitter test.
                    improving = _jitter_improves(collides, x0, y0, theta0, new_x, new_y, new_theta)
                if improving:
                    s.x, s.y, s.theta = new_x, new_y, new_theta
                else:
                    collided = True
                    s.x, s.y = x0, y0
                    s.theta = (
                        new_theta
                        if not collides(x0, y0, new_theta)
                        else theta0
                    )
                    s.speed = 0.0
            else:
                # Free motion.
                s.x, s.y, s.theta = new_x, new_y, new_theta
        else:
            s.x, s.y, s.theta = new_x, new_y, new_theta

        return collided


def _jitter_improves(
    collides: Callable[[float, float, float], bool],
    x0: float, y0: float, theta0: float,
    x1: float, y1: float, theta1: float,
) -> bool:
    """Fallback gradient test when no penetration count is available.

    Walks the straight-line segment between the old and new pose at 1 cm
    steps and checks whether moving toward the target quickly exits the
    wall. Only used when the caller doesn't supply a penetration count;
    the footprint-count path in world.py gives a much cleaner gradient.
    """
    dx = x1 - x0
    dy = y1 - y0
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return False
    nx, ny = dx / dist, dy / dist
    for k in range(1, 6):
        t = min(1.0, (k * 0.01) / dist)
        px = x0 + nx * (dist * t)
        py = y0 + ny * (dist * t)
        if not collides(px, py, theta0):
            return True
    return False
