"""pygame-based 2D top-down renderer.

Draws:
  * the occupancy grid as a greyscale background
  * the lidar cone (shaded region) and individual rays (colored by distance)
  * the car as a rectangle with a heading arrow and front-wheel chevrons
  * the safety cone (~230 deg) and safety radius (0.25 m) overlays
  * a HUD: speed, steering, throttle, safety state, FPS

Coordinate conventions:
  World x is horizontal and y is vertical (standard right-hand). Screen y
  flips so positive-y is up on screen, matching how the lidar looks on the
  real racecar. All meters-to-pixels conversion goes through `to_screen`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import pygame

from .car_physics import CarParams, CarState
from .lidar import LidarParams
from .world import World


# Colors
BG = (240, 240, 240)
WALL = (40, 40, 40)
CAR_OK = (30, 120, 220)
CAR_BRAKE = (220, 60, 60)
LIDAR_RAY = (80, 200, 120, 110)
LIDAR_HIT = (255, 180, 40)
SAFETY_CONE_OK = (80, 220, 140, 50)
SAFETY_CONE_WARN = (240, 100, 100, 70)
SAFETY_RADIUS = (240, 80, 80, 40)
TEXT = (20, 20, 20)
TEXT_ALERT = (200, 40, 40)


@dataclass
class Camera:
    """Maps world meters -> screen pixels. Follows the car with a dead-zone."""
    cx: float = 10.0   # world-x of camera center
    cy: float = 6.0
    zoom: float = 40.0  # pixels per meter

    def follow(self, x: float, y: float) -> None:
        # Smooth follow — linear interpolation
        alpha = 0.15
        self.cx += (x - self.cx) * alpha
        self.cy += (y - self.cy) * alpha


class Renderer:
    def __init__(self, width: int = 1100, height: int = 720):
        pygame.init()
        pygame.display.set_caption("Safety Controller Simulator")
        self.screen = pygame.display.set_mode((width, height))
        self.width = width
        self.height = height
        self.font = pygame.font.SysFont("monospace", 15)
        self.font_big = pygame.font.SysFont("monospace", 22, bold=True)
        self.camera = Camera()
        self._bg_surface: pygame.Surface | None = None
        self._bg_key: tuple | None = None

    # ----- coord helpers -------------------------------------------------
    def to_screen(self, x: float, y: float) -> tuple[int, int]:
        z = self.camera.zoom
        sx = int(self.width / 2 + (x - self.camera.cx) * z)
        sy = int(self.height / 2 - (y - self.camera.cy) * z)
        return sx, sy

    # ----- rendering -----------------------------------------------------
    def _render_world_bg(self, world: World) -> pygame.Surface:
        """Rasterise the world grid as a pygame Surface, then reuse it."""
        key = (id(world), self.camera.zoom)
        if self._bg_surface is not None and self._bg_key == key:
            return self._bg_surface

        # Render the world at its native resolution into a Surface, then
        # scale to current zoom. Dark cells = walls.
        g = world.grid
        arr = np.where(g, 40, 255).astype(np.uint8)
        # pygame expects (W, H, 3) with y-down. Our grid row 0 is
        # world-y min (bottom), so flip vertically for screen.
        arr = np.flipud(arr)
        rgb = np.stack([arr, arr, arr], axis=-1)
        # pygame.surfarray expects (width, height, 3) — transpose
        rgb = np.transpose(rgb, (1, 0, 2))
        surf = pygame.surfarray.make_surface(rgb)

        world_px_w = int(world.world_width * self.camera.zoom)
        world_px_h = int(world.world_height * self.camera.zoom)
        surf = pygame.transform.scale(surf, (max(1, world_px_w), max(1, world_px_h)))
        self._bg_surface = surf
        self._bg_key = key
        return surf

    def draw_world(self, world: World) -> None:
        self.screen.fill(BG)
        surf = self._render_world_bg(world)
        # The surface represents the rectangle
        # [origin_x, origin_x + world_width] x [origin_y, origin_y + world_height]
        top_left = self.to_screen(world.origin_x, world.origin_y + world.world_height)
        self.screen.blit(surf, top_left)

    def draw_safety_cone(
        self,
        state: CarState,
        car_params: CarParams,
        safe_distance: float,
        cone_half_angle: float,
        braking: bool,
    ) -> None:
        """Translucent pie showing what the safety node is watching."""
        lx, ly = state.laser_xy(car_params)
        sx, sy = self.to_screen(lx, ly)
        radius_px = int(safe_distance * 2 * self.camera.zoom)  # warn zone
        if radius_px < 4:
            return

        # 230-deg cone as a polygon
        color = SAFETY_CONE_WARN if braking else SAFETY_CONE_OK
        cone_surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        pts = [(sx, sy)]
        n = 30
        for i in range(n + 1):
            a = -cone_half_angle + (2 * cone_half_angle) * i / n + state.theta
            px = sx + radius_px * math.cos(a)
            py = sy - radius_px * math.sin(a)
            pts.append((px, py))
        pygame.draw.polygon(cone_surface, color, pts)

        # Inner stop-radius circle (0.25 m)
        inner_r = int(safe_distance * self.camera.zoom)
        pygame.draw.circle(cone_surface, SAFETY_RADIUS, (sx, sy), inner_r)
        self.screen.blit(cone_surface, (0, 0))

    def draw_lidar(
        self,
        state: CarState,
        car_params: CarParams,
        lidar_params: LidarParams,
        ranges: np.ndarray,
    ) -> None:
        lx, ly = state.laser_xy(car_params)
        sx0, sy0 = self.to_screen(lx, ly)
        angle_min = lidar_params.angle_min
        ainc = lidar_params.angle_increment
        for i, r in enumerate(ranges):
            if not np.isfinite(r):
                continue
            a = angle_min + i * ainc + state.theta
            ex = lx + r * math.cos(a)
            ey = ly + r * math.sin(a)
            sx1, sy1 = self.to_screen(ex, ey)
            pygame.draw.line(self.screen, (80, 200, 120), (sx0, sy0), (sx1, sy1), 1)
            pygame.draw.circle(self.screen, LIDAR_HIT, (sx1, sy1), 2)

    def draw_car(self, state: CarState, car_params: CarParams, braking: bool) -> None:
        # Dimensions come from CarParams so the visual rectangle is the
        # same shape the collision code uses — no mismatch between what
        # the user sees and what blocks motion.
        L, W = car_params.length, car_params.width
        rear = car_params.rear_overhang
        # Corners in local frame, center on base_link (rear axle midpoint)
        local = np.array([
            (-rear,      -W / 2),
            (L - rear,   -W / 2),
            (L - rear,    W / 2),
            (-rear,       W / 2),
        ])
        c, s = math.cos(state.theta), math.sin(state.theta)
        R = np.array([[c, -s], [s, c]])
        world_corners = (local @ R.T) + np.array([state.x, state.y])
        screen_pts = [self.to_screen(x, y) for x, y in world_corners]
        color = CAR_BRAKE if braking else CAR_OK
        pygame.draw.polygon(self.screen, color, screen_pts)
        pygame.draw.polygon(self.screen, (0, 0, 0), screen_pts, 2)

        # Heading arrow
        hx, hy = state.x + L * c, state.y + L * s
        pygame.draw.line(
            self.screen,
            (0, 0, 0),
            self.to_screen(state.x, state.y),
            self.to_screen(hx, hy),
            2,
        )

        # Front-wheel indicators — rotated by steering angle
        wheel_len = 0.12
        front_x = state.x + car_params.wheelbase * c
        front_y = state.y + car_params.wheelbase * s
        for side in (-1, 1):
            wx = front_x + side * (W / 2) * (-s)
            wy = front_y + side * (W / 2) * (c)
            sa = state.theta + state.steering
            ex = wx + wheel_len * math.cos(sa)
            ey = wy + wheel_len * math.sin(sa)
            pygame.draw.line(
                self.screen, (0, 0, 0),
                self.to_screen(wx, wy),
                self.to_screen(ex, ey),
                3,
            )

    def draw_hud(
        self,
        state: CarState,
        throttle: float,
        min_front_dist: float,
        safety_active: bool,
        nav_speed_cmd: float,
        fps: float,
    ) -> None:
        lines = [
            f"Speed      : {state.speed:+.2f} m/s",
            f"Steering   : {math.degrees(state.steering):+.1f} deg",
            f"Throttle   : {throttle:+.2f}  (nav cmd: {nav_speed_cmd:+.2f} m/s)",
            f"Pose       : ({state.x:+.2f}, {state.y:+.2f}) theta={math.degrees(state.theta):+.1f}",
            f"Min front  : {min_front_dist:.2f} m",
            f"FPS        : {fps:.0f}",
        ]
        for i, text in enumerate(lines):
            surf = self.font.render(text, True, TEXT)
            self.screen.blit(surf, (12, 10 + 18 * i))

        banner = (
            "  SAFETY OVERRIDE — BRAKING  "
            if safety_active
            else "  Safety: clear  "
        )
        banner_color = (220, 60, 60) if safety_active else (60, 160, 90)
        surf = self.font_big.render(banner, True, (255, 255, 255))
        bg = pygame.Surface((surf.get_width() + 16, surf.get_height() + 8))
        bg.fill(banner_color)
        bg.blit(surf, (8, 4))
        self.screen.blit(bg, (self.width - bg.get_width() - 12, 10))

        help_lines = [
            "W/S: throttle   A/D: steer   Space: handbrake",
            "R: reset pose   M: toggle lidar rays   Esc: quit",
        ]
        for i, text in enumerate(help_lines):
            surf = self.font.render(text, True, TEXT)
            self.screen.blit(surf, (12, self.height - 40 + 18 * i))

    def flip(self) -> None:
        pygame.display.flip()
