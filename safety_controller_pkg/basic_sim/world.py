"""World / obstacle map.

Supports two backends:
  1. A procedurally generated corridor with scattered rectangular obstacles
     (fast, requires no external files)
  2. A PNG occupancy map like racecar_simulator/maps/building_31.png
     (loaded via pygame — only if the user passes --map)

Internally the world is a grid of booleans (True = occupied) indexed by
(row, col) with a known `resolution` in meters per cell and a known origin
in world coordinates. This matches how the racecar_simulator stores maps.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass

import numpy as np


@dataclass
class World:
    grid: np.ndarray          # (H, W) bool, True = occupied
    resolution: float         # meters per cell
    origin_x: float           # world-frame x of grid[0, 0] lower-left
    origin_y: float

    @property
    def height(self) -> int:
        return self.grid.shape[0]

    @property
    def width(self) -> int:
        return self.grid.shape[1]

    @property
    def world_width(self) -> float:
        return self.width * self.resolution

    @property
    def world_height(self) -> float:
        return self.height * self.resolution

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def is_occupied(self, x: float, y: float) -> bool:
        r, c = self.world_to_grid(x, y)
        if r < 0 or r >= self.height or c < 0 or c >= self.width:
            return True  # out-of-bounds = wall
        return bool(self.grid[r, c])

    def footprint_occupancy_count(
        self,
        x: float,
        y: float,
        theta: float,
        length: float,
        width: float,
        rear_overhang: float = 0.0,
    ) -> int:
        """How many footprint samples fall on occupied cells.

        Same sampling scheme as `footprint_collides`, but returns the raw
        count instead of short-circuiting at the first hit. Used by the
        physics step to compare 'how stuck are we' between two poses when
        the car is already touching a wall, so we can reject moves that
        push deeper in while accepting moves that pull out.
        """
        return self._footprint_samples(
            x, y, theta, length, width, rear_overhang,
            stop_on_first=False,
        )

    def footprint_collides(
        self,
        x: float,
        y: float,
        theta: float,
        length: float,
        width: float,
        rear_overhang: float = 0.0,
    ) -> bool:
        """Does the car's oriented rectangle overlap any occupied cell?

        The rectangle in the car frame has:
          x_local in [-rear_overhang, length - rear_overhang]
          y_local in [-width/2, width/2]

        We rasterise the rectangle by sampling its perimeter and a
        coarse interior grid at roughly half the map resolution, then
        rotate+translate each sample into the world and test the cell.

        This is pixel-perfect against the occupancy grid up to the
        sampling density (step = resolution / 2), which is plenty fine
        for a 5 cm grid and a 0.28 m-wide car.
        """
        return self._footprint_samples(
            x, y, theta, length, width, rear_overhang,
            stop_on_first=True,
        ) > 0

    def _footprint_samples(
        self,
        x: float,
        y: float,
        theta: float,
        length: float,
        width: float,
        rear_overhang: float,
        stop_on_first: bool,
    ) -> int:
        """Rasterise the car's oriented rectangle and count occupied
        samples. Returns after the first hit if `stop_on_first` is True."""
        x_back = -rear_overhang
        x_front = length - rear_overhang
        y_side = width / 2.0

        # Sample step — half a grid cell, so we can't thread the car
        # between two adjacent occupied cells.
        step = max(1e-3, self.resolution * 0.5)

        def _linspace(a: float, b: float, s: float) -> list[float]:
            n = max(2, int(math.ceil(abs(b - a) / s)) + 1)
            return [a + (b - a) * i / (n - 1) for i in range(n)]

        xs = _linspace(x_back, x_front, step)
        ys = _linspace(-y_side, y_side, step)

        # Perimeter: top/bottom edges, then left/right (skip corners —
        # already covered by the edge loops).
        perimeter: list[tuple[float, float]] = []
        for xl in xs:
            perimeter.append((xl, -y_side))
            perimeter.append((xl, y_side))
        for yl in ys[1:-1]:
            perimeter.append((x_back, yl))
            perimeter.append((x_front, yl))

        # Interior fill — coarser (one full resolution) since anything
        # smaller than a grid cell can't hide in the middle of a 0.28 m
        # footprint if the perimeter is already clear. This catches the
        # case where a big obstacle fully engulfs the car.
        interior_step = self.resolution
        ixs = _linspace(x_back + interior_step, x_front - interior_step, interior_step)
        iys = _linspace(-y_side + interior_step, y_side - interior_step, interior_step)
        interior: list[tuple[float, float]] = []
        for xl in ixs:
            for yl in iys:
                interior.append((xl, yl))

        c, s = math.cos(theta), math.sin(theta)
        count = 0
        for xl, yl in perimeter + interior:
            wx = x + c * xl - s * yl
            wy = y + s * xl + c * yl
            if self.is_occupied(wx, wy):
                count += 1
                if stop_on_first:
                    return count
        return count


def build_demo_world(
    width_m: float = 20.0,
    height_m: float = 12.0,
    resolution: float = 0.05,
) -> World:
    """A hand-crafted corridor with a few obstacles — good for demoing
    the safety controller without shipping a huge PNG."""
    W = int(width_m / resolution)
    H = int(height_m / resolution)
    grid = np.zeros((H, W), dtype=bool)

    # Border walls (0.2 m thick)
    wall = max(1, int(0.2 / resolution))
    grid[:wall, :] = True
    grid[-wall:, :] = True
    grid[:, :wall] = True
    grid[:, -wall:] = True

    def fill_rect(x0, y0, x1, y1):
        r0 = max(0, int(y0 / resolution))
        r1 = min(H, int(y1 / resolution))
        c0 = max(0, int(x0 / resolution))
        c1 = min(W, int(x1 / resolution))
        grid[r0:r1, c0:c1] = True

    # Central pillar
    fill_rect(8.0, 5.0, 9.0, 7.0)

    # A narrow gate the car must steer through
    fill_rect(13.0, 0.0, 14.0, 4.5)
    fill_rect(13.0, 7.5, 14.0, 12.0)

    # A small crate the safety controller should catch head-on
    fill_rect(4.0, 5.5, 4.8, 6.5)

    # An L-shaped obstacle on the right side
    fill_rect(16.0, 2.0, 18.0, 2.5)
    fill_rect(17.5, 2.0, 18.0, 6.0)

    return World(grid=grid, resolution=resolution, origin_x=0.0, origin_y=0.0)


def load_png_map(path: str, resolution: float = 0.05) -> World:
    """Load an occupancy grid from a grayscale PNG like the simulator's
    building_31.png. Dark pixels (<128) are occupied. Image Y is flipped
    so world-y grows upward like in ROS."""
    try:
        from PIL import Image
    except ImportError as e:  # pragma: no cover
        raise RuntimeError(
            "PNG map loading requires Pillow. Install via `pip install Pillow`."
        ) from e

    if not os.path.exists(path):
        raise FileNotFoundError(path)

    img = Image.open(path).convert("L")
    arr = np.array(img, dtype=np.uint8)
    # Flip vertically so row 0 = world-y min
    arr = np.flipud(arr)
    grid = arr < 128
    return World(grid=grid, resolution=resolution, origin_x=0.0, origin_y=0.0)
