"""2D lidar via grid-based ray casting.

Parameters match the real racecar lidar (Hokuyo UST-10LX-style) as configured
in racecar_simulator/share/params.yaml:

    scan_beams         = 100        (reduced from the real 1081 for speed —
                                      the safety node only cares about the
                                      minimum over a 230 deg cone, so denser
                                      beams are not needed for this demo)
    scan_field_of_view = 4.71 rad   (~270 deg)
    scan_std_dev       = 0.01 m     Gaussian noise on each range

We use DDA-style grid ray casting instead of the distance-transform sphere
tracing the real simulator uses. It's plenty fast for a few hundred beams on
a small map and keeps dependencies minimal (no C++, no scikit-fmm).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .world import World


@dataclass
class LidarParams:
    num_beams: int = 100
    field_of_view: float = 4.71       # rad (~270 deg)
    range_max: float = 10.0           # m
    range_min: float = 0.02           # m
    std_dev: float = 0.01             # m — Gaussian measurement noise

    @property
    def angle_min(self) -> float:
        return -self.field_of_view / 2.0

    @property
    def angle_max(self) -> float:
        return self.field_of_view / 2.0

    @property
    def angle_increment(self) -> float:
        return self.field_of_view / (self.num_beams - 1)


class Lidar:
    def __init__(self, params: LidarParams | None = None, rng_seed: int | None = 0):
        self.params = params or LidarParams()
        self._rng = np.random.default_rng(rng_seed)

    def scan(self, world: World, x: float, y: float, theta: float) -> np.ndarray:
        """Return an array of `num_beams` range readings from pose (x, y, theta)."""
        p = self.params
        ranges = np.empty(p.num_beams, dtype=np.float32)
        angles = p.angle_min + np.arange(p.num_beams) * p.angle_increment + theta

        res = world.resolution
        W, H = world.width, world.height
        ox, oy = world.origin_x, world.origin_y
        grid = world.grid
        r_max = p.range_max
        step = res * 0.5  # half a cell — cheap and accurate enough

        n_steps = int(r_max / step) + 1

        # Vectorised over beams: walk all of them outward together.
        cos_a = np.cos(angles)
        sin_a = np.sin(angles)
        xs = np.full(p.num_beams, x, dtype=np.float32)
        ys = np.full(p.num_beams, y, dtype=np.float32)
        done = np.zeros(p.num_beams, dtype=bool)
        hit_dist = np.full(p.num_beams, r_max, dtype=np.float32)

        for i in range(1, n_steps + 1):
            d = i * step
            if d > r_max:
                break
            xs_new = x + d * cos_a
            ys_new = y + d * sin_a
            cols = ((xs_new - ox) / res).astype(np.int32)
            rows = ((ys_new - oy) / res).astype(np.int32)

            out_of_bounds = (
                (cols < 0) | (cols >= W) | (rows < 0) | (rows >= H)
            )
            # Clamp for safe indexing, then gate by out_of_bounds below
            safe_cols = np.clip(cols, 0, W - 1)
            safe_rows = np.clip(rows, 0, H - 1)
            occupied = grid[safe_rows, safe_cols]

            new_hits = (~done) & (occupied | out_of_bounds)
            if np.any(new_hits):
                hit_dist[new_hits] = d
                done[new_hits] = True
            if done.all():
                break

        ranges[:] = hit_dist
        if p.std_dev > 0:
            ranges += self._rng.normal(0.0, p.std_dev, size=p.num_beams).astype(np.float32)
            ranges = np.clip(ranges, p.range_min, p.range_max)
        return ranges
