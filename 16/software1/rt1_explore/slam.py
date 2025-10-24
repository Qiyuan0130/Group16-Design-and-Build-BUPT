import math, numpy as np
from typing import List, Tuple, Optional
from .config import MAP_SIZE, MAP_RES, LIDAR_MAX_RANGE

class SimpleSLAM:
    def __init__(self, map_size=MAP_SIZE, map_resolution=MAP_RES, max_range=LIDAR_MAX_RANGE):
        self.map_size = map_size
        self.map_resolution = map_resolution
        self.occupancy_grid = np.ones((map_size, map_size)) * 0.5
        self.robot_path: List[Tuple[float, float]] = []
        self.max_range = max_range

    def _bresenham_free(self, x0: int, y0: int, x1: int, y1: int):
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        H, W = self.occupancy_grid.shape
        while True:
            if 0 <= x < W and 0 <= y < H:
                if abs(self.occupancy_grid[y, x] - 0.5) < 1e-6:
                    self.occupancy_grid[y, x] = 0.1
            if x == x1 and y == y1: break
            e2 = 2 * err
            if e2 > -dy: err -= dy; x += sx
            if e2 <  dx: err += dx; y += sy

    def update(self, scan: List[float], rx: float, ry: float, rth: float, visible_radius: Optional[float] = None):
        self.robot_path.append((rx, ry))
        C = self.map_size // 2
        res = self.map_resolution
        px0 = int(rx / res) + C
        py0 = int(ry / res) + C
        N = len(scan)
        for i, rng in enumerate(scan):
            if visible_radius is not None and rng > visible_radius:
                continue
            ang = rth + (i * 2 * math.pi / N)
            ex = rx + rng * math.cos(ang)
            ey = ry + rng * math.sin(ang)
            px1 = int(ex / res) + C
            py1 = int(ey / res) + C
            self._bresenham_free(px0, py0, px1, py1)
            if rng < min(self.max_range * 0.95, visible_radius if visible_radius is not None else self.max_range):
                if 0 <= px1 < self.map_size and 0 <= py1 < self.map_size:
                    self.occupancy_grid[py1, px1] = 0.9
