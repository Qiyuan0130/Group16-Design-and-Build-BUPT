import math, numpy as np
from typing import List
from .config import LIDAR_MAX_RANGE, LIDAR_ANG_RES_DEG
from .env import MazeEnvironment

class LidarSimulator:
    def __init__(self, max_range=LIDAR_MAX_RANGE, angle_resolution=LIDAR_ANG_RES_DEG):
        self.max_range = max_range
        self.angle_resolution = angle_resolution
        self.num_beams = int(360 / angle_resolution)
        self.angles = np.linspace(0, 2 * math.pi, self.num_beams, endpoint=False)

    @staticmethod
    def _line_inter(x1, y1, x2, y2, x3, y3, x4, y4):
        den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if den == 0: return None, None
        t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
        return (x1 + t * (x2 - x1), y1 + t * (y2 - y1))

    def _cast(self, sx: float, sy: float, ang: float, maze: MazeEnvironment) -> float:
        min_d = float('inf')
        ex = sx + self.max_range * math.cos(ang)
        ey = sy + self.max_range * math.sin(ang)
        for (x1, y1, x2, y2) in maze.walls:
            if MazeEnvironment.segments_intersect(sx, sy, ex, ey, x1, y1, x2, y2):
                ix, iy = self._line_inter(sx, sy, ex, ey, x1, y1, x2, y2)
                if ix is None: continue
                d = math.hypot(ix - sx, iy - sy)
                if d < min_d: min_d = d
        return min_d if min_d < float('inf') else self.max_range

    def scan(self, rx: float, ry: float, rth: float, maze: MazeEnvironment) -> List[float]:
        out = []
        for a in self.angles:
            beam_a = rth + a
            r = self._cast(rx, ry, beam_a, maze)
            r += np.random.normal(0.0, 0.005)
            out.append(max(0.05, min(self.max_range, r)))
        return out
