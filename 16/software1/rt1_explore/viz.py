import math, numpy as np, matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, MultipleLocator
from .config import MIN_WALL_CLEARANCE
from .env import MazeEnvironment

class Visualizer:
    def __init__(self, maze: MazeEnvironment, nav):
        self.m = maze
        self.n = nav
        plt.ion()
        self.fig, self.ax_map = plt.subplots(1, 1, figsize=(7, 7))
        self.ax_map.set_aspect('equal', adjustable='box')
        self.ax_nav = None

    def _ticks(self, ax):
        cs = self.m.cell_size
        ax.set_xlim(0, self.m.width * cs)
        ax.set_ylim(0, self.m.height * cs)
        ax.xaxis.set_major_locator(MultipleLocator(cs))
        ax.yaxis.set_major_locator(MultipleLocator(cs))
        ax.xaxis.set_major_formatter(FuncFormatter(lambda v, pos: f"{int(round(v / cs))}"))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda v, pos: f"{int(round(v / cs))}"))
        ax.grid(True, alpha=0.25)

    def _draw_robot_rect(self, ax):
        pts = MazeEnvironment._rect_corners(self.n.x, self.n.y, self.n.theta, self.m.robot_w, self.m.robot_l)
        xs = [p[0] for p in pts] + [pts[0][0]]
        ys = [p[1] for p in pts] + [pts[0][1]]
        ax.plot(xs, ys, 'g-', lw=2)
        tip = (self.n.x + math.cos(self.n.theta) * self.m.robot_l / 2,
               self.n.y + math.sin(self.n.theta) * self.m.robot_l / 2)
        ax.plot([self.n.x, tip[0]], [self.n.y, tip[1]], 'g-', lw=2)

    def _plot_slam(self):
        m = self.n.slam
        H, W = m.occupancy_grid.shape
        cs = self.m.cell_size
        min_x, max_x = 0.0, self.m.width * cs
        min_y, max_y = 0.0, self.m.height * cs
        C = m.map_size // 2
        min_px = int(min_x / m.map_resolution) + C
        max_px = int(max_x / m.map_resolution) + C
        min_py = int(min_y / m.map_resolution) + C
        max_py = int(max_y / m.map_resolution) + C
        crop = m.occupancy_grid[max(0, min_py):min(H, max_py), max(0, min_px):min(W, max_px)]

        FREE_THR, OCC_THR = 0.3, 0.7
        seen = (crop <= FREE_THR) | (crop >= OCC_THR)
        base = np.full_like(crop, 0.5, dtype=float)
        base[seen] = 1.0
        self.ax_map.imshow(
            base, cmap='gray', origin='lower', vmin=0, vmax=1,
            extent=[min_x, max_x, min_y, max_y], interpolation='nearest'
        )

        occ = (crop >= OCC_THR)
        occ_d = occ.copy()
        occ_d |= np.pad(occ[:, :-1], ((0, 0), (1, 0)), constant_values=False)
        occ_d |= np.pad(occ[:-1, :], ((1, 0), (0, 0)), constant_values=False)
        occ_d |= np.pad(occ[:-1, :-1], ((1, 0), (1, 0)), constant_values=False)
        rgba = np.zeros((occ_d.shape[0], occ_d.shape[1], 4), dtype=float)
        rgba[occ_d] = (1.0, 0.55, 0.0, 1.0)
        self.ax_map.imshow(
            rgba, origin='lower',
            extent=[min_x, max_x, min_y, max_y], interpolation='nearest'
        )

        if self.n.returning and self.n.path:
            remain = [(self.n.x, self.n.y)] + self.n.path[self.n.i:]
            if len(remain) >= 2:
                xs = [p[0] for p in remain]; ys = [p[1] for p in remain]
                self.ax_map.plot(xs, ys, color='cyan', lw=2)
        else:
            gps = self.n.guide_points_lattice
            if gps:
                gx, gy = min(gps, key=lambda p: (p[0] - self.n.x) ** 2 + (p[1] - self.n.y) ** 2)
                self.ax_map.plot(gx, gy, 'ro', ms=6)

        self._draw_robot_rect(self.ax_map)
        self._ticks(self.ax_map)
        self.ax_map.set_title("SLAM Map")

    def update(self):
        self.ax_map.clear()
        self._plot_slam()
        plt.tight_layout()
        plt.pause(0.001)
