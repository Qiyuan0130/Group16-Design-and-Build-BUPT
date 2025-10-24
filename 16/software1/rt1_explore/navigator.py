import math, time, numpy as np
from typing import Callable, List, Optional, Tuple
from .config import (MIN_WALL_CLEARANCE, RETURN_CLEARANCE_M, ACTION_GAP_SEC, CAR_NOMINAL_SPEED_MPS,
                     SIM_STEP_SLEEP, SCAN_INTERVAL_M, PERIODIC_SCAN_SEC, MOVE_VISIBLE_RADIUS, TURN_TIME)
from .utils import wrap_pi
from .env import MazeEnvironment
from .slam import SimpleSLAM
from .lidar_sim import LidarSimulator

try:
    from rt1_explore.bluetooth import BTCar
except Exception:
    BTCar = None

class Navigator:
    def __init__(self, maze: MazeEnvironment, car: Optional['BTCar']=None):
        self.maze = maze
        self.car = car
        self.x, self.y = maze.start_world
        self.theta = 0.0
        self.path: List[Tuple[float,float]] = []
        self.i = 0
        self.traj: List[Tuple[float,float]] = [(self.x,self.y)]
        self._last_cmd = None
        self._vis_update: Optional[Callable[[], None]] = None
        self.slam = SimpleSLAM()
        self.lidar = LidarSimulator()
        self.lidar_enabled = True
        self.visible_radius = MOVE_VISIBLE_RADIUS
        self.dist_since_scan = 0.0
        self.guide_points_lattice: List[Tuple[float,float]] = []
        self.returning = False
        self.scan_during_motion = False  # 行进中不扫描

    def set_visual_updater(self, fn: Callable[[], None]):
        self._vis_update = fn

    def _grid_val_at(self, x: float, y: float) -> Optional[float]:
        m = self.slam
        C = m.map_size // 2
        px = int(x / m.map_resolution) + C
        py = int(y / m.map_resolution) + C
        if 0 <= px < m.map_size and 0 <= py < m.map_size:
            return m.occupancy_grid[py, px]
        return None

    def set_path(self, pts: List[Tuple[float,float]], keep_heading: bool = False):
        for k in range(len(pts)-1):
            dx = abs(pts[k+1][0]-pts[k][0]); dy = abs(pts[k+1][1]-pts[k][1])
            if dx>1e-6 and dy>1e-6:
                raise ValueError("路径必须轴对齐")
        self.path = pts
        self.i = 0
        if (not keep_heading) and len(self.path) >= 2:
            x0, y0 = self.path[0]
            j = 1
            while j < len(self.path) and abs(self.path[j][0] - x0) < 1e-9 and abs(self.path[j][1] - y0) < 1e-9:
                j += 1
            if j < len(self.path):
                x1, y1 = self.path[j]
                if abs(x1 - x0) >= abs(y1 - y0):
                    self.theta = 0.0 if x1 >= x0 else math.pi
                else:
                    self.theta = (math.pi / 2) if y1 >= y0 else (-math.pi / 2)

    # —— 关键：先出引导点，再移动 —— #
    def _ensure_guides(self):
        if self.returning or not self.lidar_enabled:
            return
        self._scan_once(visible_radius=None)
        self._update_lattice_guides_scanned()
        if self._vis_update: self._vis_update()

    # —— 扫描 —— #
    def _scan_once(self, visible_radius=None):
        if not self.lidar_enabled:
            return
        scan = self.lidar.scan(self.x, self.y, self.theta, self.maze)
        self.slam.update(scan, self.x, self.y, self.theta, visible_radius=visible_radius)
        if not self.returning:
            self._update_lattice_guides_scanned()

    def _sweep_seconds(self, seconds: float):
        if not self.lidar_enabled:
            return
        t0 = time.time(); base = self.theta
        while time.time() - t0 < seconds:
            for frac in np.linspace(-0.5, 0.5, 12):
                self.theta = wrap_pi(base + frac * math.radians(120))
                self._scan_once(visible_radius=None)
                if self._vis_update: self._vis_update()
        self.theta = base

    def _update_lattice_guides_scanned(self):
        if not self.lidar_enabled:
            self.guide_points_lattice = []
            return
        idx = [4,8,12,16,20,24,28,32]
        cs = self.maze.cell_size
        max_x = self.maze.width * cs
        max_y = self.maze.height * cs
        pts: List[Tuple[float,float]] = []
        for ix in idx:
            for iy in idx:
                wx, wy = ix*cs, iy*cs
                if not (0.0 <= wx <= max_x and 0.0 <= wy <= max_y): continue
                if self.maze.point_on_any_wall(wx, wy): continue
                if self.maze.is_pose_colliding_margin(wx, wy, 0.0, MIN_WALL_CLEARANCE): continue
                if self.maze.is_pose_colliding_margin(wx, wy, math.pi/2, MIN_WALL_CLEARANCE): continue
                v = self._grid_val_at(wx, wy)
                if v is None: continue
                if v <= 0.3: pts.append((wx, wy))
        pts.sort(key=lambda p: math.hypot(p[0]-self.x, p[1]-self.y))
        self.guide_points_lattice = pts

    def _do_scan_stop_and_sweep(self, seconds: float):
        if self.returning or not self.lidar_enabled:
            return
        if self.car:
            self.car.stop()
            time.sleep(ACTION_GAP_SEC)
        self._sweep_seconds(seconds)
        self._update_lattice_guides_scanned()

    # —— 直行段：先引导点，再移动 —— #
    def _forward_distance(self, dist_m: float):
        remain = max(0.0, dist_m)
        while remain > 1e-6:
            self._ensure_guides()
            if self.returning or not self.lidar_enabled:
                chunk = remain
            else:
                gap = SCAN_INTERVAL_M - self.dist_since_scan
                if gap <= 1e-6: gap = SCAN_INTERVAL_M
                chunk = min(remain, gap)

            if self.car:
                try:
                    self.car.move_meters(chunk)
                except Exception:
                    pass
                t_total = chunk / max(1e-6, CAR_NOMINAL_SPEED_MPS)
                t0 = time.time()
                sx, sy = self.x, self.y
                while True:
                    el = time.time() - t0
                    frac = min(1.0, el / t_total) if t_total > 0 else 1.0
                    nx = sx + frac*chunk*math.cos(self.theta)
                    ny = sy + frac*chunk*math.sin(self.theta)
                    self.x, self.y = nx, ny
                    self.traj.append((self.x, self.y))
                    if self._vis_update: self._vis_update()
                    if frac >= 1.0: break
                    time.sleep(SIM_STEP_SLEEP)
                time.sleep(ACTION_GAP_SEC)
            else:
                step = 0.01
                moved = 0.0
                margin = RETURN_CLEARANCE_M if self.returning else MIN_WALL_CLEARANCE
                while moved + step < chunk - 1e-6:
                    nx = self.x + step*math.cos(self.theta)
                    ny = self.y + step*math.sin(self.theta)
                    if self.maze.is_pose_colliding_margin(nx, ny, self.theta, margin):
                        break
                    self.x, self.y = nx, ny
                    self.traj.append((self.x,self.y))
                    if self._vis_update: self._vis_update()
                    moved += step
                    time.sleep(SIM_STEP_SLEEP)
                last = chunk - moved
                if last > 1e-6:
                    self.x += last*math.cos(self.theta)
                    self.y += last*math.sin(self.theta)
                    self.traj.append((self.x, self.y))
                    if self._vis_update: self._vis_update()
                time.sleep(ACTION_GAP_SEC)

            if not self.returning and self.lidar_enabled:
                self.dist_since_scan += chunk
                if self.dist_since_scan + 1e-9 >= SCAN_INTERVAL_M:
                    self._do_scan_stop_and_sweep(PERIODIC_SCAN_SEC)
                    self.dist_since_scan = 0.0

            remain -= chunk

    def _turn_left(self):
        if self.car:
            self.car.pivot_left()
            time.sleep(TURN_TIME)
            time.sleep(ACTION_GAP_SEC)
        else:
            time.sleep(TURN_TIME); time.sleep(ACTION_GAP_SEC)
        self.theta = wrap_pi(self.theta + math.pi/2)
        if self._vis_update: self._vis_update()

    def _turn_right(self):
        if self.car:
            self.car.pivot_right()
            time.sleep(TURN_TIME)
            time.sleep(ACTION_GAP_SEC)
        else:
            time.sleep(TURN_TIME); time.sleep(ACTION_GAP_SEC)
        self.theta = wrap_pi(self.theta - math.pi/2)
        if self._vis_update: self._vis_update()

    def _turn_around(self):
        self._turn_left(); self._turn_left()

    def _turn_to(self, hd: float):
        legal = [0.0, math.pi/2, math.pi, -math.pi/2]
        th = min(legal, key=lambda t: abs(wrap_pi(hd - t)))
        d = wrap_pi(th - self.theta)
        if abs(d) < 1e-6: return
        if abs(abs(d) - math.pi) < 1e-6: self._turn_around()
        elif d > 0: self._turn_left()
        else: self._turn_right()

    def _move_segment_to(self, tx: float, ty: float):
        if abs(tx - self.x) > 1e-9 and abs(ty - self.y) > 1e-9:
            raise ValueError("段必须轴对齐")
        self._ensure_guides()

        if abs(tx - self.x) >= abs(ty - self.y):
            desired = 0.0 if tx >= self.x else math.pi
            dist = abs(tx - self.x)
        else:
            desired = math.pi/2 if ty >= self.y else -math.pi/2
            dist = abs(ty - self.y)

        margin = RETURN_CLEARANCE_M if self.returning else MIN_WALL_CLEARANCE
        if not self.maze.seg_blocked_margin(self.x, self.y, tx, ty, margin):
            self._turn_to(desired)
            self._forward_distance(dist)
            self.x, self.y = tx, ty
            self.traj.append((self.x, self.y))
            if self._vis_update: self._vis_update()
        else:
            print("警告：段被阻挡，跳过或需重新规划。")

    def step_one_waypoint(self) -> bool:
        if self.i >= len(self.path): return True
        tx, ty = self.path[self.i]
        if abs(tx - self.x) < 1e-9 and abs(ty - self.y) < 1e-9:
            self.i += 1
            return self.i >= len(self.path)
        self._move_segment_to(tx, ty)
        if abs(tx - self.x) < 1e-9 and abs(ty - self.y) < 1e-9:
            self.i += 1
            return self.i >= len(self.path)
        return False
