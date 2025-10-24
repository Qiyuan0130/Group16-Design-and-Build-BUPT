import json, math
from typing import List, Optional, Tuple
from .config import UNIT_TO_M, ROBOT_W_M, ROBOT_L_M, EPS_ON_WALL, MIN_WALL_CLEARANCE

class MazeEnvironment:
    def __init__(self, width: int = 32, height: int = 32, json_path: str = "exp.json"):
        self.width = width
        self.height = height
        self.cell_size = UNIT_TO_M
        self.robot_w = ROBOT_W_M
        self.robot_l = ROBOT_L_M
        self.safety_margin = MIN_WALL_CLEARANCE
        self.walls: List[Tuple[float, float, float, float]] = []
        self.start_world: Optional[Tuple[float, float]] = None
        self.goal_world: Optional[Tuple[float, float]] = None
        if json_path:
            self.load_maze_from_json(json_path)
        else:
            self._gen_default()

    def set_margin(self, margin: float):
        self.safety_margin = margin

    @staticmethod
    def _ccw(ax, ay, bx, by, cx, cy):
        return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

    @staticmethod
    def segments_intersect(x1, y1, x2, y2, x3, y3, x4, y4) -> bool:
        a = MazeEnvironment._ccw(x1, y1, x3, y3, x4, y4) != MazeEnvironment._ccw(x2, y2, x3, y3, x4, y4)
        b = MazeEnvironment._ccw(x1, y1, x2, y2, x3, y3) != MazeEnvironment._ccw(x1, y1, x2, y2, x4, y4)
        return a and b

    @staticmethod
    def _point_seg_distance(px, py, x1, y1, x2, y2) -> float:
        vx, vy = x2 - x1, y2 - y1
        wx, wy = px - x1, py - y1
        c1 = vx*wx + vy*wy
        if c1 <= 0: return math.hypot(px - x1, py - y1)
        c2 = vx*vx + vy*vy
        if c2 <= 0: return math.hypot(px - x1, py - y1)
        t = max(0.0, min(1.0, c1 / c2))
        projx, projy = x1 + t*vx, y1 + t*vy
        return math.hypot(px - projx, py - projy)

    def point_on_any_wall(self, px: float, py: float) -> bool:
        for (x1, y1, x2, y2) in self.walls:
            if self._point_seg_distance(px, py, x1, y1, x2, y2) <= EPS_ON_WALL:
                return True
        return False

    @staticmethod
    def _rect_corners(x: float, y: float, th: float, w: float, l: float):
        c, s = math.cos(th), math.sin(th)
        half = [(l/2, w/2), (l/2,-w/2), (-l/2,-w/2), (-l/2,w/2)]
        return [(x + c*px - s*py, y + s*px + c*py) for px, py in half]

    @staticmethod
    def _poly_edges(pts):
        m = len(pts)
        return [(pts[i][0], pts[i][1], pts[(i+1)%m][0], pts[(i+1)%m][1]) for i in range(m)]

    def is_pose_colliding_margin(self, x: float, y: float, th: float, margin: float) -> bool:
        w = self.robot_w + 2.0 * margin
        l = self.robot_l + 2.0 * margin
        rect = self._rect_corners(x, y, th, w, l)
        edges = self._poly_edges(rect)
        for wx1, wy1, wx2, wy2 in self.walls:
            for ex1, ey1, ex2, ey2 in edges:
                if self.segments_intersect(wx1, wy1, wx2, wy2, ex1, ey1, ex2, ey2):
                    return True
        return False

    def seg_blocked_margin(self, x1: float, y1: float, x2: float, y2: float, margin: float) -> bool:
        th = math.atan2(y2 - y1, x2 - x1)
        seg_len = math.hypot(x2 - x1, y2 - y1)
        step = max(self.cell_size * 0.5, min(self.robot_w, self.robot_l) * 0.5)
        n = max(1, int(seg_len / step))
        for i in range(n + 1):
            t = i / n
            px = x1 + (x2 - x1) * t
            py = y1 + (y2 - y1) * t
            if self.is_pose_colliding_margin(px, py, th, margin):
                return True
        return False

    def load_maze_from_json(self, json_path: str):
        try:
            with open(json_path, 'r') as f:
                data = json.load(f)
        except FileNotFoundError:
            self._gen_default(); return
        self.walls.clear()
        cs = self.cell_size
        for seg in data.get("segments", []):
            x1u, y1u = seg["start"]; x2u, y2u = seg["end"]
            self.walls.append((x1u*cs, y1u*cs, x2u*cs, y2u*cs))
        sp = data.get("start_point", [1, 1])
        gp = data.get("goal_point", [self.width-2, self.height-2])
        self.start_world = (float(sp[0])*cs, float(sp[1])*cs)
        self.goal_world  = (float(gp[0])*cs, float(gp[1])*cs)

    def _gen_default(self):
        cs = self.cell_size
        self.walls += [(0,0,self.width*cs,0),(0,self.height*cs,self.width*cs,self.height*cs),
                       (0,0,0,self.height*cs),(self.width*cs,0,self.width*cs,self.height*cs)]
        self.start_world = (1*cs + cs/2, 1*cs + cs/2)
        self.goal_world  = ((self.width-2)*cs + cs/2, (self.height-2)*cs + cs/2)
