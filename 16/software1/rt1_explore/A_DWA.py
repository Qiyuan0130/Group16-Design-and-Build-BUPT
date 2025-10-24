import math, heapq
from typing import Dict, List, Optional, Tuple
from .config import MIN_WALL_CLEARANCE
from .env import MazeEnvironment


def build_lattice_nodes_and_edges(maze: MazeEnvironment):
    cs = maze.cell_size
    xsL = grid_lines_indices(maze.width)
    ysL = grid_lines_indices(maze.height)

    nodes: Dict[Tuple[int,int], Tuple[float,float]] = {}
    for ix in xsL:
        for iy in ysL:
            wx, wy = ix*cs, iy*cs
            if maze.point_on_any_wall(wx, wy): continue
            if maze.is_pose_colliding_margin(wx, wy, 0.0, MIN_WALL_CLEARANCE): continue
            if maze.is_pose_colliding_margin(wx, wy, math.pi/2, MIN_WALL_CLEARANCE): continue
            nodes[(ix,iy)] = (wx, wy)

    nbrs: Dict[Tuple[int,int], List[Tuple[int,int]]] = {k: [] for k in nodes}
    step = 4
    for ix in xsL:
        for iy in ysL:
            a = (ix,iy)
            if a not in nodes: continue
            for jx in [ix+step, ix-step]:
                b = (jx,iy)
                if b in nodes:
                    if not maze.seg_blocked_margin(*nodes[a], *nodes[b], MIN_WALL_CLEARANCE):
                        nbrs[a].append(b); nbrs[b].append(a)
            for jy in [iy+step, iy-step]:
                b = (ix,jy)
                if b in nodes:
                    if not maze.seg_blocked_margin(*nodes[a], *nodes[b], MIN_WALL_CLEARANCE):
                        nbrs[a].append(b); nbrs[b].append(a)

    return nodes, nbrs, xsL, ysL

def choose_attach_node(P: Tuple[float,float], nodes: Dict[Tuple[int,int], Tuple[float,float]], maze: MazeEnvironment) -> Optional[Tuple[int,int]]:
    px, py = P
    best = None
    best_cost = float('inf')
    for k, (wx, wy) in nodes.items():
        ok1 = (not maze.seg_blocked_margin(px, py, wx, py, MIN_WALL_CLEARANCE)) and \
              (not maze.seg_blocked_margin(wx, py, wx, wy, MIN_WALL_CLEARANCE))
        ok2 = (not maze.seg_blocked_margin(px, py, px, wy, MIN_WALL_CLEARANCE)) and \
              (not maze.seg_blocked_margin(px, wy, wx, wy, MIN_WALL_CLEARANCE))
        if ok1 or ok2:
            cost = abs(px - wx) + abs(py - wy)
            if cost < best_cost:
                best_cost = cost; best = k
    return best

def a_star(nodes: Dict[Tuple[int,int], Tuple[float,float]],
           nbrs: Dict[Tuple[int,int], List[Tuple[int,int]]],
           start_k: Tuple[int,int], goal_k: Tuple[int,int]) -> List[Tuple[float,float]]:
    def h(a: Tuple[int,int], b: Tuple[int,int]) -> float:
        ax, ay = nodes[a]; bx, by = nodes[b]
        return abs(ax - bx) + abs(ay - by)
    openh = []
    g = {start_k: 0.0}
    came: Dict[Tuple[int,int], Tuple[int,int]] = {}
    heapq.heappush(openh, (h(start_k, goal_k), start_k))
    closed = set()
    while openh:
        _, cur = heapq.heappop(openh)
        if cur == goal_k:
            path = [cur]
            while cur in came:
                cur = came[cur]; path.append(cur)
            path.reverse()
            return [nodes[k] for k in path]
        if cur in closed: continue
        closed.add(cur)
        for nb in nbrs.get(cur, []):
            w = math.hypot(nodes[cur][0]-nodes[nb][0], nodes[cur][1]-nodes[nb][1])
            ng = g[cur] + w
            if ng < g.get(nb, float('inf')):
                g[nb] = ng
                came[nb] = cur
                heapq.heappush(openh, (ng + h(nb, goal_k), nb))
    return []

def stitch_L(P: Tuple[float,float], Q: Tuple[float,float]) -> List[Tuple[float,float]]:
    px,py = P; qx,qy = Q
    if abs(px-qx)<1e-9 and abs(py-qy)<1e-9:
        return [Q]
    return [(qx,py),(qx,qy)] if abs(px-qx)>1e-9 else [(px,qy),(qx,qy)]

def grid_lines_indices(n_cells: int) -> List[int]:
    raw = [4,8,12,16,20,24,28,32]
    return [v for v in raw if 0 <= v <= n_cells]

def plan_path_on_4n_lattice(maze: MazeEnvironment) -> List[Tuple[float,float]]:
    nodes, nbrs, _, _ = build_lattice_nodes_and_edges(maze)
    if not nodes:
        raise RuntimeError("4n 栅格无可用节点。")
    sx, sy = maze.start_world; gx, gy = maze.goal_world
    s_key = choose_attach_node((sx,sy), nodes, maze)
    g_key = choose_attach_node((gx,gy), nodes, maze)
    if s_key is None or g_key is None:
        raise RuntimeError("起/终点无法连到 4n 节点。")
    mid = a_star(nodes, nbrs, s_key, g_key)
    if not mid:
        raise RuntimeError("A* 在 4n 栅格上无解。")

    path: List[Tuple[float,float]] = []
    path.append((sx,sy))
    path += stitch_L((sx,sy), mid[0])
    path += mid[1:]
    path += stitch_L(mid[-1], (gx,gy))

    simp = [path[0]]
    for i in range(1, len(path)-1):
        ax,ay = simp[-1]; bx,by = path[i]; cx,cy = path[i+1]
        if (abs(ax-bx)<1e-8 and abs(bx-cx)<1e-8) or (abs(ay-by)<1e-8 and abs(by-cy)<1e-8):
            continue
        simp.append(path[i])
    simp.append(path[-1])
    return simp
