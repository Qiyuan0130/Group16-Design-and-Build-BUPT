# -*- coding: utf-8 -*-
import math, numpy as np
from . import rt_config as C
from . import rt_state as S

L_master = np.zeros((C.H, C.W), dtype=np.float32)
L_live   = np.zeros((C.H, C.W), dtype=np.float32)

def world_to_cell(x, y):
    ix_raw = int(round(x / C.RES))
    iy_raw = int(round(y / C.RES))
    ix = C.CX + ix_raw
    iy = C.CY - iy_raw if C.GRID_FLIP_Y else C.CY + iy_raw
    return ix, iy

def cell_to_world(ix, iy):
    ix_raw = ix - C.CX
    iy_raw = -(iy - C.CY) if C.GRID_FLIP_Y else (iy - C.CY)
    return ix_raw * C.RES, iy_raw * C.RES

def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
    err = dx + dy; x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1: break
        e2 = 2 * err
        if e2 >= dy: err += dy; x += sx
        if e2 <= dx: err += dx; y += sy

def _stamp_block(grid, ix, iy, delta, k):
    if not (0 <= ix < C.W and 0 <= iy < C.H): return
    if k <= 1:
        grid[iy, ix] = np.clip(grid[iy, ix] + delta, C.L_MIN, C.L_MAX); return
    r = k // 2
    y0 = max(0, iy - r); y1 = min(C.H, iy + r + 1)
    x0 = max(0, ix - r); x1 = min(C.W, ix + r + 1)
    grid[y0:y1, x0:x1] = np.clip(grid[y0:y1, x0:x1] + delta, C.L_MIN, C.L_MAX)

def update_map_batch(batch, grid, l_free=C.L_FREE, l_occ=C.L_OCC):
    with S.pose_lock:
        xr, yr, yaw_deg = S.POSE_X, S.POSE_Y, S.POSE_YAW_DEG
        if not S.ORIGIN_SET: return
        xr_rel = xr - S.ORIGIN_X; yr_rel = yr - S.ORIGIN_Y
    cosc = math.cos; sinc = math.sin; rad = math.radians
    for (a_deg, d_mm, qv) in batch:
        if qv < C.Q_MIN: continue
        r = d_mm / 1000.0
        if r < C.R_MIN or r > C.R_MAX: continue
        a_use = -a_deg if not C.USE_CLOCKWISE_SENSOR_ANGLE else a_deg
        th = rad((yaw_deg + a_use + 90.0) % 360.0)
        xh = xr + r * cosc(th); yh = yr + r * sinc(th)
        xh_rel = xh - S.ORIGIN_X; yh_rel = yh - S.ORIGIN_Y
        ix0, iy0 = world_to_cell(xr_rel, yr_rel)
        ix1, iy1 = world_to_cell(xh_rel, yh_rel)
        if not (0 <= ix0 < C.W and 0 <= iy0 < C.H): continue
        first = True
        for x, y in bresenham(ix0, iy0, ix1, iy1):
            if not (0 <= x < C.W and 0 <= y < C.H): break
            if x == ix1 and y == iy1:
                _stamp_block(grid, x, y, l_occ, C.K_HIT)
            else:
                if first: first = False; continue
                _stamp_block(grid, x, y, l_free, C.K_FREE)

def prob_from_logodds(Lgrid):
    return 1.0 - 1.0 / (1.0 + np.exp(Lgrid))
