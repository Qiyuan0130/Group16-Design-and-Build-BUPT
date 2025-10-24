# -*- coding: utf-8 -*-
import math, numpy as np
from . import rt_config as C
from . import rt_slam as M

def get_robot_corners(robot_x, robot_y, robot_yaw):
    yaw_rad = math.radians((robot_yaw + 90) % 360)
    half = C.ROBOT_HALF_SIZE
    local = [(half,half),(half,-half),(-half,-half),(-half,half)]
    cs, sn = math.cos(yaw_rad), math.sin(yaw_rad)
    out = []
    for lx, ly in local:
        out.append((lx*cs - ly*sn + robot_x, lx*sn + ly*cs + robot_y))
    return out

def check_robot_collision(grid_prob, robot_x, robot_y, robot_yaw):
    xs, ys = zip(*get_robot_corners(robot_x, robot_y, robot_yaw))
    min_x, max_x = min(xs), max(xs); min_y, max_y = min(ys), max(ys)
    for x in np.linspace(min_x, max_x, 10):
        for y in np.linspace(min_y, max_y, 10):
            ix, iy = M.world_to_cell(x, y)
            if 0 <= ix < C.W and 0 <= iy < C.H and grid_prob[iy, ix] > C.OCC_THRESH:
                return True
    return False

def check_obstacles_in_region(grid_prob, rx, ry, yaw_deg, region_type):
    yaw = math.radians((yaw_deg + 90) % 360)
    fx, fy = math.cos(yaw), math.sin(yaw)
    rxp, ryp = math.cos(yaw - math.pi/2), math.sin(yaw - math.pi/2)
    centers = []
    if region_type == 'x_axis':
        for r in np.arange(C.ROBOT_HALF_SIZE + 0.05, C.DETECTION_RANGE_M + C.ROBOT_HALF_SIZE, 0.05):
            centers += [(rx + r*rxp, ry + r*ryp), (rx - r*rxp, ry - r*ryp)]
    elif region_type == 'y_front':
        for r in np.arange(C.ROBOT_HALF_SIZE + 0.05, C.DETECTION_RANGE_M + C.ROBOT_HALF_SIZE, 0.05):
            centers.append((rx + r*fx, ry + r*fy))
    elif region_type == 'y_left':
        for r in np.arange(C.ROBOT_HALF_SIZE + 0.05, C.DETECTION_RANGE_M + C.ROBOT_HALF_SIZE, 0.08):
            for ao in np.arange(0, 90, 15):
                a = yaw + math.radians(ao); centers.append((rx + r*math.cos(a), ry + r*math.sin(a)))
    elif region_type == 'y_right':
        for r in np.arange(C.ROBOT_HALF_SIZE + 0.05, C.DETECTION_RANGE_M + C.ROBOT_HALF_SIZE, 0.08):
            for ao in np.arange(-90, 0, 15):
                a = yaw + math.radians(ao); centers.append((rx + r*math.cos(a), ry + r*math.sin(a)))
    obstacle, total = 0, 0
    for px, py in centers:
        total += 1; hit = False
        for dx in np.linspace(-C.ROBOT_HALF_SIZE, C.ROBOT_HALF_SIZE, 5):
            for dy in np.linspace(-C.ROBOT_HALF_SIZE, C.ROBOT_HALF_SIZE, 5):
                ix, iy = M.world_to_cell(px + dx, py + dy)
                if 0 <= ix < C.W and 0 <= iy < C.H and grid_prob[iy, ix] > C.OCC_THRESH:
                    hit = True; break
            if hit: break
        if hit: obstacle += 1
    if total == 0: return False
    return (obstacle / total) > 0.3

def get_front_obstacle_distance(grid_prob, rx, ry, yaw_deg, max_dist=2.0):
    MINC = 10
    yaw = math.radians((yaw_deg + 90) % 360)
    fx, fy = math.cos(yaw), math.sin(yaw)
    rxp, ryp = math.cos(yaw - math.pi/2), math.sin(yaw - math.pi/2)
    for r in np.arange(C.ROBOT_HALF_SIZE + 0.02, max_dist + C.ROBOT_HALF_SIZE, 0.02):
        consec = 0
        for lat in np.arange(-C.ROBOT_HALF_SIZE - 0.1, C.ROBOT_HALF_SIZE + 0.1, C.RES * 2):
            px = rx + r*fx + lat*rxp; py = ry + r*fy + lat*ryp
            ix, iy = M.world_to_cell(px, py)
            if not (0 <= ix < C.W and 0 <= iy < C.H):
                consec += 1; continue
            if grid_prob[iy, ix] > C.OCC_THRESH: consec += 1
            else:
                if consec >= MINC: break
                consec = 0
        if consec >= MINC: return r - C.ROBOT_HALF_SIZE
    return max_dist

def get_side_clearance(grid_prob, rx, ry, yaw_deg, side='left', check_range=1.2):
    MINC = 10
    yaw = math.radians((yaw_deg + 90) % 360)
    fx, fy = math.cos(yaw), math.sin(yaw)
    rxp, ryp = math.cos(yaw - math.pi/2), math.sin(yaw - math.pi/2)
    latx, laty = (-rxp, -ryp) if side == 'left' else (rxp, ryp)
    check_forward = 0.5
    for lateral in np.arange(0.15, check_range, 0.05):
        consec = 0
        for f in np.arange(-check_forward, check_forward, C.RES * 2):
            cx = rx + f*fx + lateral*latx; cy = ry + f*fy + lateral*laty
            ix, iy = M.world_to_cell(cx, cy)
            if not (0 <= ix < C.W and 0 <= iy < C.H):
                consec += 1; continue
            if grid_prob[iy, ix] > C.OCC_THRESH: consec += 1
            else:
                if consec >= MINC: break
                consec = 0
        if consec >= MINC: return lateral
    return check_range

def check_corridor_wall(grid_prob, rx, ry, yaw_deg, side='left', distance_threshold=0.40, min_consecutive=10):
    yaw = math.radians((yaw_deg + 90) % 360)
    fx, fy = math.cos(yaw), math.sin(yaw)
    rxp, ryp = math.cos(yaw - math.pi/2), math.sin(yaw - math.pi/2)
    latx, laty = (-rxp, -ryp) if side == 'left' else (rxp, ryp)
    check_forward = 0.5; max_lat = 0.6
    best_c = 0; best_d = max_lat
    for lateral in np.arange(0.15, max_lat, 0.05):
        c = 0; tmp = []
        for f in np.arange(-check_forward, check_forward, C.RES * 2):
            cx = rx + f*fx + lateral*latx; cy = ry + f*fy + lateral*laty
            ix, iy = M.world_to_cell(cx, cy)
            if 0 <= ix < C.W and 0 <= iy < C.H and grid_prob[iy, ix] > C.OCC_THRESH:
                c += 1; tmp.append(lateral)
            else:
                if c >= min_consecutive and c > best_c:
                    best_c = c; best_d = (sum(tmp)/len(tmp) if tmp else lateral)
                c = 0; tmp = []
        if c >= min_consecutive and c > best_c:
            best_c = c; best_d = (sum(tmp)/len(tmp) if tmp else lateral)
    return (best_c >= min_consecutive and best_d < distance_threshold), best_d
