# -*- coding: utf-8 -*-
import time, math, threading
from collections import deque
from . import rt_config as C

pose_lock = threading.Lock()

POSE_X = 0.0
POSE_Y = 0.0
POSE_YAW_DEG = 0.0
ORIGIN_SET = False
ORIGIN_X = 0.0
ORIGIN_Y = 0.0

ROBOT_REL_X = 0.0
ROBOT_REL_Y = 0.0

pose_hist = deque(maxlen=300)
path_pts = []
last_path_append_t = 0.0
PATH_MIN_STEP_M = 0.02
PATH_MIN_DT = 0.05

def set_pose_from_device(x_m=None, y_m=None, yaw_deg=None):
    global POSE_X, POSE_Y, POSE_YAW_DEG, ORIGIN_SET, ORIGIN_X, ORIGIN_Y
    global ROBOT_REL_X, ROBOT_REL_Y, last_path_append_t, path_pts
    with pose_lock:
        if x_m is not None: POSE_X = float(x_m)
        if y_m is not None: POSE_Y = float(y_m)
        if yaw_deg is not None: POSE_YAW_DEG = float(yaw_deg) + C.YAW_INPUT_OFFSET_DEG
        if not ORIGIN_SET:
            ORIGIN_X, ORIGIN_Y = POSE_X, POSE_Y; ORIGIN_SET = True
        xr_rel = POSE_X - ORIGIN_X; yr_rel = POSE_Y - ORIGIN_Y
        ROBOT_REL_X, ROBOT_REL_Y = xr_rel, yr_rel
        t = time.time(); pose_hist.append((t, POSE_X, POSE_Y, POSE_YAW_DEG))
        if not path_pts:
            path_pts.append((xr_rel, yr_rel)); last_path_append_t = t
        else:
            if (t - last_path_append_t) >= PATH_MIN_DT:
                lx, ly = path_pts[-1]
                if math.hypot(xr_rel - lx, yr_rel - ly) >= PATH_MIN_STEP_M:
                    path_pts.append((xr_rel, yr_rel)); last_path_append_t = t

def get_pose_rel():
    with pose_lock:
        if not ORIGIN_SET: return None
        return (POSE_X - ORIGIN_X, POSE_Y - ORIGIN_Y, POSE_YAW_DEG)

def moved_since(p0, p1):
    if not p0 or not p1: return 0.0
    return math.hypot(p1[0]-p0[0], p1[1]-p0[1])

def is_settled(now=None):
    now = now or time.time()
    with pose_lock:
        if len(pose_hist) < 2: return False
        latest = None; earliest = None
        for t,x,y,yaw in reversed(pose_hist):
            if latest is None: latest = (t,x,y,yaw)
            if now - t > C.SETTLE_WINDOW_SEC: break
            earliest = (t,x,y,yaw)
        if earliest is None: earliest = pose_hist[0]
        _,x0,y0,y0aw = earliest; _,x1,y1,y1aw = latest
    d = math.hypot(x1-x0, y1-y0)
    dyaw = abs(((y1aw - y0aw + 180) % 360) - 180)
    return (d <= C.SETTLE_EPS_M) and (dyaw <= C.SETTLE_YAW_EPS_DEG)

def calculate_heading_to_target(cx, cy, tx, ty):
    ang = math.degrees(math.atan2(ty - cy, tx - cx))
    return ang if ang >= 0 else ang + 360

def normalize_angle_diff(diff):
    while diff > 180: diff -= 360
    while diff < -180: diff += 360
    return diff
