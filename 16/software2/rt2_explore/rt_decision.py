# -*- coding: utf-8 -*-
import math
from . import rt_config as C
from . import rt_perception as P

def decide_action(grid_prob, rx, ry, yaw_deg, goal_x=None, goal_y=None):
    front_dist = P.get_front_obstacle_distance(grid_prob, rx, ry, yaw_deg, max_dist=2.0)
    has_x_obstacle = P.check_obstacles_in_region(grid_prob, rx, ry, yaw_deg, 'x_axis')
    has_y_front = P.check_obstacles_in_region(grid_prob, rx, ry, yaw_deg, 'y_front')
    has_y_left  = P.check_obstacles_in_region(grid_prob, rx, ry, yaw_deg, 'y_left')
    has_y_right = P.check_obstacles_in_region(grid_prob, rx, ry, yaw_deg, 'y_right')

    print(f"\n{'='*70}")
    print("[障碍物检测]")
    print(f"  前方障碍物距离: {front_dist:.2f}m")
    print(f"  X轴左右35cm内: {'有' if has_x_obstacle else '无'}")
    print(f"  前方35cm: {'有' if has_y_front else '无'} 左35cm: {'有' if has_y_left else '无'} 右35cm: {'有' if has_y_right else '无'}")
    print(f"{'='*70}\n")

    # 死路优先
    DEADEND_FRONT_DIST = 0.80
    DEADEND_SIDE_CLEARANCE = 0.50
    if front_dist < DEADEND_FRONT_DIST and has_y_front:
        lc = P.get_side_clearance(grid_prob, rx, ry, yaw_deg, side='left',  check_range=0.8)
        rc = P.get_side_clearance(grid_prob, rx, ry, yaw_deg, side='right', check_range=0.8)
        print("[死路检测]", f"前={front_dist:.2f}m 左={lc:.2f}m 右={rc:.2f}m 阈=0.50m")
        if lc < DEADEND_SIDE_CLEARANCE and rc < DEADEND_SIDE_CLEARANCE:
            print("[决策] 死路 → 掉头"); return 'turn_back'
        else:
            print("[死路检测] 非死路")

    # 远距避障
    if front_dist < C.FRONT_OBSTACLE_WARN_DIST:
        lc = P.get_side_clearance(grid_prob, rx, ry, yaw_deg, 'left',  C.SIDE_CHECK_RANGE)
        rc = P.get_side_clearance(grid_prob, rx, ry, yaw_deg, 'right', C.SIDE_CHECK_RANGE)
        print(f"[远距避障] 左空旷={lc:.2f}m 右空旷={rc:.2f}m")
        th, min_turn = 0.45, 0.60
        lh, ld = P.check_corridor_wall(grid_prob, rx, ry, yaw_deg, 'left',  th, 10)
        rh, rd = P.check_corridor_wall(grid_prob, rx, ry, yaw_deg, 'right', th, 10)
        print(f"[走廊检测] 左: {'是' if lh else '否'}({ld:.2f}m) 右: {'是' if rh else '否'}({rd:.2f}m)")
        if lh and rh:
            print("[决策] 走廊模式 → 前进"); return 'forward'
        if lc < min_turn and rc < min_turn:
            if lc < 0.50 and rc < 0.50 and front_dist < 0.70:
                print("[决策] 空间极窄且前方近 → 掉头"); return 'turn_back'
            print("[决策] 双侧不足 → 前进"); return 'forward'

        # 终点方向优先（可选）
        goal_side = None
        if goal_x is not None and goal_y is not None:
            yaw = math.radians((yaw_deg + 90) % 360)
            fx, fy = math.cos(yaw), math.sin(yaw)
            gx, gy = goal_x - rx, goal_y - ry
            gd = math.hypot(gx, gy)
            if gd > 1e-3:
                cross = fx * gy - fy * gx
                dot = fx * gx + fy * gy
                ang = abs(math.degrees(math.atan2(cross, dot)))
                if gd < 0.5: amin, amax = 30, 150
                elif gd < 1.0: amin, amax = 45, 135
                else: amin, amax = 60, 120
                print(f"[终点信息] 距={gd:.2f}m 角={ang:.0f}° 范围={amin}-{amax}°")
                if amin <= ang <= amax:
                    goal_side = 'left' if cross > 0 else 'right'
                    print(f"[终点方向] {goal_side}")

        if lc > rc:
            if lc >= min_turn:
                if goal_side == 'right' and rc >= min_turn:
                    print("[决策] 终点优先 → 右转"); return 'turn_right'
                print("[决策] 左侧更空旷 → 左转"); return 'turn_left'
            print("[决策] 左不足 → 前进"); return 'forward'
        elif rc > lc:
            if rc >= min_turn:
                if goal_side == 'left' and lc >= min_turn:
                    print("[决策] 终点优先 → 左转"); return 'turn_left'
                print("[决策] 右侧更空旷 → 右转"); return 'turn_right'
            print("[决策] 右不足 → 前进"); return 'forward'
        else:
            if goal_side == 'left' and lc >= min_turn:
                print("[决策] 对称且终点在左 → 左转"); return 'turn_left'
            if goal_side == 'right' and rc >= min_turn:
                print("[决策] 对称且终点在右 → 右转"); return 'turn_right'
            if lc >= min_turn:
                print("[决策] 对称默认 → 左转"); return 'turn_left'
            print("[决策] 对称但不足 → 前进"); return 'forward'

    if has_x_obstacle and not has_y_front:
        print("[决策] 规则1 → 前进"); return 'forward'

    if has_y_left and has_y_right and has_y_front:
        if front_dist < 0.8:
            print("[决策] 规则2 真死路 → 掉头"); return 'turn_back'
        print("[决策] 规则2 误判避免 → 前进"); return 'forward'

    if has_y_left and has_y_right and not has_y_front:
        print("[决策] 规则2 路口 → 前进"); return 'forward'

    if has_y_left and not has_y_right:
        print("[决策] 规则3 → 右转"); return 'turn_right'
    if has_y_right and not has_y_left:
        print("[决策] 规则4 → 左转"); return 'turn_left'
    if has_y_front:
        print("[决策] 规则5 → 左转"); return 'turn_left'

    # 路口偏好（仅在安全且设置终点）
    if goal_x is not None and goal_y is not None:
        front_clear = (front_dist > 0.8) and (not has_y_front)
        left_clear  = not has_y_left
        right_clear = not has_y_right
        clear_cnt = sum([front_clear, left_clear, right_clear])
        print(f"[路径偏好] 可走数={clear_cnt}")
        if clear_cnt >= 2:
            yaw = math.radians((yaw_deg + 90) % 360)
            fx, fy = math.cos(yaw), math.sin(yaw)
            rxp, ryp = math.cos(yaw - math.pi/2), math.sin(yaw - math.pi/2)
            gx, gy = goal_x - rx, goal_y - ry; gd = math.hypot(gx, gy)
            if gd > 1e-3:
                gx, gy = gx/gd, gy/gd
                sim_f = fx*gx + fy*gy if front_clear else -9
                sim_l = (-rxp)*gx + (-ryp)*gy if left_clear else -9
                sim_r = ( rxp)*gx + ( ryp)*gy if right_clear else -9
                cands = [('forward', sim_f, '直走'),
                         ('turn_left', sim_l, '左转'),
                         ('turn_right', sim_r, '右转')]
                cands = [c for c in cands if c[1] > -8.9]
                cands.sort(key=lambda x: x[1], reverse=True)
                for i,(a,s,n) in enumerate(cands): print(f"[候选] {i+1}. {n} {s:.3f}")
                if cands:
                    best = cands[0]
                    if best[0]=='forward' and len(cands)>=2:
                        second = cands[1]
                        if second[0] in ('turn_left','turn_right') and (best[1]-second[1])<0.15:
                            best = second
                    print(f"[决策] 路口偏好 → {best[2]}"); return best[0]

    print("[决策] 默认 → 前进"); return 'forward'
