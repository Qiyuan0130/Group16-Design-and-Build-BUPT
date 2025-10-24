# -*- coding: utf-8 -*-
import time, threading, math
import numpy as np
from . import rt_config as C
from . import rt_serial as IO
from . import rt_state  as S
from . import rt_slam   as M
from . import rt_viz    as V
from . import rt_decision as D

def _ask_goal():
    print(f"\n{'='*70}")
    print("终点坐标设置")
    print(f"{'='*70}")
    print("说明：起点(0,0)m，右手系，Y 轴为初始前进方向，输入单位 cm")
    use_goal = input("是否设置终点？(y/n，默认n): ").strip().lower()
    goal_x_cm = None; goal_y_cm = None
    goal_tol_cm = 15.0
    enable_return = False
    if use_goal in ('y','yes'):
        try:
            xs, ys = input("请输入终点坐标 (x,y cm, 例: 0,-35): ").strip().split(',')
            goal_x_cm = float(xs.strip()); goal_y_cm = float(ys.strip())
            t_in = input(f"到达容差(cm，默认{goal_tol_cm}): ").strip()
            if t_in: goal_tol_cm = float(t_in)
            r_in = input("是否到达终点后返回起点？(y/n，默认n): ").strip().lower()
            enable_return = r_in in ('y','yes')
        except Exception as e:
            print(f"输入格式错误: {e}，继续无终点模式")
            goal_x_cm = None; goal_y_cm = None
    return goal_x_cm, goal_y_cm, goal_tol_cm, enable_return

def main():
    gx_cm, gy_cm, tol_cm, enable_return = _ask_goal()
    goal_xy_m = (gx_cm/100.0, gy_cm/100.0) if gx_cm is not None and gy_cm is not None else None
    goal_tol_m = tol_cm/100.0

    t_rx = threading.Thread(target=IO.reader, daemon=True); t_rx.start()
    t_poll = threading.Thread(target=IO.poller, daemon=True); t_poll.start()

    fig, ax, img, path_line, heading_line, robot_square = V.setup_figure(
        goal_xy_m=goal_xy_m, tol_m=goal_tol_m if goal_xy_m else None, enable_return=enable_return
    )

    total_recv_pts = 0
    last_plot = time.time()

    state = "SCAN"
    mapping_active = True
    scan_deadline = time.time() + C.SCAN_SEC
    scan_count = 0

    ref_pose_for_move = None
    stop_cmd_sent = False
    stop_cmd_time = 0.0
    action_to_execute = None

    reached_goal = False
    is_returning = False

    print(f"\n{'='*70}")
    print("[启动] 基于局部感知的自主探索系统")
    print(f"[小车尺寸] {C.ROBOT_SIZE*100:.0f}cm × {C.ROBOT_SIZE*100:.0f}cm")
    print("[规则] 扫描5s → 决策 → 前进/转向 → 循环")
    print(f"{'='*70}\n")

    try:
        while True:
            # 取批次
            batch = []
            while len(batch) < C.BATCH_MAX:
                try: batch.extend(IO.Q_POINTS.get_nowait())
                except: break
            if mapping_active and batch:
                M.update_map_batch(batch, M.L_live)
                total_recv_pts += len(batch)

            now = time.time()

            # 实时终点/起点检测
            if goal_xy_m is not None:
                rp = S.get_pose_rel()
                if rp:
                    dist_to_goal = math.hypot(rp[0]-goal_xy_m[0], rp[1]-goal_xy_m[1])
                    if dist_to_goal <= goal_tol_m:
                        if not is_returning and not reached_goal:
                            print(f"\n{'='*70}\n🎯 到达终点（实时）\n{'='*70}")
                            print(f"终点: ({goal_xy_m[0]*100:.1f},{goal_xy_m[1]*100:.1f}) cm  当前: ({rp[0]*100:.1f},{rp[1]*100:.1f}) cm")
                            reached_goal = True
                            if enable_return:
                                IO.send_ascii('S'); time.sleep(1.0)
                                is_returning = True
                                mapping_active = True
                                state = "SCAN"
                                scan_deadline = time.time() + C.SCAN_SEC
                                goal_xy_m = (0.0, 0.0)
                                goal_tol_m = 0.15
                                print("[返回模式] 目标切换为起点 (0,0)m, 容差15cm")
                            else:
                                IO.send_ascii('S'); print("[停止] 完成"); break
                        elif is_returning:
                            print(f"\n{'='*70}\n🏠 返回起点成功\n{'='*70}")
                            IO.send_ascii('S'); print("[停止] 往返完成"); break

            # 状态机
            if state == "SCAN":
                if now >= scan_deadline:
                    scan_count += 1
                    print(f"\n{'='*70}\n[扫描完成 #{scan_count}] 接收点: {total_recv_pts}")
                    np.maximum(M.L_master, M.L_live, out=M.L_master)
                    M.L_live[:] = 0.0
                    mapping_active = False
                    grid_prob = M.prob_from_logodds(M.L_master)
                    rp = S.get_pose_rel()
                    if rp:
                        action_to_execute = D.decide_action(
                            grid_prob, rp[0], rp[1], rp[2],
                            goal_x=(goal_xy_m[0] if goal_xy_m else None),
                            goal_y=(goal_xy_m[1] if goal_xy_m else None)
                        )
                        state = "EXECUTE"
                    else:
                        print("[错误] 无位姿"); break

            elif state == "EXECUTE":
                if action_to_execute == 'forward':
                    print(f"[执行] 前进 {C.MOVE_STEP_M}m")
                    IO.send_ascii(f"M{C.MOVE_STEP_M:.2f}\n")
                    ref_pose_for_move = S.get_pose_rel()
                    state = "MOVING"; stop_cmd_sent = False
                elif action_to_execute == 'turn_left':
                    print("[执行] 左转90°")
                    IO.send_ascii("Lp"); time.sleep(2.0)
                    print(f"[执行] 前进 {C.MOVE_STEP_M}m")
                    IO.send_ascii(f"M{C.MOVE_STEP_M:.2f}\n")
                    ref_pose_for_move = S.get_pose_rel()
                    state = "MOVING"; stop_cmd_sent = False
                elif action_to_execute == 'turn_right':
                    print("[执行] 右转90°")
                    IO.send_ascii("Rp"); time.sleep(2.0)
                    print(f"[执行] 前进 {C.MOVE_STEP_M}m")
                    IO.send_ascii(f"M{C.MOVE_STEP_M:.2f}\n")
                    ref_pose_for_move = S.get_pose_rel()
                    state = "MOVING"; stop_cmd_sent = False
                elif action_to_execute == 'turn_back':
                    print("[执行] 掉头180°")
                    IO.send_ascii("Lp"); time.sleep(2.0)
                    IO.send_ascii("Lp"); time.sleep(2.0)
                    print(f"[执行] 前进 {C.MOVE_STEP_M}m")
                    IO.send_ascii(f"M{C.MOVE_STEP_M:.2f}\n")
                    ref_pose_for_move = S.get_pose_rel()
                    state = "MOVING"; stop_cmd_sent = False
                action_to_execute = None

            elif state == "MOVING":
                rp = S.get_pose_rel()
                if ref_pose_for_move and rp:
                    dist = S.moved_since((ref_pose_for_move[0], ref_pose_for_move[1]),
                                         (rp[0], rp[1]))
                    if dist >= C.MOVE_STEP_M and C.STOP_AFTER_MOVE and not stop_cmd_sent:
                        IO.send_ascii('S'); stop_cmd_sent = True; stop_cmd_time = now
                        print(f"[移动] 已移动{dist:.2f}m，发送停止")
                    stopped = S.is_settled(now)
                    extra_ok = (not C.STOP_AFTER_MOVE) or (stop_cmd_sent and (now - stop_cmd_time) >= C.POST_STOP_SETTLE_SEC)
                    if dist >= C.MOVE_STEP_M and stopped and extra_ok:
                        print(f"[移动完成] 实际{dist:.2f}m → 新一轮扫描")
                        state = "SCAN"; mapping_active = True; scan_deadline = now + C.SCAN_SEC
                        ref_pose_for_move = None; stop_cmd_sent = False

            # 绘图
            if now - last_plot >= 1.0 / C.PLOT_HZ:
                L_show = np.maximum(M.L_master, M.L_live)
                prob_img = M.prob_from_logodds(L_show)
                rel_pose = S.get_pose_rel()
                scan_remain = (scan_deadline - time.time()) if state == "SCAN" else 0
                extra = f"状态={state} | 接收点={total_recv_pts} | 队列={IO.Q_POINTS.qsize()} | 位置=({S.ROBOT_REL_X:.2f},{S.ROBOT_REL_Y:.2f})m | 朝向={S.POSE_YAW_DEG:.1f}°"
                if enable_return and rel_pose:
                    if is_returning:
                        dist_to_start = math.hypot(rel_pose[0], rel_pose[1])
                        extra += f" | 🏠返回起点-距离={dist_to_start*100:.1f}cm"
                    elif goal_xy_m is not None:
                        dist_to_goal = math.hypot(rel_pose[0]-goal_xy_m[0], rel_pose[1]-goal_xy_m[1])
                        extra += f" | 🎯前往终点-距离={dist_to_goal*100:.1f}cm"
                elif goal_xy_m is not None and rel_pose:
                    dist_to_goal = math.hypot(rel_pose[0]-goal_xy_m[0], rel_pose[1]-goal_xy_m[1])
                    extra += f" | 🎯距终点={dist_to_goal*100:.1f}cm"
                V.update_figure(ax, img, path_line, heading_line, robot_square,
                                prob_img, S.path_pts, rel_pose, scan_remain, extra_status=extra)
                last_plot = now

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[中断] 用户停止")
    finally:
        IO.send_ascii('S'); IO.STOP.set(); time.sleep(0.5)
        import matplotlib.pyplot as plt
        plt.ioff(); plt.show()

if __name__ == "__main__":
    main()
