#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import tkinter as tk
from tkinter import filedialog
from rt1_explore.env import MazeEnvironment
from rt1_explore.A_DWA import plan_path_on_4n_lattice
from rt1_explore.navigator import Navigator
from rt1_explore.viz import Visualizer
from rt1_explore.config import ACTION_GAP_SEC, CELL_METERS, START_SCAN_SEC
from rt1_explore.bluetooth import BTCar

def select_maze_json_gui(initialdir='.'):
    root = tk.Tk(); root.withdraw()
    file_path = filedialog.askopenfilename(
        title="请选择迷宫json文件",
        initialdir=initialdir,
        filetypes=[("JSON files", "*.json"), ("All files", "*.*")]
    )
    root.destroy()
    if not file_path:
        print("未选择文件，程序退出。"); raise SystemExit(1)
    print(f"你选择的迷宫文件是: {file_path}")
    return file_path

def run_simulation(json_path: str, max_steps: int = 200000, bt_port: str = "COM9", bt_baud: int = 921600):
    print("策略：先发送'e'并禁用雷达；按引导点整段下发；段内不扫描；返航同节奏；动作间隔=%.1fs；1格=%.4fm。"
          % (ACTION_GAP_SEC, CELL_METERS))
    maze = MazeEnvironment(width=32, height=32, json_path=json_path)
    maze.set_margin(0.10)

    car = None
    try:
        car = BTCar(port=bt_port, baud=bt_baud, eol="\n", read_async=True)
        print(f"蓝牙已连接: {bt_port}@{bt_baud}")
        # 关闭设备侧雷达TX，避免串口抢占
        try:
            car.radar_tx_off()
        except Exception:
            pass
    except Exception as e:
        print(f"蓝牙连接失败（继续仿真）：{e}")

    path = plan_path_on_4n_lattice(maze)
    nav = Navigator(maze, car=car)
    nav.set_path(path)
    vis = Visualizer(maze, nav)
    nav.set_visual_updater(vis.update)
    vis.update()

    # 起点先出引导点
    nav._ensure_guides()
    vis.update()

    # 起点旋转扫描（可选）
    if nav.lidar_enabled:
        nav._do_scan_stop_and_sweep(START_SCAN_SEC)
        vis.update()

    steps = 0
    reached = False
    try:
        while steps < max_steps:
            done = nav.step_one_waypoint()
            vis.update()
            if done:
                reached = True
                break
            steps += 1
    finally:
        pass

    if reached:
        print("到达终点，原地两次左转并沿来时路返航…")
        nav._turn_left(); nav._turn_left()
        ret_path = list(reversed(path))
        nav.returning = True
        nav.set_path(ret_path, keep_heading=True)

        while True:
            done = nav.step_one_waypoint()
            vis.update()
            if done:
                print("按来时路返航完成。")
                break

    if car is not None:
        try:
            car.stop(); car.close()
        except Exception:
            pass

if __name__ == "__main__":
    maze_json = select_maze_json_gui(initialdir='.')
    run_simulation(json_path=maze_json, max_steps=200000, bt_port="COM9", bt_baud=921600)
