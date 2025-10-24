# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from . import rt_config as C
from . import rt_slam as M
from . import rt_perception as P

def draw_axes_overlay(ax):
    ax.axhline(C.CY, color='lime', lw=0.8, alpha=0.7, zorder=3)
    ax.axvline(C.CX, color='red',  lw=0.8, alpha=0.7, zorder=3)
    px_1m = int(round(1.0 / C.RES))
    ax.quiver(C.CX, C.CY, px_1m, 0, angles='xy', scale_units='xy', scale=1, color='lime', width=0.003, zorder=4)
    ax.quiver(C.CX, C.CY, 0, -px_1m, angles='xy', scale_units='xy', scale=1, color='red',  width=0.003, zorder=4)
    ax.text(C.CX + px_1m + 6, C.CY, "+X (1m)", color='lime', fontsize=8, va='center', zorder=5)
    ax.text(C.CX + 6, C.CY - px_1m - 6, "+Y (1m)", color='red',  fontsize=8, va='bottom', zorder=5)

def setup_figure(goal_xy_m=None, tol_m=None, enable_return=False):
    plt.ion()
    fig, ax = plt.subplots(figsize=(8,8))
    img = ax.imshow(np.full((C.H,C.W), 0.5, dtype=np.float32),
                    vmin=0.0, vmax=1.0, cmap='gray', origin='upper')
    ax.set_title("基于局部感知的自主探索 (灰=未知 黑=自由 白=障碍)")
    ax.set_xticks([]); ax.set_yticks([])
    draw_axes_overlay(ax)
    path_line,   = ax.plot([], [], color='cyan',   lw=1.2, alpha=0.9, zorder=6)
    heading_line,= ax.plot([], [], color='yellow', lw=1.5, alpha=0.9, zorder=7)
    robot_square = Polygon([[0,0],[0,0],[0,0],[0,0]], closed=True, fill=False,
                           edgecolor='red', linewidth=2, zorder=8)
    ax.add_patch(robot_square)

    start_ix, start_iy = M.world_to_cell(0.0, 0.0)
    ax.scatter([start_ix], [start_iy], c='blue', s=200, marker='o',
               edgecolors='white', linewidths=2, zorder=9, label='起点')

    if goal_xy_m is not None:
        gx, gy = goal_xy_m
        gi, gj = M.world_to_cell(gx, gy)
        ax.scatter([gi], [gj], c='green', s=200, marker='*',
                   edgecolors='white', linewidths=2, zorder=9, label='终点')
        if tol_m is not None:
            ax.add_patch(Circle((gi, gj), tol_m / C.RES,
                                fill=False, edgecolor='green',
                                linewidth=1.5, linestyle='--', alpha=0.7, zorder=8))
            if enable_return:
                ax.add_patch(Circle((start_ix, start_iy), tol_m / C.RES,
                                    fill=False, edgecolor='blue',
                                    linewidth=1.5, linestyle='--', alpha=0.7, zorder=8))
    ax.legend(loc='upper right')
    return fig, ax, img, path_line, heading_line, robot_square

def update_figure(ax, img, path_line, heading_line, robot_square,
                  prob_img, path_pts, rel_pose, scan_remain_s, extra_status=""):
    img.set_data(prob_img)
    if path_pts:
        xs, ys = zip(*[M.world_to_cell(x, y) for (x,y) in path_pts])
        path_line.set_data(xs, ys)
    if rel_pose:
        import math
        px0, py0 = M.world_to_cell(rel_pose[0], rel_pose[1])
        th = math.radians((rel_pose[2] + 90.0) % 360.0)
        x1 = rel_pose[0] + 0.5 * math.cos(th); y1 = rel_pose[1] + 0.5 * math.sin(th)
        px1, py1 = M.world_to_cell(x1, y1)
        heading_line.set_data([px0, px1], [py0, py1])
        corners_world = P.get_robot_corners(rel_pose[0], rel_pose[1], rel_pose[2])
        robot_square.set_xy([M.world_to_cell(x, y) for x, y in corners_world])

    ax.set_xlabel(f"{extra_status} | 扫描倒计时={max(0.0,scan_remain_s):.1f}s")
    import matplotlib.pyplot as plt
    plt.pause(0.001)
