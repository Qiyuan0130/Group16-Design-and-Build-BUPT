# pip install pyserial numpy matplotlib
import serial, threading, queue, time, math, struct
from collections import deque
import numpy as np
import matplotlib.pyplot as plt

# ==== 串口与协议 ====
PORTS = ["COM9"]
BAUD  = 921600
TYPE_POINTS = 0x11
TYPE_POINT1 = 0x01

# ==== 雷达与过滤 ====
Q_MIN = 15
R_MIN = 0.05
R_MAX = 6.0
USE_CLOCKWISE_SENSOR_ANGLE = False

# ==== 坐标系 ====
YAW_INPUT_OFFSET_DEG = 0.0

# ==== 地图参数 ====
MAP_SIZE_M = 8.0
RES = 0.02
W = H = int(MAP_SIZE_M / RES)
CX, CY = W // 2, H // 2
# 修复：改为右手系（Y轴向上为正，X轴向右为正）
GRID_FLIP_X = False
GRID_FLIP_Y = True  # Y轴需要翻转以匹配右手系

L_master = np.zeros((H, W), dtype=np.float32)
L_live   = np.zeros((H, W), dtype=np.float32)
L_FREE, L_OCC = -0.5, +2.0
L_MIN, L_MAX = -5.0, +5.0

K_FREE = 3
K_HIT  = 3

# ==== 扫描/动作时长 ====
SCAN_SEC = 10.0
MOVE1_M  = 1.5
MOVE2_M  = 0.7

# ==== 动作与稳定判定 ====
SAFE_READ_ONLY   = False
SEND_E_ON_START  = True
POLL_XY_YAW      = True
STOP_AFTER_MOVE        = True
POST_STOP_SETTLE_SEC   = 0.5
RX_STATS_EVERY = 1.0

SETTLE_WINDOW_SEC   = 0.7
SETTLE_EPS_M        = 0.03
SETTLE_YAW_EPS_DEG  = 5.0

# ==== 并发 ====
Q_POINTS = queue.Queue(maxsize=2000)
STOP = threading.Event()
SER = None
TEXT_BUF = bytearray()

# ==== 位姿 ====
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

# ====== 显示细化（可关） ======
OCC_THRESH   = 0.65
CLOSE_ITERS  = 1
LINE_FUSE_ON = True
def _shift(img, dy, dx):
    h, w = img.shape
    y0 = max(0,  dy); y1 = min(h, h+dy)
    x0 = max(0,  dx); x1 = min(w, w+dx)
    out = np.zeros_like(img)
    out[y0:y1, x0:x1] = img[y0-dy:y1-dy, x0-dx:x1-dx]
    return out
def _neighbor_count(binimg):
    s = (_shift(binimg, -1, -1) + _shift(binimg, -1, 0) + _shift(binimg, -1, 1) +
         _shift(binimg,  0, -1)                         + _shift(binimg,  0, 1) +
         _shift(binimg,  1, -1) + _shift(binimg,  1, 0) + _shift(binimg,  1, 1))
    return s
def _dilate(binimg): return (_neighbor_count(binimg) + binimg) > 0
def _erode(binimg):  return (_neighbor_count(binimg) == 8) & (binimg == True)
def _closing(binimg, iters=1):
    out = binimg.copy()
    for _ in range(iters): out = _dilate(out)
    for _ in range(iters): out = _erode(out)
    return out
def _transitions(p):
    seq = [p[0,1], p[0,2], p[1,2], p[2,2], p[2,1], p[2,0], p[1,0], p[0,0], p[0,1]]
    return np.sum((np.array(seq[:-1]) == 0) & (np.array(seq[1:]) == 1))
def _zhang_suen_thinning(binimg):
    img = binimg.copy().astype(np.uint8)
    changed = True
    while changed:
        changed = False
        rm = np.zeros_like(img)
        for y in range(1, img.shape[0]-1):
            for x in range(1, img.shape[1]-1):
                if img[y,x] == 0: continue
                p = img[y-1:y+2, x-1:x+2]; n = _neighbor_count(p)[1,1]
                if n < 2 or n > 6: continue
                if _transitions(p) != 1: continue
                if (p[0,1] * p[1,2] * p[2,1]) != 0: continue
                if (p[1,2] * p[2,1] * p[1,0]) != 0: continue
                rm[y,x] = 1
        if np.any(rm): img[rm==1] = 0; changed = True
        rm[:] = 0
        for y in range(1, img.shape[0]-1):
            for x in range(1, img.shape[1]-1):
                if img[y,x] == 0: continue
                p = img[y-1:y+2, x-1:x+2]; n = _neighbor_count(p)[1,1]
                if n < 2 or n > 6: continue
                if _transitions(p) != 1: continue
                if (p[0,1] * p[1,2] * p[1,0]) != 0: continue
                if (p[0,1] * p[2,1] * p[1,0]) != 0: continue
                rm[y,x] = 1
        if np.any(rm): img[rm==1] = 0; changed = True
    return img.astype(bool)
def fuse_lines_from_prob(prob):
    if not LINE_FUSE_ON: return prob
    occ = (prob >= OCC_THRESH)
    if CLOSE_ITERS > 0: occ = _closing(occ, iters=CLOSE_ITERS)
    skel = _zhang_suen_thinning(occ)
    out = prob.copy(); out[skel] = 1.0
    return out

# ==== 基础工具 ====
def world_to_cell(x, y):
    # 修复：改为右手系坐标转换
    ix_raw = int(round(x / RES))
    iy_raw = int(round(y / RES))
    # X轴：右手系，向右为正
    ix = CX + ix_raw
    # Y轴：右手系，向上为正，但图像坐标系向下为正，所以需要翻转
    iy = CY - iy_raw if GRID_FLIP_Y else CY + iy_raw
    return ix, iy

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

def prob_from_logodds(Lgrid):
    return 1.0 - 1.0 / (1.0 + np.exp(Lgrid))

def draw_axes_overlay(ax):
    ax.axhline(CY, color='lime', lw=0.8, alpha=0.7, zorder=3)
    ax.axvline(CX, color='red',  lw=0.8, alpha=0.7, zorder=3)
    px_1m = int(round(1.0 / RES))
    # 修复：右手系坐标轴显示
    # X轴：向右为正（右手系）
    ax.quiver(CX, CY, px_1m, 0, angles='xy', scale_units='xy', scale=1, color='lime', width=0.003, zorder=4)
    # Y轴：向上为正（右手系），但图像坐标系向下为正，所以箭头向下
    ax.quiver(CX, CY, 0, -px_1m, angles='xy', scale_units='xy', scale=1, color='red',  width=0.003, zorder=4)
    ax.text(CX + px_1m + 6, CY, "+X (1m)", color='lime', fontsize=8, va='center', zorder=5)
    ax.text(CX + 6, CY - px_1m - 6, "+Y (1m)", color='red',  fontsize=8, va='bottom', zorder=5)

# ==== 文本/二进制混流解析 ====
def handle_text_bytes(b: bytes):
    global TEXT_BUF
    if not b: return
    TEXT_BUF += b
    while True:
        p = TEXT_BUF.find(b'\n')
        if p == -1:
            if len(TEXT_BUF) > 2048: TEXT_BUF = TEXT_BUF[-256:]
            return
        line = TEXT_BUF[:p+1]; TEXT_BUF = TEXT_BUF[p+1:]
        parse_text_line(line.decode('utf-8', errors='ignore').strip())

def parse_text_line(s: str):
    if not s: return
    if s.startswith("XY"):
        try:
            _, xs, ys = s.split()
            set_pose_from_device(x_m=float(xs), y_m=float(ys))
        except: pass
    elif s.startswith("YAW"):
        try:
            parts = s.replace(',', ' ').split()
            if len(parts) >= 2: set_pose_from_device(yaw_deg=float(parts[1]))
        except: pass

def parse_frames(buf: bytearray, out_queue: queue.Queue):
    i = 0; n = len(buf)
    batch_local = []
    def flush_local():
        nonlocal batch_local
        if batch_local:
            try: out_queue.put_nowait(batch_local)
            except queue.Full:
                try: out_queue.get_nowait()
                except: pass
                out_queue.put_nowait(batch_local)
            batch_local = []
    while True:
        j = buf.find(b'\xAA\x55', i)
        if j < 0:
            tail_keep = b''
            if n > i and buf[n-1] == 0xAA:
                tail_keep = b'\xAA'; handle_text_bytes(bytes(buf[i:n-1]))
            else:
                handle_text_bytes(bytes(buf[i:n]))
            flush_local(); return bytearray(tail_keep)
        if j > i:
            handle_text_bytes(bytes(buf[i:j])); i = j
        if i + 3 > n:
            flush_local(); return bytearray(buf[i:])
        ln = buf[i+2]; total = 2 + 1 + ln + 1
        if i + total > n:
            flush_local(); return bytearray(buf[i:])
        frame = bytes(buf[i:i+total])
        type_byte = frame[3]; payload = frame[4:-1]; csum = frame[-1]
        if ((type_byte + sum(payload) + csum) & 0xFF) == 0:
            if type_byte == TYPE_POINTS and len(payload) >= 1:
                cnt = payload[0]; need = 1 + cnt * 5
                if len(payload) >= need:
                    for k in range(cnt):
                        a_cdeg, d_mm, qv = struct.unpack_from('<HHB', payload, 1 + k*5)
                        batch_local.append((a_cdeg/100.0, float(d_mm), int(qv)))
            elif type_byte == TYPE_POINT1 and len(payload) == 5:
                a_cdeg, d_mm, qv = struct.unpack_from('<HHB', payload, 0)
                batch_local.append((a_cdeg/100.0, float(d_mm), int(qv)))
            if len(batch_local) >= 20000: flush_local()
            i += total
        else:
            i += 1

# ==== 串口 ====
def send_ascii(cmd: str):
    try:
        if SER: SER.write(cmd.encode('utf-8'))
    except: pass

def reader(port_list=None):
    global SER
    ports = port_list or PORTS
    rx_bytes = 0; last_stat = time.time()
    def open_any():
        last_err = None
        for p in ports:
            try:
                s = serial.Serial(p, BAUD, timeout=0)
                try: s.set_buffer_size(rx_size=262144, tx_size=4096)
                except: pass
                s.reset_input_buffer(); s.reset_output_buffer()
                print(f"[RX] open {s.port} @ {BAUD}"); return s
            except Exception as e: last_err = e
        raise RuntimeError(f"open failed: {ports}, last: {last_err}")
    while not STOP.is_set():
        try:
            SER = open_any(); buf = bytearray()
            while not STOP.is_set():
                n = SER.in_waiting
                if n:
                    data = SER.read(n); rx_bytes += len(data)
                    buf += data; buf = parse_frames(buf, Q_POINTS)
                if RX_STATS_EVERY > 0 and time.time() - last_stat >= RX_STATS_EVERY:
                    print(f"[RX] {rx_bytes} B/s, queue={Q_POINTS.qsize()}")
                    rx_bytes = 0; last_stat = time.time()
                time.sleep(0.0005)
        except Exception as e:
            print(f"[RX] error: {e}. Reopen in 1s...")
            try:
                if SER: SER.close()
            except: pass
            SER = None; time.sleep(1.0)
        finally:
            try:
                if SER: SER.close()
            except: pass
            print("[RX] closed")

def poller():
    if SEND_E_ON_START: send_ascii('E'); time.sleep(0.05)
    last_xy = 0.0; last_yaw = 0.0
    while not STOP.is_set():
        now = time.time()
        if POLL_XY_YAW and (now - last_xy) >= 0.05: send_ascii('Q'); last_xy = now
        if POLL_XY_YAW and (now - last_yaw) >= 0.10: send_ascii('Y'); last_yaw = now
        time.sleep(0.01)

# ==== 位姿/地图 ====
def set_pose_from_device(x_m=None, y_m=None, yaw_deg=None):
    global POSE_X, POSE_Y, POSE_YAW_DEG, ORIGIN_SET, ORIGIN_X, ORIGIN_Y
    global ROBOT_REL_X, ROBOT_REL_Y, last_path_append_t, path_pts
    with pose_lock:
        if x_m is not None: POSE_X = float(x_m)
        if y_m is not None: POSE_Y = float(y_m)
        # 修复：右手系YAW角度处理（保持原始角度，不再取反）
        if yaw_deg is not None: POSE_YAW_DEG = float(yaw_deg) + YAW_INPUT_OFFSET_DEG
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

def _stamp_block(grid, ix, iy, delta, k):
    if not (0 <= ix < W and 0 <= iy < H): return
    if k <= 1:
        grid[iy, ix] = np.clip(grid[iy, ix] + delta, L_MIN, L_MAX); return
    r = k // 2
    y0 = max(0, iy - r); y1 = min(H, iy + r + 1)
    x0 = max(0, ix - r); x1 = min(W, ix + r + 1)
    grid[y0:y1, x0:x1] = np.clip(grid[y0:y1, x0:x1] + delta, L_MIN, L_MAX)

def update_map_batch(batch, grid, l_free=L_FREE, l_occ=L_OCC):
    with pose_lock:
        xr, yr, yaw_deg = POSE_X, POSE_Y, POSE_YAW_DEG
        if not ORIGIN_SET: return
        xr_rel = xr - ORIGIN_X; yr_rel = yr - ORIGIN_Y
    cosc = math.cos; sinc = math.sin; rad = math.radians
    for (a_deg, d_mm, qv) in batch:
        if qv < Q_MIN: continue
        r = d_mm / 1000.0
        if r < R_MIN or r > R_MAX: continue
        # 修复：右手系雷达角度计算，确保前方为Y正半轴
        a_use = -a_deg if not USE_CLOCKWISE_SENSOR_ANGLE else a_deg
        # 右手系：雷达扫描角度 = 机器人朝向 + 雷达相对角度 + 90度（前方为Y正半轴）
        th = rad((yaw_deg + a_use + 90.0) % 360.0)
        xh = xr + r * cosc(th); yh = yr + r * sinc(th)
        xh_rel = xh - ORIGIN_X; yh_rel = yh - ORIGIN_Y
        ix0, iy0 = world_to_cell(xr_rel, yr_rel)
        ix1, iy1 = world_to_cell(xh_rel, yh_rel)
        if not (0 <= ix0 < W and 0 <= iy0 < H): continue
        first = True
        for x, y in bresenham(ix0, iy0, ix1, iy1):
            if not (0 <= x < W and 0 <= y < H): break
            if x == ix1 and y == iy1:
                _stamp_block(grid, x, y, l_occ, K_HIT)
            else:
                if first: first = False; continue
                _stamp_block(grid, x, y, l_free, K_FREE)

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
            if now - t > SETTLE_WINDOW_SEC: break
            earliest = (t,x,y,yaw)
        if earliest is None: earliest = pose_hist[0]
        _,x0,y0,y0aw = earliest; _,x1,y1,y1aw = latest
    d = math.hypot(x1-x0, y1-y0)
    dyaw = abs(((y1aw - y0aw + 180) % 360) - 180)
    return (d <= SETTLE_EPS_M) and (dyaw <= SETTLE_YAW_EPS_DEG)

# ==== 主程序：SCAN10 → MOVE1.5 → SCAN10 → LEFT → MOVE0.7 → SCAN10 ====
def main():
    # 线程
    t_rx = threading.Thread(target=reader, daemon=True); t_rx.start()
    t_poll = threading.Thread(target=poller, daemon=True); t_poll.start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(7,7))
    img = ax.imshow(np.full((H,W), 0.5, dtype=np.float32),
                    vmin=0.0, vmax=1.0, cmap='gray', origin='upper')
    ax.set_title("Occupancy Grid (prob)"); ax.set_xticks([]); ax.set_yticks([])
    draw_axes_overlay(ax)
    path_line,   = ax.plot([], [], color='cyan',   lw=1.2, alpha=0.9, zorder=6)
    heading_line,= ax.plot([], [], color='yellow', lw=1.5, alpha=0.9, zorder=7)

    total_recv_pts = 0
    last_plot = time.time(); BATCH_MAX = 30000; PLOT_HZ = 8.0

    # 状态机
    state = "SCAN1"
    mapping_active = True
    scan_deadline = time.time() + SCAN_SEC

    # 动作控制
    ref_pose_for_move = None
    move_target = 0.0
    move_cmd_sent = False
    stop_cmd_sent = False
    stop_cmd_time = 0.0
    turn_cmd_sent = False
    turn_cmd_time = 0.0

    try:
        while True:
            # 取批次
            batch = []
            while len(batch) < BATCH_MAX:
                try: batch.extend(Q_POINTS.get_nowait())
                except queue.Empty: break

            # 只在 mapping_active 时更新地图；运动阶段仅耗尽队列不写图
            if mapping_active and batch:
                used = sum(1 for a,d,q in batch if (q >= Q_MIN and (R_MIN <= d/1000.0 <= R_MAX)))
                update_map_batch(batch, L_live)
                total_recv_pts += len(batch)

            now = time.time()

            # ====== 状态机 ======
            if state.startswith("SCAN"):
                # 扫描到时：提交当轮→清空增量→进入下一步
                if now >= scan_deadline:
                    # 历史只增不减
                    np.maximum(L_master, L_live, out=L_master)
                    L_live[:] = 0.0
                    mapping_active = False
                    if state == "SCAN1":
                        # 下一步：MOVE1
                        state = "MOVE1"
                        ref_pose_for_move = get_pose_rel()
                        move_target = MOVE1_M
                        move_cmd_sent = stop_cmd_sent = False
                        print("[FSM] SCAN1 done → MOVE1 1.5m")
                    elif state == "SCAN2":
                        state = "TURN_LEFT"
                        turn_cmd_sent = False
                        print("[FSM] SCAN2 done → TURN_LEFT")
                    elif state == "SCAN3":
                        print("[FSM] SCAN3 done → DONE")
                        break

            elif state == "MOVE1" or state == "MOVE2":
                # 发 M 距离
                if not move_cmd_sent:
                    send_ascii(f"M{move_target:.2f}\n")
                    move_cmd_sent = True
                    stop_cmd_sent = False
                    stop_cmd_time = 0.0
                    print(f"[CMD] Move {move_target:.2f} m issued.")
                # 监控距离/停止/稳定
                cur_rel = get_pose_rel()
                if ref_pose_for_move and cur_rel:
                    dist = moved_since((ref_pose_for_move[0], ref_pose_for_move[1]),
                                       (cur_rel[0],            cur_rel[1]))
                    if dist >= move_target and STOP_AFTER_MOVE and not stop_cmd_sent:
                        send_ascii('S'); stop_cmd_sent = True; stop_cmd_time = now
                        print("[CMD] Stop issued after move target reached.")
                    stopped_stable = is_settled(now)
                    extra_ok = (not STOP_AFTER_MOVE) or (stop_cmd_sent and (now - stop_cmd_time) >= POST_STOP_SETTLE_SEC)
                    if dist >= move_target and stopped_stable and extra_ok:
                        # 下一步：扫描10s
                        mapping_active = True
                        scan_deadline = now + SCAN_SEC
                        if state == "MOVE1":
                            state = "SCAN2"; print("[FSM] MOVE1 done → SCAN2 10s")
                        else:
                            state = "SCAN3"; print("[FSM] MOVE2 done → SCAN3 10s")

            elif state == "TURN_LEFT":
                if not turn_cmd_sent:
                    send_ascii("Lp")  # 原地左转90°
                    turn_cmd_sent = True
                    turn_cmd_time = now
                    print("[CMD] Pivot Left issued.")
                else:
                    # 等待稳定
                    if is_settled(now) and (now - turn_cmd_time) >= 0.3:
                        # 下一步：MOVE2
                        state = "MOVE2"
                        ref_pose_for_move = get_pose_rel()
                        move_target = MOVE2_M
                        move_cmd_sent = stop_cmd_sent = False
                        mapping_active = False
                        print("[FSM] TURN_LEFT done → MOVE2 0.7m")

            # ====== 绘图 ======
            if now - last_plot >= 1.0 / PLOT_HZ:
                L_show = np.maximum(L_master, L_live)
                raw_prob = prob_from_logodds(L_show)
                img.set_data(fuse_lines_from_prob(raw_prob))
                with pose_lock:
                    pts_idx = [world_to_cell(x, y) for (x, y) in path_pts]
                    if pts_idx:
                        xs, ys = zip(*pts_idx); path_line.set_data(xs, ys)
                    rel_pose = (POSE_X-ORIGIN_X, POSE_Y-ORIGIN_Y, POSE_YAW_DEG) if ORIGIN_SET else None
                if rel_pose:
                    px0, py0 = world_to_cell(rel_pose[0], rel_pose[1])
                    # 修复：右手系heading方向计算
                    th = math.radians((rel_pose[2] + 90.0) % 360.0)
                    x1 = rel_pose[0] + 0.5 * math.cos(th); y1 = rel_pose[1] + 0.5 * math.sin(th)
                    px1, py1 = world_to_cell(x1, y1)
                    heading_line.set_data([px0, px1], [py0, py1])
                ax.set_xlabel(
                    f"state={state} scan_remain={max(0.0, scan_deadline-time.time()):.1f}s "
                    f"recv_pts={total_recv_pts} queue={Q_POINTS.qsize()} "
                    f"robot=({ROBOT_REL_X:.2f},{ROBOT_REL_Y:.2f})m"
                )
                plt.pause(0.001); last_plot = now

            time.sleep(0.0008)

        print("[END] Sequence completed.")
        plt.ioff(); plt.show()

    except KeyboardInterrupt:
        pass
    finally:
        STOP.set()
        time.sleep(0.5)
        plt.ioff(); plt.show()

if __name__ == "__main__":
    main()
