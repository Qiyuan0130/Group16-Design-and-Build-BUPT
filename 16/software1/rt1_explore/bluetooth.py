#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bt_car.py — 蓝牙串口控制小车（匹配固件: F/S, Lp/Rp, M<distance>, Z/D/E/e, Y/Q）
- 前进定速: F
- 停止: S
- 原地左转90°: Lp
- 原地右转90°: Rp
- 前进指定距离(米, 0–10): M<distance>（以换行结束）
- 里程复位: Z
- 查询里程: D
- 雷达TX开关: E/e
- 查询航向: Y
- 查询XY: Q
- 掉头: 两次 Lp

注意：不发送单字符 L/R/T。掉头请调用 uturn() 或两次 pivot_left()。
依赖: pip install pyserial
"""

import sys
import time
import argparse
import threading
from typing import Optional, Iterable, List
import serial

# 固件的“原子令牌”（不含带参数的 M）
ALLOWED_TOKENS = {"F", "S", "Lp", "Rp", "Z", "D", "E", "e", "Y", "Q"}

# 两次 Lp 的默认间隔
UTURN_GAP_SEC = 0.10


class BTCar:
    def __init__(self, port: str, baud: int = 9600, timeout: float = 0.2,
                 eol: str = "\n", read_async: bool = True):
        self.port = port
        self.baud = baud
        self.eol = eol
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        # 稳定化
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self._stop_reader = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        if read_async:
            self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._reader_thread.start()

    def close(self):
        self._stop_reader.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=0.5)
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _read_loop(self):
        # 打印设备返回
        while not self._stop_reader.is_set():
            try:
                line = self.ser.readline()
                if line:
                    try:
                        print(f"[RX] {line.decode(errors='ignore').rstrip()}")
                    except Exception:
                        print(f"[RX] {line}")
            except Exception:
                time.sleep(0.05)

    # === 低层发送 ===
    def _write(self, payload: str):
        data = (payload + self.eol).encode("ascii", errors="ignore")
        n = self.ser.write(data)
        self.ser.flush()
        print(f"[TX] {repr(payload)} ({n} bytes)")

    def send_token(self, token: str):
        """发送单个固件令牌，如 'F'、'S'、'Lp'、'Rp'、'Z' 等。"""
        token = token.strip()
        if token not in ALLOWED_TOKENS:
            raise ValueError(f"非法令牌: {token}，允许: {sorted(ALLOWED_TOKENS)}")
        self._write(token)

    def send_sequence(self, tokens: Iterable[str], gap_sec: float = 0.0):
        """按序发送多个令牌；支持混入形如 'M0.50' 的距离命令。"""
        seq: List[str] = list(tokens)
        for i, t in enumerate(seq):
            t = t.strip()
            if not t:
                continue
            if t == "U":
                self.uturn()
            elif t[0] in ("M", "m"):
                # 形如 M0.5 或 M 0.5 都支持
                val = t[1:].strip()
                if not val:
                    raise ValueError("M 命令缺少距离参数")
                self._write(f"M{val}")
            else:
                self.send_token(t)
            if gap_sec > 0.0 and i + 1 < len(seq):
                time.sleep(gap_sec)

    # === 语义动作封装 ===
    def forward(self): self.send_token("F")
    def stop(self):    self.send_token("S")

    def pivot_left(self):   self.send_token("Lp")  # 原地左转90°
    def pivot_right(self):  self.send_token("Rp")  # 原地右转90°

    def uturn(self, gap_sec: float = UTURN_GAP_SEC):
        """180° 掉头，用两次 Lp 实现。"""
        self.send_token("Lp")
        time.sleep(max(0.0, gap_sec))
        self.send_token("Lp")

    def move_meters(self, meters: float):
        """前进指定距离（米，(0,10]）。固件以换行作为参数结束。"""
        if not (0.0 < meters <= 10.0):
            raise ValueError("距离需在 (0, 10] 米")
        self._write(f"M{meters:.3f}")

    # 设备辅助指令
    def reset_distance(self): self.send_token("Z")
    def query_distance(self): self.send_token("D")
    def radar_tx_on(self):    self.send_token("E")
    def radar_tx_off(self):   self.send_token("e")
    def query_yaw(self):      self.send_token("Y")
    def query_xy(self):       self.send_token("Q")


# ======================
# 命令行接口（保留基础发送，不含任何测试/演示）
# ======================
def main():
    p = argparse.ArgumentParser(description="蓝牙串口控制小车（F/S, Lp/Rp, M<d>, Z/D/E/e, Y/Q）")
    p.add_argument("--port", "-p", default="COM5",
                   help="串口名，Windows 如 COM12；Linux 如 /dev/rfcomm0")
    p.add_argument("--baud", "-b", type=int, default=9600, help="波特率")
    p.add_argument("--eol", choices=["LF", "CRLF", "NONE"], default="LF", help="结尾符")
    # 发送令牌/序列：支持 F,S,Lp,Rp,Z,D,E,e,Y,Q,U 以及 M0.50；多令牌逗号分隔
    p.add_argument("--cmd", "-c",
                   help="发送后退出，如: Z,M0.50,Lp,M0.50,U,M0.50,S")
    p.add_argument("--no-read", action="store_true", help="不异步读取回传数据")
    args = p.parse_args()

    eol_map = {"LF": "\n", "CRLF": "\r\n", "NONE": ""}
    eol = eol_map[args.eol]

    try:
        car = BTCar(args.port, args.baud, eol=eol, read_async=not args.no_read)
    except Exception as e:
        print(f"打开串口失败: {e}")
        sys.exit(1)

    try:
        if args.cmd:
            seq = [s.strip() for s in args.cmd.split(",") if s.strip()]
            car.send_sequence(seq, gap_sec=0.1)
            return

        # 交互 REPL
        print("已连接。输入: F,S,Lp,Rp,Z,D,E,e,Y,Q,U 或 M<距离米>。q 退出。")
        while True:
            s = input("> ").strip()
            if not s:
                continue
            if s.lower() in {"q", "quit", "exit"}:
                break
            try:
                toks = [t.strip() for t in s.split(",") if t.strip()]
                car.send_sequence(toks, gap_sec=0.1)
            except Exception as e:
                print(f"发送失败: {e}")
    finally:
        car.close()


if __name__ == "__main__":
    main()
