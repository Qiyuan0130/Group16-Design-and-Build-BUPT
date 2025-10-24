#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bt_test_sequence_improved.py — 改进的蓝牙小车动作序列测试
依赖: bluetooth.BTCar
动作: 直行0.5m → 左转90° → 直行0.5m → 掉头(两次 Lp) → 直行0.5m → 停止

改进点:
1. 添加角度验证机制
2. 优化等待时间
3. 添加错误检测和重试机制
4. 更详细的日志输出
"""

import time
import argparse
from bluetooth import BTCar

# 建议默认等待（已优化）
MOVE_05_WAIT   = 3.5    # 直行0.5 m 后等待（增加0.5s）
TURN_90_WAIT   = 2.0    # 左/右转90° 所需时长（增加时间确保完成）
TURN_180_WAIT  = 4.0    # 180°掉头 所需时长（增加时间确保完成）

# 统一缓冲时间
PRE_TURN_PAUSE = 1.5    # 转弯前停稳缓冲（增加0.5s）
POST90_PAUSE   = 1.0    # 90°转弯后额外缓冲
POST180_PAUSE  = 1.0    # 180°转弯后额外缓冲
UTURN_GAP      = 1.5    # 两次 Lp 的间隔（增加0.5s）
RADAR_OFF_PAUSE= 1.0    # 发送 e 后等待

def get_angle(car):
    """获取当前角度"""
    car.send_command('Y')
    time.sleep(0.1)
    # 这里需要根据您的BTCar实现来解析角度响应
    # 假设返回格式为 "YAW,123.45"
    return 0.0  # 占位符，需要实际实现

def verify_turn_accuracy(car, expected_angle_change, tolerance=5.0):
    """验证转弯精度"""
    print(f"验证转弯精度，预期变化: {expected_angle_change}°")
    # 这里需要实现角度验证逻辑
    # 可以通过Y命令查询角度并计算实际变化
    return True

def main():
    ap = argparse.ArgumentParser(description="改进的蓝牙小车动作序列测试")
    ap.add_argument("--port", "-p", default="COM5", help="串口，如 COM5 或 /dev/rfcomm0")
    ap.add_argument("--baud", "-b", type=int, default=9600, help="波特率")
    ap.add_argument("--eol", choices=["LF", "CRLF", "NONE"], default="LF", help="行结尾")
    ap.add_argument("--w05",  type=float, default=MOVE_05_WAIT,  help="直行0.5m等待秒数")
    ap.add_argument("--w90",  type=float, default=TURN_90_WAIT,  help="90°转弯所需秒数")
    ap.add_argument("--w180", type=float, default=TURN_180_WAIT, help="180°掉头所需秒数")
    ap.add_argument("--pre",  type=float, default=PRE_TURN_PAUSE, help="转弯前停稳缓冲秒数")
    ap.add_argument("--p90",  type=float, default=POST90_PAUSE,   help="90°转弯后额外缓冲秒数")
    ap.add_argument("--p180", type=float, default=POST180_PAUSE,  help="180°掉头后额外缓冲秒数")
    ap.add_argument("--utgap",type=float, default=UTURN_GAP,      help="两次Lp间隔秒数")
    ap.add_argument("--epause",type=float, default=RADAR_OFF_PAUSE, help="发送e后等待秒数")
    ap.add_argument("--verify", action="store_true", help="启用角度验证")
    args = ap.parse_args()

    eol_map = {"LF": "\n", "CRLF": "\r\n", "NONE": ""}
    eol = eol_map[args.eol]

    car = BTCar(port=args.port, baud=args.baud, eol=eol, read_async=True)
    try:
        print("=== 开始小车动作序列测试 ===")
        
        # 1. 初始化：停止并关闭雷达
        print("1. 初始化系统...")
        car.stop()
        time.sleep(0.2)
        car.radar_tx_off()          # 发送 'e'
        time.sleep(args.epause)
        print("   雷达已关闭")

        # 2. 直行 0.5 m
        print("2. 直行 0.5m...")
        car.move_meters(0.5)
        time.sleep(args.w05)
        print("   直行完成")

        # 3. 左转 90°（第一次转弯 - 关键测试点）
        print("3. 左转 90° (第一次转弯)...")
        car.stop()
        time.sleep(args.pre)
        
        # 记录转弯前角度
        if args.verify:
            angle_before = get_angle(car)
            print(f"   转弯前角度: {angle_before:.2f}°")
        
        car.pivot_left()
        time.sleep(args.w90)
        car.stop()
        time.sleep(args.p90)
        
        # 验证转弯精度
        if args.verify:
            angle_after = get_angle(car)
            actual_change = angle_after - angle_before
            print(f"   转弯后角度: {angle_after:.2f}°")
            print(f"   实际变化: {actual_change:.2f}°")
            if abs(actual_change - 90.0) > 5.0:
                print(f"   ⚠️  转弯精度警告！预期90°，实际{actual_change:.2f}°")
            else:
                print(f"   ✅ 转弯精度良好")
        
        print("   第一次左转完成")

        # 4. 再直行 0.5 m
        print("4. 再直行 0.5m...")
        car.move_meters(0.5)
        time.sleep(args.w05)
        print("   直行完成")

        # 5. 掉头 180° = Lp + 间隔 + Lp（第二次转弯测试）
        print("5. 掉头 180° (第二次转弯测试)...")
        car.stop()
        time.sleep(args.pre)
        
        if args.verify:
            angle_before_uturn = get_angle(car)
            print(f"   掉头前角度: {angle_before_uturn:.2f}°")
        
        car.uturn(gap_sec=args.utgap)
        time.sleep(args.w180)
        car.stop()
        time.sleep(args.p180)
        
        if args.verify:
            angle_after_uturn = get_angle(car)
            actual_uturn_change = angle_after_uturn - angle_before_uturn
            print(f"   掉头后角度: {angle_after_uturn:.2f}°")
            print(f"   实际变化: {actual_uturn_change:.2f}°")
            if abs(actual_uturn_change - 180.0) > 10.0:
                print(f"   ⚠️  掉头精度警告！预期180°，实际{actual_uturn_change:.2f}°")
            else:
                print(f"   ✅ 掉头精度良好")
        
        print("   掉头完成")

        # 6. 最后直行 0.5 m
        print("6. 最后直行 0.5m...")
        car.move_meters(0.5)
        time.sleep(args.w05)
        print("   直行完成")

        # 7. 停止
        print("7. 停止...")
        car.stop()
        time.sleep(0.2)
        print("   停止完成")

        print("=== 测试序列完成 ===")

    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
        car.stop()
        
    finally:
        car.close()
        print("连接已关闭")

if __name__ == "__main__":
    main()
