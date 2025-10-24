#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_startup_stability.py — 启动稳定性测试脚本
测试小车启动时是否会出现扭动现象
"""

import time
import argparse
from bluetooth import BTCar

def main():
    ap = argparse.ArgumentParser(description="启动稳定性测试")
    ap.add_argument("--port", "-p", default="COM5", help="串口")
    ap.add_argument("--baud", "-b", type=int, default=9600, help="波特率")
    ap.add_argument("--wait", "-w", type=int, default=10, help="启动后等待时间（秒）")
    args = ap.parse_args()

    car = BTCar(port=args.port, baud=args.baud, read_async=True)
    
    try:
        print("=== 启动稳定性测试 ===")
        print("测试小车启动时是否会出现扭动现象")
        print(f"启动后等待时间: {args.wait}秒")
        print("\n请观察小车启动时的行为...")
        
        # 连接后立即开始观察
        print("\n1. 小车已连接，开始观察启动过程...")
        print("   - 观察是否有扭动现象")
        print("   - 观察是否有异常声音")
        print("   - 观察是否有不正常的振动")
        
        # 等待启动稳定化完成
        print(f"\n2. 等待 {args.wait} 秒观察启动稳定性...")
        for i in range(args.wait):
            print(f"   等待中... {i+1}/{args.wait}")
            time.sleep(1)
        
        # 测试基本命令响应
        print("\n3. 测试基本命令响应...")
        
        print("   测试停止命令...")
        car.stop()
        time.sleep(1)
        
        print("   测试角度查询...")
        car.send_command('Y')
        time.sleep(0.5)
        
        print("   测试位置查询...")
        car.send_command('Q')
        time.sleep(0.5)
        
        print("   测试距离查询...")
        car.send_command('D')
        time.sleep(0.5)
        
        # 测试小幅运动
        print("\n4. 测试小幅运动...")
        print("   短暂前进...")
        car.send_command('F')
        time.sleep(2)
        car.send_command('S')
        time.sleep(1)
        
        print("   测试完成！")
        
        # 评估结果
        print("\n=== 测试评估 ===")
        print("请根据观察结果评估启动稳定性：")
        print("✅ 优秀：启动平稳，无扭动，无异常声音")
        print("⚠️  良好：轻微扭动，但不影响使用")
        print("❌ 需要改进：明显扭动或异常行为")
        
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
        
    finally:
        car.close()
        print("\n测试完成，连接已关闭")

if __name__ == "__main__":
    main()
