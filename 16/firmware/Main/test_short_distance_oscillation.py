#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_short_distance_oscillation.py — 短距离震荡测试脚本
专门测试短距离行进时的震荡情况
"""

import time
import argparse
from bluetooth import BTCar

def main():
    ap = argparse.ArgumentParser(description="短距离震荡测试")
    ap.add_argument("--port", "-p", default="COM5", help="串口")
    ap.add_argument("--baud", "-b", type=int, default=9600, help="波特率")
    ap.add_argument("--trials", "-t", type=int, default=5, help="每种距离的测试次数")
    args = ap.parse_args()

    car = BTCar(port=args.port, baud=args.baud, read_async=True)
    
    # 测试距离：短距离和长距离对比
    test_distances = [0.3, 0.5, 1.0, 2.0]
    
    try:
        print("=== 短距离震荡测试 ===")
        print("测试不同距离行进时的震荡情况")
        print(f"每种距离测试 {args.trials} 次")
        
        # 初始化
        car.stop()
        time.sleep(0.5)
        car.radar_tx_off()
        time.sleep(1.0)
        
        results = {}
        
        for distance in test_distances:
            print(f"\n--- 测试距离: {distance}m ---")
            results[distance] = []
            
            for trial in range(args.trials):
                print(f"\n  第 {trial + 1} 次测试:")
                
                # 获取起始角度
                car.send_command('Y')
                time.sleep(0.3)
                start_angle = 0.0  # 这里需要根据实际响应解析
                
                # 执行距离控制
                print(f"    开始行进 {distance}m...")
                car.send_command('M')
                time.sleep(0.2)
                car.send_command(f"{distance}")
                time.sleep(0.2)
                
                # 等待完成
                wait_time = distance * 2.5 + 1  # 根据距离估算等待时间
                print(f"    等待 {wait_time:.1f} 秒...")
                time.sleep(wait_time)
                
                # 获取结束角度
                car.send_command('Y')
                time.sleep(0.3)
                end_angle = 0.0  # 这里需要根据实际响应解析
                
                # 计算角度偏移
                angle_offset = abs(end_angle - start_angle)
                results[distance].append(angle_offset)
                
                print(f"    角度偏移: {angle_offset:.2f}°")
                
                # 短暂停止
                car.stop()
                time.sleep(1.0)
        
        # 分析结果
        print("\n=== 测试结果分析 ===")
        for distance in test_distances:
            if results[distance]:
                avg_offset = sum(results[distance]) / len(results[distance])
                max_offset = max(results[distance])
                min_offset = min(results[distance])
                
                print(f"\n距离 {distance}m:")
                print(f"  平均偏移: {avg_offset:.2f}°")
                print(f"  最大偏移: {max_offset:.2f}°")
                print(f"  最小偏移: {min_offset:.2f}°")
                
                # 评估震荡程度
                if avg_offset < 0.5:
                    print(f"  震荡程度: ✅ 优秀")
                elif avg_offset < 1.0:
                    print(f"  震荡程度: ⚠️  良好")
                elif avg_offset < 2.0:
                    print(f"  震荡程度: ⚠️  可接受")
                else:
                    print(f"  震荡程度: ❌ 需要改进")
        
        # 对比分析
        print("\n=== 对比分析 ===")
        short_distances = [d for d in test_distances if d <= 1.0]
        long_distances = [d for d in test_distances if d > 1.0]
        
        if short_distances and long_distances:
            short_avg = sum([sum(results[d])/len(results[d]) for d in short_distances]) / len(short_distances)
            long_avg = sum([sum(results[d])/len(results[d]) for d in long_distances]) / len(long_distances)
            
            print(f"短距离平均震荡: {short_avg:.2f}°")
            print(f"长距离平均震荡: {long_avg:.2f}°")
            
            if short_avg < long_avg * 1.5:
                print("✅ 短距离震荡控制良好！")
            else:
                print("⚠️  短距离震荡仍需优化")
        
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
        
    finally:
        car.close()
        print("\n测试完成，连接已关闭")

if __name__ == "__main__":
    main()
