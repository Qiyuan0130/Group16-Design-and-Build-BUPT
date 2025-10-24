#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_straight_accuracy.py — 直行精度测试脚本
测试小车直行后是否保持角度稳定
"""

import time
import argparse
from bluetooth import BTCar

def get_angle_from_response(response):
    """从响应中解析角度值"""
    try:
        # 假设响应格式为 "YAW,123.45" 或 "Current angle: 123.45"
        if "YAW," in response:
            return float(response.split("YAW,")[1])
        elif "Current angle:" in response:
            return float(response.split("Current angle:")[1].strip())
        elif "Final angle:" in response:
            return float(response.split("Final angle:")[1].strip())
    except:
        pass
    return None

def main():
    ap = argparse.ArgumentParser(description="直行精度测试")
    ap.add_argument("--port", "-p", default="COM5", help="串口")
    ap.add_argument("--baud", "-b", type=int, default=9600, help="波特率")
    ap.add_argument("--distance", "-d", type=float, default=2.0, help="测试距离（米）")
    ap.add_argument("--trials", "-t", type=int, default=3, help="测试次数")
    args = ap.parse_args()

    car = BTCar(port=args.port, baud=args.baud, read_async=True)
    
    try:
        print("=== 直行精度测试开始 ===")
        print(f"测试距离: {args.distance}米")
        print(f"测试次数: {args.trials}次")
        
        # 初始化
        car.stop()
        time.sleep(0.5)
        car.radar_tx_off()  # 关闭雷达
        time.sleep(1.0)
        
        angle_measurements = []
        
        for trial in range(args.trials):
            print(f"\n--- 第 {trial + 1} 次测试 ---")
            
            # 1. 获取起始角度
            print("1. 获取起始角度...")
            car.send_command('Y')
            time.sleep(0.5)
            start_angle = get_angle_from_response(car.get_last_response())
            if start_angle is None:
                print("   ❌ 无法获取起始角度")
                continue
            print(f"   起始角度: {start_angle:.2f}°")
            
            # 2. 执行直行
            print("2. 执行直行...")
            car.send_command('M')
            time.sleep(0.2)
            car.send_command(f"{args.distance}")
            time.sleep(0.2)
            
            # 等待直行完成
            wait_time = args.distance * 2 + 2  # 根据距离估算等待时间
            print(f"   等待 {wait_time} 秒...")
            time.sleep(wait_time)
            
            # 3. 获取结束角度
            print("3. 获取结束角度...")
            car.send_command('Y')
            time.sleep(0.5)
            end_angle = get_angle_from_response(car.get_last_response())
            if end_angle is None:
                print("   ❌ 无法获取结束角度")
                continue
            print(f"   结束角度: {end_angle:.2f}°")
            
            # 4. 计算角度偏移
            angle_offset = end_angle - start_angle
            # 处理角度环绕
            if angle_offset > 180:
                angle_offset -= 360
            elif angle_offset < -180:
                angle_offset += 360
                
            print(f"   角度偏移: {angle_offset:.2f}°")
            angle_measurements.append(abs(angle_offset))
            
            # 5. 角度校正（如果需要）
            if abs(angle_offset) > 2.0:  # 如果偏移超过2度
                print("4. 执行角度校正...")
                car.send_command('C')
                time.sleep(1.0)
                print("   角度已校正")
            
            # 6. 短暂停止
            car.stop()
            time.sleep(1.0)
        
        # 统计结果
        print("\n=== 测试结果统计 ===")
        if angle_measurements:
            avg_offset = sum(angle_measurements) / len(angle_measurements)
            max_offset = max(angle_measurements)
            min_offset = min(angle_measurements)
            
            print(f"平均角度偏移: {avg_offset:.2f}°")
            print(f"最大角度偏移: {max_offset:.2f}°")
            print(f"最小角度偏移: {min_offset:.2f}°")
            
            if avg_offset < 1.0:
                print("✅ 直行精度优秀！")
            elif avg_offset < 2.0:
                print("⚠️  直行精度良好")
            else:
                print("❌ 直行精度需要改进")
        else:
            print("❌ 没有有效的测试数据")
            
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
        
    finally:
        car.close()
        print("\n测试完成，连接已关闭")

if __name__ == "__main__":
    main()
