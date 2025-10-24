// Radar.h
#pragma once
#include <Arduino.h>

/* ===== 过滤阈值（可在编译时通过 -D 覆盖）===== */
#ifndef RADAR_MIN_QUALITY
#define RADAR_MIN_QUALITY 0        // 放宽质量阈值，避免角度缺口
#endif
#ifndef RADAR_MIN_DIST_MM
#define RADAR_MIN_DIST_MM 1.0f     // 最小距离(mm)
#endif
#ifndef RADAR_MAX_DIST_MM
#define RADAR_MAX_DIST_MM 6000.0f  // 最大距离(mm)
#endif

// 全局 UART 实例（Radar.cpp 定义，F446RE: UART5 RX=PD2, TX=PC12）
extern HardwareSerial RPSerial;

void Radar_init(void);
// 读取并解析标准采样节点；toSerial=true 时发送通过过滤的点
void Radar_read(bool toSerial = true);

// 运行时控制：是否通过蓝牙发送雷达数据
void Radar_EnableTx(bool enable);
bool Radar_IsTxEnabled(void);

