#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

/*
 * @brief 编码器数据结构体
 */
typedef struct {
    int pinA;
    int pinB;
    volatile long* count;
    const char* name;
} Encoder_TypeDef;

/*
 * @brief 初始化编码器结构体
 */
void Encoder_Struct_Init(Encoder_TypeDef* encoder, int pinA, int pinB, volatile long* count, const char* name);

/*
 * @brief 初始化编码器硬件和中断 (4倍频)
 */
void Encoder_Init(void);

/*
 * @brief 全局定时器任务，用于计算速度
 */
void Encoder_Tick(void);

/*
 * @brief 获取左轮脉冲速度 (pulses/sec) - 用于PID控制
 * @return 左轮速度 (PPS)
 */
float getLeftSpeedPPS(void);

/*
 * @brief 获取右轮脉冲速度 (pulses/sec) - 用于PID控制
 * @return 右轮速度 (PPS)
 */
float getRightSpeedPPS(void);

/*
 * @brief 获取左轮物理速度 (m/s) - 用于显示或记录
 * @return 左轮速度 (m/s)
 */
float getLeftSpeed(void); // --- 名称已修改 ---

/*
 * @brief 获取右轮物理速度 (m/s) - 用于显示或记录
 * @return 右轮速度 (m/s)
 */
float getRightSpeed(void); // --- 名称已修改 ---

#ifdef __cplusplus
extern "C" {
#endif

/*
 * @brief 获取累计里程（米）
 */
float Encoder_GetLeftDistanceMeters(void);
float Encoder_GetRightDistanceMeters(void);
float Encoder_GetRobotDistanceMeters(void);

/*
 * @brief 获取上一周期增量里程（米）
 */
void Encoder_GetDeltaDistance(float* left_m, float* right_m, float* robot_m);

/*
 * @brief 清零累计里程（不影响速度计算的周期计数）
 */
void Encoder_ResetDistance(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */