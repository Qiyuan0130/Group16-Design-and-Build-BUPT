#ifndef __PID_H
#define __PID_H

#include <Arduino.h>
#include <math.h>

// PID 控制器结构体
typedef struct {
    // PID Gains
    float Kp;
    float Ki;
    float Kd;

    // State Variables
    float setpoint;
    float output;
    
    // Limits
    float output_max;
    float output_min;
    
    // Positional PID Internal Variables
    float integral;
    float last_error;
    
    // --- 高级功能所需变量 ---
    float last_measured;      // 用于“基于测量的微分”
    float integral_threshold; // 用于“积分分离”的误差门限值

} PID_TypeDef;

/**
 * @brief 初始化PID控制器
 * @param pid 指向PID结构体的指针
 * @param Kp  比例系数
 * @param Ki  积分系数
 * @param Kd  微分系数
 * @param output_min 输出最小值
 * @param output_max 输出最大值
 * @param integral_threshold 积分分离的阈值 (如果设为0或负数, 则不启用此功能)
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_min, float output_max, float integral_threshold);

/**
 * @brief 为【速度环】设计的PID计算函数
 * @note  采用“基于测量的微分”来消除微分前冲，优化启动过程。
 * @param pid 指向PID结构体的指针
 * @param measured 当前测量的速度值
 * @return 计算出的PID输出值 (例如PWM)
 */
float PID_Calculate_Speed(PID_TypeDef *pid, float measured);

/**
 * @brief 为【角度环】设计的PID计算函数
 * @note  采用“积分分离”和“抗饱和”来优化大角度转向和稳定性。
 * @param pid 指向PID结构体的指针
 * @param measured 当前测量的角度值
 * @return 计算出的PID输出值 (例如速度修正量)
 */
float PID_Calculate_Angle(PID_TypeDef *pid, float measured);

/**
 * @brief 重置PID控制器的内部状态
 * @note  用于在开始新任务前，清空积分累积和误差历史。
 * @param pid 指向PID结构体的指针
 */
void PID_Reset(PID_TypeDef *pid);

#endif