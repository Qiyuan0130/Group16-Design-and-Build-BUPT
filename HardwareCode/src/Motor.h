#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "PID.h"
#include "Encoder.h"

// --- 新增代码：使用 extern 关键字声明全局变量 ---
// 这会告诉其他包含了 Motor.h 的文件 (比如 main.cpp)，
// 这些变量是存在的，但它们的实体定义在别处 (Motor.cpp)。
extern PID_TypeDef pidLeft;
extern PID_TypeDef pidRight;
extern PID_TypeDef pidYaw;
// --- 新增结束 ---

// Motor control pin definitions
#define L_A PC8  
#define L_B PB10
#define R_A PC7  
#define R_B PB3  

// Public function declarations
void Motor_Init(void);
void Motor_Forward(void);
void Motor_Backward(void);
void Motor_Right(void);
void Motor_Left(void);
void Motor_Stop(void);

// New function to set motor PWM and direction
void Motor_Set_PWM(int left_pwm, int right_pwm);

// PID and serial logging functions (now part of Motor module)
void Motor_Tick(float speed_target, float yaw_target);
void MotorSpeed_Control(float left_target_speed, float right_target_speed);
void Motor_Control(float base_speed, float target_yaw, float current_yaw);

// 控制是否允许"静止时原地转向"
void Motor_SetRotateActive(bool enable);

// 调试函数：获取左右轮速度信息
void Motor_PrintSpeedInfo(void);

// External declarations for PID variables
extern PID_TypeDef pidLeft, pidRight;
extern PID_TypeDef pidNewParams;

#endif // MOTOR_H
