#include "Motor.h"
#include "Bluetooth.h"
#include "IMU.h"
#include "PID.h" // 确保包含了PID.h
#include <stdio.h> 
#include <string.h> 

// --- 新增：死区补偿阈值 ---
// V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V V
// V V V   请用您实际测量的【启动阈值PWM】替换下面的示例值！  V V V
// 调整死区补偿以平衡左右轮速度
#define LEFT_PWM_DEADZONE  54 
#define RIGHT_PWM_DEADZONE 54
// A A A A A A A A A A A A A A A A A A A A A A A A A A A A A A A A A A A

// --- 新增：速度补偿系数 ---
// 用于微调左右轮速度差异，正值表示右轮需要加速
#define SPEED_COMPENSATION_FACTOR 0.95f  // 左轮速度补偿系数

// --- 新增：转向补偿系数 ---
// 用于修正右转角度不足的问题
#define RIGHT_TURN_COMPENSATION 1.3f    // 右转补偿系数 

// PID controller instances
PID_TypeDef pidLeft, pidRight, pidYaw;

// Task period definition
#define TASK_MOTOR_PERIOD 40

// System variables
volatile uint32_t motorCounter = 0;

// Motor PWM constants for simple movement functions
#define MOTOR_PWM 200

// Forward declarations
void MotorSpeed_Control(float left_target_speed, float right_target_speed);
void Motor_Control(float base_speed, float target_yaw, float current_yaw);

// 控制是否允许静止时进行原地转向（默认关闭，避免无指令自旋）
static volatile bool g_rotate_active = false;
void Motor_SetRotateActive(bool enable) { g_rotate_active = enable; }

// 震荡检测和抑制
static float last_yaw_output = 0.0f;
static uint32_t oscillation_count = 0;
#define OSCILLATION_THRESHOLD 5    // 连续震荡次数阈值
#define OSCILLATION_DETECTION_THRESHOLD 100.0f  // 震荡检测阈值

// 左右轮速度校准系数（用于补偿电机特性差异）
// 如果左轮快，降低左轮系数或提高右轮系数
static float left_wheel_calibration = 0.95f;   // 左轮校准系数（左轮快，降低到0.95）
static float right_wheel_calibration = 1.0f;   // 右轮校准系数

void Motor_Init(void) {
    pinMode(L_A, OUTPUT);
    pinMode(L_B, OUTPUT);
    pinMode(R_A, OUTPUT);
    pinMode(R_B, OUTPUT);
    Motor_Stop();
    
    // PID Gains for the Angle (Yaw) Loop - 低震荡优化参数
    float IMUKp = 45.0f;    // 降低比例增益，减少震荡
    float IMUKi = 0.2f;     // 进一步减小积分项，避免短距离震荡
    float IMUKd = 30.0f;    // 增加微分增益，提供更强的阻尼
    
    // Initialize Speed Loop PIDs (inner loop)
    // 最后一个参数 (0.0f) 禁用了积分分离功能
    // 调整PID参数以平衡左右轮速度
    PID_Init(&pidLeft,  0.0400, 0.0070, 0.008, -200, 200, 0.0f);
    PID_Init(&pidRight, 0.0400, 0.0070, 0.008, -200, 200, 0.0f);
    
    // Initialize Angle Loop PID (outer loop)
    // 最后一个参数 (2.0f) 启用积分分离，阈值为2度，减少短距离震荡
    PID_Init(&pidYaw, IMUKp, IMUKi, IMUKd, -1000, 1000, 2.0f);

    // Set initial setpoints to 0
    pidLeft.setpoint = 0;
    pidRight.setpoint = 0;
    pidYaw.setpoint = 0;
}

void Motor_Backward(void) {
    analogWrite(L_A, MOTOR_PWM);
    analogWrite(L_B, 0);
    analogWrite(R_A, MOTOR_PWM);
    analogWrite(R_B, 0);
}

void Motor_Forward(void) {
    analogWrite(L_A, 0);
    analogWrite(L_B, MOTOR_PWM);
    analogWrite(R_A, 0);
    analogWrite(R_B, MOTOR_PWM); // 已修正
}

void Motor_Right(void) {
    analogWrite(L_A, MOTOR_PWM);
    analogWrite(L_B, 0);
    analogWrite(R_A, 0);
    analogWrite(R_B, MOTOR_PWM);
}

void Motor_Left(void) {
    analogWrite(L_A, 0);
    analogWrite(L_B, MOTOR_PWM);
    analogWrite(R_A, MOTOR_PWM);
    analogWrite(R_B, 0);
}

void Motor_Stop(void) {
    analogWrite(L_A, 0);
    analogWrite(L_B, 0);
    analogWrite(R_A, 0);
    analogWrite(R_B, 0);
}

void Motor_Set_PWM(int left_pwm, int right_pwm) {
    // --- 死区补偿逻辑 ---
    if (left_pwm > 0) {
        left_pwm = map(left_pwm, 1, 200, LEFT_PWM_DEADZONE, 200);
    } else if (left_pwm < 0) {
        left_pwm = map(left_pwm, -1, -200, -LEFT_PWM_DEADZONE, -200);
    }

    if (right_pwm > 0) {
        right_pwm = map(right_pwm, 1, 200, RIGHT_PWM_DEADZONE, 200);
    } else if (right_pwm < 0) {
        right_pwm = map(right_pwm, -1, -200, -RIGHT_PWM_DEADZONE, -200);
    }
    // --- 死区补偿逻辑结束 ---

    int left_abs_pwm = constrain(abs(left_pwm), 0, 200);
    int right_abs_pwm = constrain(abs(right_pwm), 0, 200);

    if (left_pwm >= 0) {
        analogWrite(L_A, 0);
        analogWrite(L_B, left_abs_pwm);
    } else {
        analogWrite(L_A, left_abs_pwm);
        analogWrite(L_B, 0);
    }
    
    if (right_pwm >= 0) {
        analogWrite(R_A, 0);
        analogWrite(R_B, right_abs_pwm);
    } else {
        analogWrite(R_A, right_abs_pwm);
        analogWrite(R_B, 0);
    }
}

// --- Inner Loop: Speed Controller ---
void MotorSpeed_Control(float left_target_speed, float right_target_speed) {
    // 应用速度补偿系数来平衡左右轮速度
    float compensated_left_target = left_target_speed * SPEED_COMPENSATION_FACTOR;
    
    pidLeft.setpoint = compensated_left_target;
    pidRight.setpoint = right_target_speed;

    float leftSpeed = getLeftSpeedPPS();
    float rightSpeed = getRightSpeedPPS();

    float leftOutput = PID_Calculate_Speed(&pidLeft, leftSpeed);
    float rightOutput = PID_Calculate_Speed(&pidRight, rightSpeed);
    
    Motor_Set_PWM((int)leftOutput, (int)rightOutput);
}

// --- Outer Loop + Inner Loop Integration ---
void Motor_Control(float base_speed, float target_yaw, float current_yaw) {
    float angle_error = target_yaw - current_yaw;

    // 如果线速度为0且未显式允许原地转向，则直接清空速度环并置PWM为0
    // 避免速度环对微小测量噪声输出被死区映射放大导致自旋
    if (base_speed == 0.0f && !g_rotate_active) {
        PID_Reset(&pidLeft);
        PID_Reset(&pidRight);
        Motor_Set_PWM(0, 0);
        return;
    }

    // 若处于原地转向模式，达到小误差阈值则自动退出旋转模式
    // 为右转设置更严格的阈值，因为右转通常角度不足
    float turn_threshold = (angle_error < 0) ? 0.3f : 0.5f;  // 右转用更小阈值
    if (base_speed == 0.0f && g_rotate_active && fabsf(angle_error) < turn_threshold) {
        g_rotate_active = false; // 自动清除旋转标志
        MotorSpeed_Control(0.0f, 0.0f);
        
        // 转弯完成后发送角度信息
        float final_yaw = MPU6500_GetYaw();
        Bluetooth_SendString("Turn completed. Final angle: ");
        Bluetooth_SendFloat(final_yaw, 2);
        Bluetooth_SendNewLine();
        return;
    }
    
    pidYaw.setpoint = target_yaw;
    float yaw_adjustment = PID_Calculate_Angle(&pidYaw, current_yaw);

    // 震荡检测和抑制
    float output_diff = fabsf(yaw_adjustment - last_yaw_output);
    if (output_diff > OSCILLATION_DETECTION_THRESHOLD) {
        oscillation_count++;
        if (oscillation_count >= OSCILLATION_THRESHOLD) {
            // 检测到震荡，应用抑制措施
            yaw_adjustment *= 0.5f;  // 降低输出幅度
            oscillation_count = 0;   // 重置计数器
        }
    } else {
        oscillation_count = 0;  // 重置计数器
    }
    last_yaw_output = yaw_adjustment;

    // 应用转向补偿：如果是右转（负的yaw_adjustment），增加补偿
    float compensated_yaw_adjustment = yaw_adjustment;
    if (base_speed == 0.0f && g_rotate_active && yaw_adjustment < 0) {
        // 右转时应用补偿
        compensated_yaw_adjustment = yaw_adjustment * RIGHT_TURN_COMPENSATION;
        // 调试信息：显示补偿应用
        Serial.print("Right turn compensation applied: ");
        Serial.print(yaw_adjustment, 1);
        Serial.print(" -> ");
        Serial.print(compensated_yaw_adjustment, 1);
        Serial.println();
    }

    float left_target_speed = base_speed - compensated_yaw_adjustment;
    float right_target_speed = base_speed + compensated_yaw_adjustment;

    MotorSpeed_Control(left_target_speed, right_target_speed);
}

// --- 已重写：Motor_Tick 现在是一个纯粹的任务执行函数 ---
/**
 * @brief  电机控制主任务
 * @note   此函数由 main.cpp 中的调度器按固定周期调用。
 * 它负责调用更底层的内外环控制函数。
 * @param  speed_target: 目标前进速度
 * @param  yaw_target: 目标偏航角
 * @retval None
 */
void Motor_Tick(float speed_target, float yaw_target) {
    // 定时逻辑已移至 main.cpp 的调度器，这里直接执行核心控制任务
    float yaw_now = MPU6500_GetYaw();
    Motor_Control(speed_target, yaw_target, yaw_now);
}

// 调试函数：打印左右轮速度信息
void Motor_PrintSpeedInfo(void) {
    float leftSpeed = getLeftSpeedPPS();
    float rightSpeed = getRightSpeedPPS();
    float speedDiff = leftSpeed - rightSpeed;
    
    Serial.print("Left: ");
    Serial.print(leftSpeed, 1);
    Serial.print(" PPS, Right: ");
    Serial.print(rightSpeed, 1);
    Serial.print(" PPS, Diff: ");
    Serial.print(speedDiff, 1);
    Serial.print(" PPS");
    
    // 显示PID输出
    Serial.print(", L_PID: ");
    Serial.print(pidLeft.output, 1);
    Serial.print(", R_PID: ");
    Serial.print(pidRight.output, 1);
    
    // 显示转向信息
    if (g_rotate_active) {
        float current_yaw = MPU6500_GetYaw();
        float angle_error = pidYaw.setpoint - current_yaw;
        Serial.print(", Turning, Error: ");
        Serial.print(angle_error, 1);
        Serial.print("°");
    }
    Serial.println();
}