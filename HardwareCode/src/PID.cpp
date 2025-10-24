#include "PID.h"

// PID 初始化函数
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float output_min, float output_max, float integral_threshold) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0;
    pid->output = 0;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral = 0;
    pid->last_error = 0;
    pid->last_measured = 0; // 初始化“上一次测量值”
    pid->integral_threshold = integral_threshold;
}

// PID 重置函数
void PID_Reset(PID_TypeDef *pid) {
    pid->integral = 0;
    pid->last_error = 0;
    pid->last_measured = 0;
    pid->output = 0;
}


// --- 速度环使用的PID计算函数 (采用【基于测量的微分】) ---
float PID_Calculate_Speed(PID_TypeDef *pid, float measured) {
    // --- 比例项 P ---
    float error = pid->setpoint - measured;
    float p_term = pid->Kp * error;

    // --- 积分项 I ---
    // 速度环直接累积积分
    pid->integral += error;
    float i_term = pid->Ki * pid->integral;

    // --- 微分项 D (基于测量值) ---
    // 微分项基于【测量值】的变化，而不是【误差】的变化，以消除微分前冲
    // 它带有一个负号，起到对速度剧烈变化的阻尼作用
    float d_term = -pid->Kd * (measured - pid->last_measured);

    // 更新“上一次测量值”，为下一次计算做准备
    pid->last_measured = measured;
    
    // PID输出 = P + I + D
    pid->output = p_term + i_term + d_term;
    
    // 输出限幅处理
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    }
    if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    
    return pid->output;
}


// --- 角度环使用的PID计算函数 (采用【积分分离】和【抗饱和】) ---
float PID_Calculate_Angle(PID_TypeDef *pid, float measured) {
    // 1. 计算当前误差
    float error = pid->setpoint - measured;
    
    // 2. --- 【至关重要的一步：对角度误差进行环绕处理】 ---
    //    确保控制器总是走最短的路径来修正角度
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }
    
    // --- 3. 积分项 I (优化积分分离逻辑，减少震荡) ---
    // 对于角度控制，使用更保守的积分分离策略
    if (pid->integral_threshold > 0) {
        // 更严格的条件：只有在误差很小且误差变化很小时才累加积分
        if (fabsf(error) < pid->integral_threshold && 
            fabsf(error - pid->last_error) < pid->integral_threshold * 0.3f) {
            pid->integral += error;
        }
        // 更严格的积分限幅，防止积分项过度累积导致震荡
        if (fabsf(pid->integral) > 50.0f) {
            pid->integral = (pid->integral > 0) ? 50.0f : -50.0f;
        }
    } else {
        // 如果阈值设为0或更小，则总是累加积分
        pid->integral += error;
    }
    
    // --- 4. 微分项 D (基于误差) ---
    // 角度环的目标值变化不频繁，可以使用基于误差的传统微分
    float derivative = error - pid->last_error;
    
    // --- 5. 计算总输出 ---
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // --- 6. 更新“上一次误差”，为下一次计算做准备 ---
    pid->last_error = error;
    
    // --- 7. 输出限幅与积分抗饱和 (Anti-Windup) ---
    // 防止在输出饱和时，积分项仍在无限制地增长
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
        if (error * pid->output > 0 && fabsf(pid->Ki) > 1e-6) {
             pid->integral = (pid->output_max - (pid->Kp * error) - (pid->Kd * derivative)) / pid->Ki;
        }
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
        if (error * pid->output > 0 && fabsf(pid->Ki) > 1e-6) {
             pid->integral = (pid->output_min - (pid->Kp * error) - (pid->Kd * derivative)) / pid->Ki;
        }
    }
    
    return pid->output;
}