#include "Bluetooth.h"
#include "IMU.h"
#include "Motor.h"
#include "PID.h"
#include "Encoder.h"
#include "Radar.h"
#include <Wire.h>
#include <HardwareTimer.h>
#include <Arduino.h>
#include <math.h>

/* ===== 任务周期（ms）===== */
#define TASK_MOTOR_PERIOD    40    // 25 Hz
#define TASK_RADAR_PERIOD     1    // 1 kHz 读尽串口
#define TASK_MPU6500_PERIOD  10    // 100 Hz

/* ===== 系统滴答 ===== */
HardwareTimer *mainTimer = NULL;
volatile uint32_t systemTick = 0;

/* ===== 控制量 ===== */
static float current_speed_target = 0.0f;
static float current_yaw_target   = 0.0f;

/* ===== 距离控制 ===== */
static float target_distance_m = 0.0f;        // 目标距离（米）
static float start_distance_m = 0.0f;         // 起始距离（米）
static bool distance_control_active = false;  // 距离控制是否激活

/* ===== 里程计（以启动点为原点）===== */
static float odom_x_m = 0.0f;  // x 坐标（米）
static float odom_y_m = 0.0f;  // y 坐标（米）

/* ===== 速度计算（基于里程计）===== */
static float robot_speed_x = 0.0f;  // X方向速度（m/s）
static float robot_speed_y = 0.0f;  // Y方向速度（m/s）
static float robot_speed_total = 0.0f;  // 总速度（m/s）
static float last_odom_x = 0.0f;  // 上一周期的X坐标
static float last_odom_y = 0.0f;  // 上一周期的Y坐标
static uint32_t last_speed_calc_time = 0;  // 上次速度计算时间

// 获取里程计数据的函数
void GetOdometryData(float* x, float* y) {
    if (x) *x = odom_x_m;
    if (y) *y = odom_y_m;
}

// 获取机器人速度数据的函数
void GetRobotSpeed(float* vx, float* vy, float* v_total) {
    if (vx) *vx = robot_speed_x;
    if (vy) *vy = robot_speed_y;
    if (v_total) *v_total = robot_speed_total;
}

/* ===== 启动稳定化 ===== */
static bool system_startup_complete = false;  // 系统启动完成标志
static uint32_t startup_stable_count = 0;     // 启动稳定计数器
#define STARTUP_STABLE_THRESHOLD 50           // 需要稳定的周期数（约2秒）

/* ===== 动态PID调整 ===== */
static bool short_distance_mode = false;      // 短距离模式标志
#define SHORT_DISTANCE_THRESHOLD 1.0f         // 短距离阈值（米）

/* ===== Flags ===== */
volatile bool runMotorTask = false;
volatile bool runRadarTask = false;
volatile bool runMpuTask   = false;

// 蓝牙命令解析状态：0空闲，1收到'L'等待'p'，2收到'R'等待'p'，3收到'M'等待距离参数
static uint8_t bt_cmd_state = 0;
static char distance_buffer[16];  // 存储距离参数的缓冲区
static uint8_t distance_buffer_index = 0;

/* ===== 主定时器 ===== */
void MainTimer_Init(void) {
    mainTimer = new HardwareTimer(TIM1);
    mainTimer->setPrescaleFactor(180);              // 180MHz/180=1MHz
    mainTimer->setOverflow(1000, MICROSEC_FORMAT);  // 1kHz → 1ms
    mainTimer->attachInterrupt([]() {
        systemTick++;
        if (systemTick % TASK_MOTOR_PERIOD   == 0) runMotorTask = true;
        if (systemTick % TASK_RADAR_PERIOD   == 0) runRadarTask = true;
        if (systemTick % TASK_MPU6500_PERIOD == 0) runMpuTask   = true;
        Encoder_Tick();
    });
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 0);
    mainTimer->resume();
}

void setup(void) {
    Serial.begin(115200);
    Serial.println("系统初始化开始...");

    Bluetooth_Init();
    Encoder_Init();
    MPU6500_Init();
    Motor_Init();
    
    // 仅初始化串口对象
    Radar_init();

    MainTimer_Init();

    Serial.println("等待IMU稳定...");
    delay(2000);
    
    // 获取稳定的初始角度
    float initial_yaw = MPU6500_GetYaw();
    current_yaw_target = initial_yaw;
    
    // 初始化PID控制器状态，确保与当前系统状态一致
    pidYaw.setpoint = initial_yaw;
    pidYaw.last_error = 0;
    pidYaw.last_measured = initial_yaw;
    pidYaw.integral = 0;  // 启动时清空积分项
    pidYaw.output = 0;
    
    // 确保电机停止
    current_speed_target = 0;
    Motor_SetRotateActive(false);
    
    // 初始化启动稳定化
    system_startup_complete = false;
    startup_stable_count = 0;
    
    Serial.println("系统初始化完成");
    Serial.print("初始角度: ");
    Serial.println(initial_yaw);
}

void loop(void) {
    // 非阻塞蓝牙发送：每次循环尽可能将队列写入串口
    Bluetooth_TxPoll();
    // 1) 蓝牙：尽量一次性读尽，避免RX溢出
    for (int _i = 0; _i < 64 && Bluetooth_Available(); ++_i) {
        char cmd = Bluetooth_Read();
        // 先处理多字符命令的后续字符判定
        if (bt_cmd_state == 1) { // 等待 'Lp' 的 'p'
            if (cmd == 'p') {
                // 先停止并等待系统稳定
                current_speed_target = 0;
                Motor_SetRotateActive(false);
                // 短暂延迟让系统稳定
                delay(50);
                
                // 记录当前角度作为基准
                float current_yaw = MPU6500_GetYaw();
                current_yaw_target = current_yaw + 90;
                if (current_yaw_target > 180.0f) current_yaw_target -= 360.0f;
                
                // 设置PID目标并重置（保留一些积分项以避免静态误差）
                pidYaw.setpoint = current_yaw_target;
                // 不完全重置，只重置误差历史，保留部分积分项
                pidYaw.last_error = 0;
                pidYaw.last_measured = current_yaw;
                // 保留积分项的50%，避免完全从0开始
                pidYaw.integral *= 0.5f;
                
                Motor_SetRotateActive(true);
                Bluetooth_SendString("Left turn 90° from ");
                Bluetooth_SendFloat(current_yaw, 2);
                Bluetooth_SendString(" to ");
                Bluetooth_SendFloat(current_yaw_target, 2);
                Bluetooth_SendNewLine();
            }
            bt_cmd_state = 0;
            goto bt_parsed;
        } else if (bt_cmd_state == 2) { // 等待 'Rp' 的 'p'
            if (cmd == 'p') {
                // 先停止并等待系统稳定
                current_speed_target = 0;
                Motor_SetRotateActive(false);
                // 短暂延迟让系统稳定
                delay(50);
                
                // 记录当前角度作为基准
                float current_yaw = MPU6500_GetYaw();
                // 增大右转角度以补偿角度不足问题
                current_yaw_target = current_yaw - 91.5;  // 从90度增加到91.5度
                if (current_yaw_target < -180.0f) current_yaw_target += 360.0f;
                
                // 设置PID目标并重置（保留一些积分项以避免静态误差）
                pidYaw.setpoint = current_yaw_target;
                // 不完全重置，只重置误差历史，保留部分积分项
                pidYaw.last_error = 0;
                pidYaw.last_measured = current_yaw;
                // 保留积分项的50%，避免完全从0开始
                pidYaw.integral *= 0.5f;
                
                Motor_SetRotateActive(true);
                Bluetooth_SendString("Right turn 91.5° from ");
                Bluetooth_SendFloat(current_yaw, 2);
                Bluetooth_SendString(" to ");
                Bluetooth_SendFloat(current_yaw_target, 2);
                Bluetooth_SendNewLine();
            }
            bt_cmd_state = 0;
            goto bt_parsed;
        } else if (bt_cmd_state == 3) { // 等待距离参数的结束符
            if (cmd == '\n' || cmd == '\r') {
                // 距离参数接收完毕，开始解析
                distance_buffer[distance_buffer_index] = '\0';
                float distance = atof(distance_buffer);
                
                if (distance > 0 && distance <= 10.0f) { // 限制距离在0-10米之间
                    start_distance_m = Encoder_GetRobotDistanceMeters();
                    target_distance_m = distance;
                    distance_control_active = true;
                    
                    // 检测是否为短距离模式
                    short_distance_mode = (distance <= SHORT_DISTANCE_THRESHOLD);
                    
                    // 设置前进速度（短距离使用较低速度减少震荡）
                    if (current_speed_target == 0.0f) {
                        PID_Reset(&pidYaw);
                        current_yaw_target = MPU6500_GetYaw();
                    }
                    // 短距离使用较低速度，减少震荡
                    current_speed_target = short_distance_mode ? 1800 : 2500;
                    Motor_SetRotateActive(false);
                    
                    Bluetooth_SendString("Moving forward ");
                    Bluetooth_SendFloat(distance, 2);
                    Bluetooth_SendString(" meters");
                    if (short_distance_mode) {
                        Bluetooth_SendString(" (short distance mode)");
                    }
                    Bluetooth_SendNewLine();
                } else {
                    Bluetooth_SendString("Invalid distance (0-10m)");
                    Bluetooth_SendNewLine();
                }
                
                // 重置状态
                bt_cmd_state = 0;
                distance_buffer_index = 0;
            } else if (cmd >= '0' && cmd <= '9' || cmd == '.') {
                // 继续接收数字字符
                if (distance_buffer_index < sizeof(distance_buffer) - 1) {
                    distance_buffer[distance_buffer_index++] = cmd;
                }
            } else {
                // 无效字符，重置状态
                bt_cmd_state = 0;
                distance_buffer_index = 0;
                Bluetooth_SendString("Invalid distance format");
                Bluetooth_SendNewLine();
            }
            goto bt_parsed;
        }

        switch (cmd) {
            case 'F': {
                // 无论当前状态如何，都重新设定航向角目标
                float current_yaw = MPU6500_GetYaw();
                current_yaw_target = current_yaw;
                
                // 如果是从停止状态开始前进，重置PID
                if (current_speed_target == 0.0f) {
                    PID_Reset(&pidYaw);
                }
                
                current_speed_target = 2500;
                Motor_SetRotateActive(false);
                Bluetooth_SendString("Forward started. Target angle: ");
                Bluetooth_SendFloat(current_yaw_target, 2);
                Bluetooth_SendNewLine();
                break;
            }
            case 'S': {
                current_speed_target = 0;
                
                // 记录停止时的角度，用于校正
                float stop_yaw = MPU6500_GetYaw();
                current_yaw_target = stop_yaw;  // 更新目标角度为当前角度
                
                // 重置速度环PID，但保留角度环的积分项以维持角度稳定性
                PID_Reset(&pidLeft); 
                PID_Reset(&pidRight);
                // 不完全重置角度PID，只重置误差历史
                pidYaw.last_error = 0;
                pidYaw.last_measured = stop_yaw;
                // 保留积分项的一部分，避免角度漂移
                pidYaw.integral *= 0.8f;
                
                Motor_SetRotateActive(false);
                Bluetooth_SendString("Stopped. Current angle: ");
                Bluetooth_SendFloat(stop_yaw, 2);
                Bluetooth_SendNewLine();
                break;
            }
            case 'L':
                bt_cmd_state = 1; // 等待 'p'
                break;
            case 'R':
                bt_cmd_state = 2; // 等待 'p'
                break;
            case 'M':
                bt_cmd_state = 3; // 等待距离参数
                distance_buffer_index = 0;
                Bluetooth_SendString("Enter distance (0-10m): ");
                Bluetooth_SendNewLine();
                break;
            case 'Z':
                Encoder_ResetDistance();
                Bluetooth_SendString("Distance reset.");
                Bluetooth_SendNewLine();
                break;
            case 'D': {
                float d = Encoder_GetRobotDistanceMeters();
                Bluetooth_SendString("Distance(m): ");
                Bluetooth_SendFloat(d, 3);
                Bluetooth_SendNewLine();
                break;
            }
            case 'E': // Enable radar TX
                Radar_EnableTx(true);
                Bluetooth_SendString("RADAR_TX_ON");
                Bluetooth_SendNewLine();
                break;
            case 'e': // Disable radar TX
                Radar_EnableTx(false);
                Bluetooth_SendString("RADAR_TX_OFF");
                Bluetooth_SendNewLine();
                break;
            case 'Y': {
                float yaw = MPU6500_GetYaw();
                Bluetooth_SendString("YAW,");
                Bluetooth_SendFloat(yaw, 2);
                Bluetooth_SendNewLine();
                break;
            }
            case 'Q': {
                char buf[64];
                char xs[16], ys[16];
                dtostrf(odom_x_m, 0, 3, xs);
                dtostrf(odom_y_m, 0, 3, ys);
                snprintf(buf, sizeof(buf), "XY %s %s", xs, ys);
                Bluetooth_SendString(buf);
                Bluetooth_SendNewLine();
                break;
            }
            case 'C': { // 角度校正命令 - 将当前IMU读数设为目标角度
                float correct_yaw = MPU6500_GetYaw();
                current_yaw_target = correct_yaw;
                
                // 智能重置：保留部分积分项以避免静态误差
                pidYaw.last_error = 0;
                pidYaw.last_measured = correct_yaw;
                // 保留积分项的30%，用于消除静态误差
                pidYaw.integral *= 0.3f;
                
                Bluetooth_SendString("Yaw corrected to current reading: ");
                Bluetooth_SendFloat(current_yaw_target, 2);
                Bluetooth_SendNewLine();
                break;
            }
            // V命令已移除 - 不再显示轮速信息
            // T命令已移除 - 不再需要转向调试功能
            case 'W': { // 机器人速度命令 - 显示基于里程计的速度
                float vx, vy, v_total;
                GetRobotSpeed(&vx, &vy, &v_total);
                Bluetooth_SendString("Robot Speed - Vx: ");
                Bluetooth_SendFloat(vx, 3);
                Bluetooth_SendString(" Vy: ");
                Bluetooth_SendFloat(vy, 3);
                Bluetooth_SendString(" Total: ");
                Bluetooth_SendFloat(v_total, 3);
                Bluetooth_SendString(" m/s");
                Bluetooth_SendNewLine();
                break;
            }
        }
bt_parsed:
        ;
    }

    // 2) MPU
    if (runMpuTask) { 
        runMpuTask = false; 
        MPU6500_Tick(); 
    }

    // 3) 电机
    if (runMotorTask) { 
        runMotorTask = false; 
        
        // 启动稳定化逻辑
        if (!system_startup_complete) {
            startup_stable_count++;
            if (startup_stable_count >= STARTUP_STABLE_THRESHOLD) {
                system_startup_complete = true;
                Serial.println("系统启动稳定化完成");
            } else {
                // 启动期间保持停止状态，避免扭动
                current_speed_target = 0;
                Motor_SetRotateActive(false);
                // 更新PID状态以保持同步
                float current_yaw = MPU6500_GetYaw();
                pidYaw.last_measured = current_yaw;
                pidYaw.last_error = 0;
                return; // 跳过正常控制逻辑
            }
        }
        
        // 距离控制逻辑
        if (distance_control_active) {
            float current_distance = Encoder_GetRobotDistanceMeters() - start_distance_m;
            if (current_distance >= target_distance_m) {
                // 达到目标距离，停止小车
                current_speed_target = 0;
                distance_control_active = false;
                
                // 记录停止时的角度，用于校正
                float final_yaw = MPU6500_GetYaw();
                current_yaw_target = final_yaw;  // 更新目标角度为当前角度
                
                // 重置速度环PID，但保留角度环的积分项以维持角度稳定性
                PID_Reset(&pidLeft); 
                PID_Reset(&pidRight);
                // 不完全重置角度PID，只重置误差历史
                pidYaw.last_error = 0;
                pidYaw.last_measured = final_yaw;
                // 保留积分项的一部分，避免角度漂移
                pidYaw.integral *= 0.8f;
                
                Motor_SetRotateActive(false);
                
                // 发送完成信息和角度信息
                Bluetooth_SendString("Target distance reached! Final angle: ");
                Bluetooth_SendFloat(final_yaw, 2);
                Bluetooth_SendNewLine();
            }
        }
        
        Motor_Tick(current_speed_target, current_yaw_target); 
        // 位姿累计：编码器直线位移 + IMU 航向
        float ds = 0.0f; // 本周期线位移（米）
        Encoder_GetDeltaDistance(NULL, NULL, &ds);
        if (ds != 0.0f) {
            float yaw_deg = MPU6500_GetYaw();
            float yaw_rad = yaw_deg * (3.14159265358979323846f / 180.0f);
            // 坐标系约定调整：前进为 +Y，右侧为 -X（左右符号互换）
            odom_x_m -= ds * sinf(yaw_rad);
            odom_y_m += ds * cosf(yaw_rad);
        }
        
        // 计算机器人速度（基于里程计位置变化）
        uint32_t current_time = millis();
        if (current_time - last_speed_calc_time >= TASK_MOTOR_PERIOD) {
            float dt = (current_time - last_speed_calc_time) / 1000.0f; // 转换为秒
            if (dt > 0.001f) { // 避免除零
                // 计算X、Y方向速度
                robot_speed_x = (odom_x_m - last_odom_x) / dt;
                robot_speed_y = (odom_y_m - last_odom_y) / dt;
                // 计算总速度
                robot_speed_total = sqrtf(robot_speed_x * robot_speed_x + robot_speed_y * robot_speed_y);
                
                // 更新历史数据
                last_odom_x = odom_x_m;
                last_odom_y = odom_y_m;
                last_speed_calc_time = current_time;
            }
        }
        }

    // 4) 雷达：高频读并打印
    if (runRadarTask) {
        runRadarTask = false;
        Radar_read(true);    // 打印到 USB 串口
    }
}
