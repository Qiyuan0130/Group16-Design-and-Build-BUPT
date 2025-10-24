#include "Encoder.h"
#include <Arduino.h>
#include <math.h>

/* ========== 编码器引脚定义 =========== */
#define LEFT_ENCODER_A  PA10
#define LEFT_ENCODER_B  PB5
#define RIGHT_ENCODER_A PA8
#define RIGHT_ENCODER_B PA9

/* ========== 任务周期定义 =========== */
#define TASK_ENCODER_PERIOD 40  /* 编码器任务：40ms */

/* ========== 编码器变量 =========== */
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
volatile uint32_t encoderCounter = 0;

/* ========== 物理参数定义 =========== */
const int PULSES_PER_REVOLUTION = 390;
const float WHEEL_DIAMETER = 0.065; // 保留，不再用于里程换算
const float WHEEL_CIRCUMFERENCE = 0.215f; // 轮周长（米）：21.5cm

/* ========== 速度变量 (同时保留两种单位) =========== */
volatile float leftSpeed_pps = 0.0;
volatile float rightSpeed_pps = 0.0;
volatile float _safeLeftSpeed_pps, _safeRightSpeed_pps;
volatile float leftSpeed_mps = 0.0;
volatile float rightSpeed_mps = 0.0;
volatile float _safeLeftSpeed_mps, _safeRightSpeed_mps;

// ========== 里程累计变量（新增） ==========
static volatile long leftTotalCount = 0;
static volatile long rightTotalCount = 0;

static volatile float leftTotalMeters = 0.0f;
static volatile float rightTotalMeters = 0.0f;
static volatile float robotTotalMeters = 0.0f;

static volatile float lastDeltaLeftMeters = 0.0f;
static volatile float lastDeltaRightMeters = 0.0f;
static volatile float lastDeltaRobotMeters = 0.0f;

// --- 新增代码：为低通滤波器声明静态变量 ---
// 这些变量用于存储上一次滤波后的速度值，以实现平滑处理
static float filteredLeftSpeed_pps = 0.0f;
static float filteredRightSpeed_pps = 0.0f;

// --- 新增代码：定义滤波器系数 ---
// alpha 值决定了滤波强度。范围 (0, 1]。值越小，滤波越平滑，但响应越慢。
const float FILTER_ALPHA = 0.4f;

/* 全局编码器结构体实例 */
Encoder_TypeDef g_leftEncoder;
Encoder_TypeDef g_rightEncoder;

/* 函数前向声明 */
void leftEncoderISR(void);
void rightEncoderISR(void);

/* 4倍频状态变量 */
static volatile uint8_t old_state_left = 0;
static volatile uint8_t old_state_right = 0;

/* 编码器结构体初始化函数 */
void Encoder_Struct_Init(Encoder_TypeDef* encoder, int pinA, int pinB, volatile long* count, const char* name) {
    encoder->pinA = pinA;
    encoder->pinB = pinB;
    encoder->count = count;
    encoder->name = name;
}

/* 编码器硬件和中断初始化函数 */
void Encoder_Init(void) {
    Encoder_Struct_Init(&g_leftEncoder, LEFT_ENCODER_A, LEFT_ENCODER_B, &leftEncoderCount, "Left");
    Encoder_Struct_Init(&g_rightEncoder, RIGHT_ENCODER_A, RIGHT_ENCODER_B, &rightEncoderCount, "Right");
    
    pinMode(g_leftEncoder.pinA, INPUT_PULLUP);
    pinMode(g_leftEncoder.pinB, INPUT_PULLUP);
    pinMode(g_rightEncoder.pinA, INPUT_PULLUP);
    pinMode(g_rightEncoder.pinB, INPUT_PULLUP);

    old_state_left = (digitalRead(g_leftEncoder.pinA) << 1) | digitalRead(g_leftEncoder.pinB);
    // V-- 已将 g_rightEncoder.B 修正为 g_rightEncoder.pinB --V
    old_state_right = (digitalRead(g_rightEncoder.pinA) << 1) | digitalRead(g_rightEncoder.pinB);

    attachInterrupt(digitalPinToInterrupt(g_leftEncoder.pinA), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(g_leftEncoder.pinB), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(g_rightEncoder.pinA), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(g_rightEncoder.pinB), rightEncoderISR, CHANGE);

    leftEncoderCount = 0;
    rightEncoderCount = 0;
}

/* 4倍频中断服务函数 */
const int8_t encoder_lut[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void leftEncoderISR() {
    uint8_t new_state = (digitalRead(g_leftEncoder.pinA) << 1) | digitalRead(g_leftEncoder.pinB);
    uint8_t index = (old_state_left << 2) | new_state;
    leftEncoderCount += encoder_lut[index];
    leftTotalCount   += encoder_lut[index];
    old_state_left = new_state;
}

void rightEncoderISR() {
    uint8_t new_state = (digitalRead(g_rightEncoder.pinA) << 1) | digitalRead(g_rightEncoder.pinB);
    uint8_t index = (old_state_right << 2) | new_state;
    rightEncoderCount -= encoder_lut[index];
    rightTotalCount   -= encoder_lut[index];
    old_state_right = new_state;
}

/* ========== 编码器数据处理任务 =========== */
void Encoder_Tick(void) {
    if (++encoderCounter >= TASK_ENCODER_PERIOD) {
        noInterrupts();
        long leftCount = leftEncoderCount;
        long rightCount = rightEncoderCount;
        leftEncoderCount = 0;
        rightEncoderCount = 0;
        interrupts();

        float timeDiff = (float)encoderCounter / 1000.0f;
        if (timeDiff <= 0) timeDiff = (float)TASK_ENCODER_PERIOD / 1000.0f;

        // 1. 计算出原始的、带有噪声的速度值
        leftSpeed_pps = (float)leftCount / timeDiff;
        rightSpeed_pps = (float)rightCount / timeDiff;

        // 2. 应用低通滤波器
        // 使用 EMA 公式对原始速度进行平滑处理
        filteredLeftSpeed_pps = (FILTER_ALPHA * leftSpeed_pps) + (1.0f - FILTER_ALPHA) * filteredLeftSpeed_pps;
        filteredRightSpeed_pps = (FILTER_ALPHA * rightSpeed_pps) + (1.0f - FILTER_ALPHA) * filteredRightSpeed_pps;
        
        // 3. 使用【滤波后】的速度值来计算 m/s 单位的速度
        leftSpeed_mps = filteredLeftSpeed_pps / (4.0f * PULSES_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
        rightSpeed_mps = filteredRightSpeed_pps / (4.0f * PULSES_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;

        // 4. 将【滤波后】的速度值存入安全变量，供外部API调用
        _safeLeftSpeed_pps = filteredLeftSpeed_pps;
        _safeRightSpeed_pps = filteredRightSpeed_pps;
        _safeLeftSpeed_mps = leftSpeed_mps;
        _safeRightSpeed_mps = rightSpeed_mps;
        
        // 新增：基于本周期脉冲计算增量里程（米），并累计
        float pulsesPerRev = 4.0f * PULSES_PER_REVOLUTION;
        float deltaLeft  = (float)leftCount  / pulsesPerRev * WHEEL_CIRCUMFERENCE;
        float deltaRight = (float)rightCount / pulsesPerRev * WHEEL_CIRCUMFERENCE;
        float deltaRobot = 0.5f * (deltaLeft + deltaRight);

        lastDeltaLeftMeters  = deltaLeft;
        lastDeltaRightMeters = deltaRight;
        lastDeltaRobotMeters = deltaRobot;

        leftTotalMeters  += deltaLeft;
        rightTotalMeters += deltaRight;
        robotTotalMeters += deltaRobot;

        encoderCounter = 0;
    }
}

/* ========== 速度读取API =========== */
// 返回脉冲速度 (PPS) - 现在返回的是滤波后的值
float getLeftSpeedPPS(void) {
    return _safeLeftSpeed_pps;
}
float getRightSpeedPPS(void) {
    return _safeRightSpeed_pps;
}

// 返回物理速度 (m/s) - 现在返回的是滤波后的值
float getLeftSpeed(void) {
    return _safeLeftSpeed_mps;
}
float getRightSpeed(void) {
    return _safeRightSpeed_mps;
}

// ========== 新增：里程读取与复位 API ==========
float Encoder_GetLeftDistanceMeters(void) {
    noInterrupts();
    float v = leftTotalMeters;
    interrupts();
    return v;
}

float Encoder_GetRightDistanceMeters(void) {
    noInterrupts();
    float v = rightTotalMeters;
    interrupts();
    return v;
}

float Encoder_GetRobotDistanceMeters(void) {
    noInterrupts();
    float v = robotTotalMeters;
    interrupts();
    return v;
}

void Encoder_GetDeltaDistance(float* left_m, float* right_m, float* robot_m) {
    if (left_m || right_m || robot_m) {
        noInterrupts();
        if (left_m)  *left_m  = lastDeltaLeftMeters;
        if (right_m) *right_m = lastDeltaRightMeters;
        if (robot_m) *robot_m = lastDeltaRobotMeters;
        interrupts();
    }
}

void Encoder_ResetDistance(void) {
    noInterrupts();
    leftTotalCount = 0;
    rightTotalCount = 0;
    leftTotalMeters = 0.0f;
    rightTotalMeters = 0.0f;
    robotTotalMeters = 0.0f;
    lastDeltaLeftMeters = 0.0f;
    lastDeltaRightMeters = 0.0f;
    lastDeltaRobotMeters = 0.0f;
    interrupts();
}