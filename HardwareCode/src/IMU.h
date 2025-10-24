/**
 * @file IMU.h
 * @brief IMU 驱动模块的公共接口头文件
 *
 * 声明了所有外部可调用的函数，用于初始化 IMU 和获取姿态数据。
 */

#ifndef IMU_H
#define IMU_H


// --- Public API ---


// --- MPU6500 设置 ---
#define MPU_ADDR 0x68
// 寄存器地址
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define PWR_MGMT_1   0x6B
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG  0x1B

// 换算系数
#define ACCEL_SCALE_FACTOR 4096.0f // 对于 ±8g 量程
#define GYRO_SCALE_FACTOR  65.5f   // 对于 ±500 °/s 量程
#define G_VALUE            9.80665f // 重力加速度 (m/s^2)

// 使用结构体来存储一个MPU6500传感器实例的所有数据
typedef struct {
  // 存储校准偏移量 (raw)
  float accel_offset_x, accel_offset_y, accel_offset_z;
  float gyro_offset_x, gyro_offset_y, gyro_offset_z;

  // 存储最终处理过的数据
  float ax, ay, az; // 加速度 (g)
  float gx, gy, gz; // 角速度 (°/s)
} MPU6500_t;


// --- 声明与MPU6500操作相关的函数 ---
void mpu6500_begin();
void mpu6500_calibrate(MPU6500_t* mpu);
void mpu6500_read_data(MPU6500_t* mpu);
void MPU6500_Init(); // 新增的初始化函数
void MPU6500_Tick(); // 新增的循环任务函数
float MPU6500_GetYaw();

#endif // IMU_H
