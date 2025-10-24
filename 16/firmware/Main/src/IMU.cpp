#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <MadgwickAHRS.h>
#include "IMU.h"

float current_yaw = 0.0f;  // 存储当前的偏航角

// --- 全局变量 ---
MPU6500_t myMPU;          // 创建一个 MPU6500 实例
Madgwick filter;          // 创建 Madgwick 滤波器实例
const float SAMPLE_RATE_HZ = 100.0f;

// --- 函数实现 ---

void mpu6500_begin() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x10); // AFS_SEL = 2 (±8g)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x08); // FS_SEL = 1 (±500°/s)
  Wire.endTransmission(true);
}

void mpu6500_calibrate(MPU6500_t* mpu) {
  const int samples = 200;
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;
  int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;

  Serial.println("开始传感器校准...请保持MPU6500水平静止！");
  delay(1000);

  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);
    ax_r = (int16_t)(Wire.read() << 8 | Wire.read());
    ay_r = (int16_t)(Wire.read() << 8 | Wire.read());
    az_r = (int16_t)(Wire.read() << 8 | Wire.read());
    Wire.read(); Wire.read(); // Skip temp
    gx_r = (int16_t)(Wire.read() << 8 | Wire.read());
    gy_r = (int16_t)(Wire.read() << 8 | Wire.read());
    gz_r = (int16_t)(Wire.read() << 8 | Wire.read());
    sum_ax += ax_r; sum_ay += ay_r; sum_az += az_r;
    sum_gx += gx_r; sum_gy += gy_r; sum_gz += gz_r;
    delay(10);
  }

  mpu->accel_offset_x = sum_ax / samples;
  mpu->accel_offset_y = sum_ay / samples;
  mpu->accel_offset_z = (sum_az / samples) - ACCEL_SCALE_FACTOR;
  mpu->gyro_offset_x = sum_gx / samples;
  mpu->gyro_offset_y = sum_gy / samples;
  mpu->gyro_offset_z = sum_gz / samples;

  Serial.println("校准完成！");
}

void mpu6500_read_data(MPU6500_t* mpu) {
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  ax_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  ay_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  az_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  Wire.read(); Wire.read(); // Skip temperature

  gx_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  gy_raw = (int16_t)(Wire.read() << 8 | Wire.read());
  gz_raw = (int16_t)(Wire.read() << 8 | Wire.read());

  mpu->ax = (ax_raw - mpu->accel_offset_x) / ACCEL_SCALE_FACTOR;
  mpu->ay = (ay_raw - mpu->accel_offset_y) / ACCEL_SCALE_FACTOR;
  mpu->az = (az_raw - mpu->accel_offset_z) / ACCEL_SCALE_FACTOR;
  mpu->gx = (gx_raw - mpu->gyro_offset_x) / GYRO_SCALE_FACTOR;
  mpu->gy = (gy_raw - mpu->gyro_offset_y) / GYRO_SCALE_FACTOR;
  mpu->gz = (gz_raw - mpu->gyro_offset_z) / GYRO_SCALE_FACTOR;
}

void MPU6500_Init(){
  Wire.begin();
  delay(100);
  mpu6500_begin();
  Serial.println("MPU6500 初始化成功！");
  mpu6500_calibrate(&myMPU);
  filter.begin(SAMPLE_RATE_HZ);
  Serial.println("Madgwick 滤波器已启动。");
}

void MPU6500_Tick(){
    mpu6500_read_data(&myMPU);
    filter.updateIMU(myMPU.gx, myMPU.gy, myMPU.gz, myMPU.ax, myMPU.ay, myMPU.az);

    float final_yaw = filter.getYaw();

    // --- 【最终坐标系修正】 ---
    // a. 施加180度的偏移量
    final_yaw -= 180.0f;

    // b. 角度环绕处理，确保范围在 [-180, 180]
    if (final_yaw < -180.0f) {
        final_yaw += 360.0f;
    } else if (final_yaw > 180.0f) {
        final_yaw -= 360.0f;
    }
    
    // c. 确保之前用于反转方向的 `final_yaw = -final_yaw;` 已经被删除或注释掉

    // 5. 将完全修正好的值存入全局变量
    noInterrupts();
    current_yaw = final_yaw;
    interrupts();
}

float MPU6500_GetYaw() {
    return current_yaw;
}