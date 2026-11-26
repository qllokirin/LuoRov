/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : imu.h
  * @brief          : MPU6050 陀螺仪/加速度计驱动 (软件 I2C)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __IMU_H
#define __IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "luorov_config.h"

typedef struct {
    int16_t ax, ay, az;   /* 原始加速度，±2g => 16384 LSB/g */
    int16_t gx, gy, gz;   /* 原始角速度，±250dps => 131 LSB/dps */
} imu_raw_t;

typedef struct {
    float acc_x_g;        /* 加速度 g */
    float acc_y_g;
    float acc_z_g;
    float gyro_x_dps;     /* 角速度 deg/s */
    float gyro_y_dps;
    float gyro_z_dps;
    float roll_deg;       /* 横滚角 deg (加速度计解算) */
    float pitch_deg;      /* 俯仰角 deg (加速度计解算) */
    float yaw_deg;        /* 偏航角 deg (陀螺仪 Z 积分, 会漂移) */
} imu_data_t;

/**
  * @brief 初始化软件 I2C 引脚、配置 MPU6050 并执行启动陀螺仪零偏校准。
  * @retval 0 成功(WHO_AM_I 校验通过)，非 0 失败
  */
int32_t imu_init(void);

/**
  * @brief 周期采样并执行互补滤波姿态解算 (建议 200Hz 调用)。
  * @param out 输出结构体 (可传 NULL，仅更新内部缓存)
  * @retval 0 成功，非 0 读取失败
  */
int32_t imu_read(imu_data_t *out);

/**
  * @brief 获取最近一次解算的姿态缓存 (无 I2C 阻塞开销，供遥测与控制环读取)。
  */
void imu_get_latest(imu_data_t *out);

/**
  * @brief 重新校准陀螺仪零偏，并将当前静止姿态设为水平零点 (Roll=0, Pitch=0, Yaw=0)。
  * @retval 0 成功，非 0 失败
  */
int32_t imu_calibrate_level(void);

/**
  * @brief 是否已成功初始化并读到有效数据。
  */
uint8_t imu_ok(void);

/**
  * @brief 陀螺仪零偏是否已完成校准。
  */
uint8_t imu_is_calibrated(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H */
