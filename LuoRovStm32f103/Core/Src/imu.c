/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : imu.c
  * @brief          : MPU6050 驱动 (软件 I2C 位操作)
  *
  *  使用 PB10(SCL)/PB11(SDA) 开漏输出 + 模块自带 4.7k 上拉。
  *  MPU6050 地址 0x68(AD0=0)。寄存器级 I2C 读写，避免 F1 硬件 I2C 坑。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "imu.h"
#include "luorov_system.h"
#include <math.h>

#define MPU6050_ADDR_W   (uint8_t)((IMU_I2C_ADDR << 1) | 0x00)  /* 0xD0 */
#define MPU6050_ADDR_R   (uint8_t)((IMU_I2C_ADDR << 1) | 0x01)  /* 0xD1 */

/* 寄存器 */
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_WHO_AM_I     0x75
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_GYRO_XOUT_H  0x43

static uint8_t s_imu_ok = 0;
static float   s_yaw_deg = 0.0f;
static uint32_t s_last_ms = 0;

/* --------------------------- 软件 I2C 位操作 --------------------------- */

static inline void i2c_scl_h(void) { gpio_set(IMU_SCL_GPIO, IMU_SCL_PIN); }
static inline void i2c_scl_l(void) { gpio_clr(IMU_SCL_GPIO, IMU_SCL_PIN); }
static inline void i2c_sda_h(void) { gpio_set(IMU_SDA_GPIO, IMU_SDA_PIN); }
static inline void i2c_sda_l(void) { gpio_clr(IMU_SDA_GPIO, IMU_SDA_PIN); }
static inline uint8_t i2c_sda_r(void) { return gpio_read(IMU_SDA_GPIO, IMU_SDA_PIN); }

static void i2c_delay(void)
{
    /* 72MHz 下约 2~4us 一次翻转，I2C 约 100~200kHz */
    volatile uint32_t i;
    for (i = 0; i < 12; i++) { __NOP(); }
}

static void i2c_start(void)
{
    i2c_sda_h(); i2c_scl_h(); i2c_delay();
    i2c_sda_l(); i2c_delay();
    i2c_scl_l(); i2c_delay();
}

static void i2c_stop(void)
{
    i2c_sda_l(); i2c_delay();
    i2c_scl_h(); i2c_delay();
    i2c_sda_h(); i2c_delay();
}

/* 返回从机 ACK (0=ACK, 1=NACK) */
static uint8_t i2c_write_byte(uint8_t b)
{
    uint8_t i, ack;
    for (i = 0; i < 8; i++) {
        if (b & 0x80) i2c_sda_h(); else i2c_sda_l();
        b <<= 1;
        i2c_delay();
        i2c_scl_h(); i2c_delay();
        i2c_scl_l(); i2c_delay();
    }
    /* 释放 SDA 读取 ACK */
    i2c_sda_h(); i2c_delay();
    i2c_scl_h(); i2c_delay();
    ack = i2c_sda_r();
    i2c_scl_l(); i2c_delay();
    return ack;
}

/* ack=0 表示主机回 ACK，ack=1 表示 NACK */
static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t i, b = 0;
    i2c_sda_h();
    for (i = 0; i < 8; i++) {
        b <<= 1;
        i2c_scl_h(); i2c_delay();
        if (i2c_sda_r()) b |= 1;
        i2c_scl_l(); i2c_delay();
    }
    if (ack) i2c_sda_h(); else i2c_sda_l();
    i2c_delay();
    i2c_scl_h(); i2c_delay();
    i2c_scl_l(); i2c_delay();
    i2c_sda_h();
    return b;
}

/* --------------------------- MPU6050 寄存器读写 --------------------------- */

static uint8_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    i2c_start();
    if (i2c_write_byte(MPU6050_ADDR_W)) { i2c_stop(); return 1; }
    if (i2c_write_byte(reg))            { i2c_stop(); return 1; }
    if (i2c_write_byte(val))            { i2c_stop(); return 1; }
    i2c_stop();
    return 0;
}

static uint8_t mpu_read_regs(uint8_t reg, uint8_t *dst, uint8_t n)
{
    uint8_t i;
    i2c_start();
    if (i2c_write_byte(MPU6050_ADDR_W)) { i2c_stop(); return 1; }
    if (i2c_write_byte(reg))            { i2c_stop(); return 1; }
    /* 重复起始 */
    i2c_start();
    if (i2c_write_byte(MPU6050_ADDR_R)) { i2c_stop(); return 1; }
    for (i = 0; i < n; i++) {
        dst[i] = i2c_read_byte((i == (uint8_t)(n - 1)) ? 1 : 0);
    }
    i2c_stop();
    return 0;
}

/* --------------------------- 对外接口 --------------------------- */

int32_t imu_init(void)
{
    uint8_t who = 0;

    /* 引脚: SCL/SDA 开漏输出 50MHz */
    gpio_cfg(IMU_SCL_GPIO, IMU_SCL_PIN, GPIO_CFG_OUT_OD, GPIO_CFG_SPEED_50M);
    gpio_cfg(IMU_SDA_GPIO, IMU_SDA_PIN, GPIO_CFG_OUT_OD, GPIO_CFG_SPEED_50M);
    i2c_scl_h();
    i2c_sda_h();

    /* 复位并唤醒 */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x80);   /* device reset */
    delay_ms(100);
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x00);   /* wake up, clock = internal 8MHz */
    delay_ms(20);

    /* 采样率分频与 DLPF */
    mpu_write_reg(MPU_REG_SMPLRT_DIV, 0x00);    /* 1kHz */
    mpu_write_reg(MPU_REG_CONFIG, 0x03);        /* DLPF=3: 44Hz accel / 42Hz gyro */
    /* 量程: 陀螺 ±250dps, 加速度 ±2g */
    mpu_write_reg(MPU_REG_GYRO_CONFIG, 0x00);
    mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x00);

    /* 校验 WHO_AM_I */
    if (mpu_read_regs(MPU_REG_WHO_AM_I, &who, 1)) { s_imu_ok = 0; return -1; }
    if (who == 0x68U || who == 0x70U) {
        s_imu_ok = 1;
        s_last_ms = millis();
        return 0;
    }
    s_imu_ok = 0;
    return -1;
}

static void imu_read_raw(imu_raw_t *r)
{
    uint8_t buf[6];

    if (mpu_read_regs(MPU_REG_ACCEL_XOUT_H, buf, 6)) { s_imu_ok = 0; return; }
    r->ax = (int16_t)((buf[0] << 8) | buf[1]);
    r->ay = (int16_t)((buf[2] << 8) | buf[3]);
    r->az = (int16_t)((buf[4] << 8) | buf[5]);

    if (mpu_read_regs(MPU_REG_GYRO_XOUT_H, buf, 6)) { s_imu_ok = 0; return; }
    r->gx = (int16_t)((buf[0] << 8) | buf[1]);
    r->gy = (int16_t)((buf[2] << 8) | buf[3]);
    r->gz = (int16_t)((buf[4] << 8) | buf[5]);
    s_imu_ok = 1;
}

int32_t imu_read(imu_data_t *out)
{
    imu_raw_t r;
    float ax, ay, az;
    uint32_t now, dt;

    if (!s_imu_ok) return -1;

    imu_read_raw(&r);
    if (!s_imu_ok) return -1;

    ax = (float)r.ax / 16384.0f;
    ay = (float)r.ay / 16384.0f;
    az = (float)r.az / 16384.0f;

    out->acc_x_g = ax;
    out->acc_y_g = ay;
    out->acc_z_g = az;
    out->gyro_x_dps = (float)r.gx / 131.0f;
    out->gyro_y_dps = (float)r.gy / 131.0f;
    out->gyro_z_dps = (float)r.gz / 131.0f;

    /* 横滚/俯仰由加速度计解算 */
    out->roll_deg  = atan2f(ay, az) * 57.2957795f;
    out->pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;

    /* 偏航由陀螺仪 Z 轴积分(会缓慢漂移，只作参考) */
    now = millis();
    dt  = now - s_last_ms;
    s_last_ms = now;
    s_yaw_deg += out->gyro_z_dps * ((float)dt / 1000.0f);
    out->yaw_deg = s_yaw_deg;

    return 0;
}

uint8_t imu_ok(void)
{
    return s_imu_ok;
}
