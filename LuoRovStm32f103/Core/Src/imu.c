/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : imu.c
  * @brief          : MPU6050 驱动 + 陀螺零偏校准 + 200Hz 互补滤波姿态解算
  *
  *  使用 PB10(SCL)/PB11(SDA) 开漏输出 + 模块自带 4.7k 上拉。
  *  一次连续突发读取 14 字节 (加速度 6 + 温度 2 + 陀螺 6)，耗时 < 1ms，
  *  融合陀螺仪积分与加速度计重力方向，抑制水下推进器高频振动噪声。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "imu.h"
#include "luorov_system.h"
#include <math.h>
#include <string.h>

#define MPU6050_ADDR_W   (uint8_t)((IMU_I2C_ADDR << 1) | 0x00)  /* 0xD0 */
#define MPU6050_ADDR_R   (uint8_t)((IMU_I2C_ADDR << 1) | 0x01)  /* 0xD1 */

/* 寄存器 */
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75

#define RAD2DEG              57.2957795f
#define COMP_ALPHA           0.985f      /* 200Hz 下互补滤波陀螺权重 */

static uint8_t    s_imu_ok = 0;
static uint8_t    s_calibrated = 0;
static uint8_t    s_inited_angles = 0;
static uint32_t   s_last_ms = 0;

/* 零偏与安装水平偏置 */
static float      s_gx_bias = 0.0f;
static float      s_gy_bias = 0.0f;
static float      s_gz_bias = 0.0f;
static float      s_roll_zero  = 0.0f;
static float      s_pitch_zero = 0.0f;

/* 滤波状态与最新缓存 */
static float      s_roll_deg  = 0.0f;
static float      s_pitch_deg = 0.0f;
static float      s_yaw_deg   = 0.0f;
static imu_data_t s_latest;

/* --------------------------- 软件 I2C 位操作 --------------------------- */

static inline void i2c_scl_h(void) { gpio_set(IMU_SCL_GPIO, IMU_SCL_PIN); }
static inline void i2c_scl_l(void) { gpio_clr(IMU_SCL_GPIO, IMU_SCL_PIN); }
static inline void i2c_sda_h(void) { gpio_set(IMU_SDA_GPIO, IMU_SDA_PIN); }
static inline void i2c_sda_l(void) { gpio_clr(IMU_SDA_GPIO, IMU_SDA_PIN); }
static inline uint8_t i2c_sda_r(void) { return gpio_read(IMU_SDA_GPIO, IMU_SDA_PIN); }

static void i2c_delay(void)
{
    /* 72MHz 下约 1.5~2.5us，I2C 速率约 180~220kHz */
    volatile uint32_t i;
    for (i = 0; i < 10; i++) { __NOP(); }
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
    i2c_sda_h(); i2c_delay();
    i2c_scl_h(); i2c_delay();
    ack = i2c_sda_r();
    i2c_scl_l(); i2c_delay();
    return ack;
}

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
    i2c_start();
    if (i2c_write_byte(MPU6050_ADDR_R)) { i2c_stop(); return 1; }
    for (i = 0; i < n; i++) {
        dst[i] = i2c_read_byte((i == (uint8_t)(n - 1)) ? 1 : 0);
    }
    i2c_stop();
    return 0;
}

/* 一次突发读取 14 字节: Accel(6) + Temp(2) + Gyro(6) */
static int32_t mpu_read_burst(imu_raw_t *r)
{
    uint8_t buf[14];
    if (mpu_read_regs(MPU_REG_ACCEL_XOUT_H, buf, 14)) {
        return -1;
    }
    r->ax = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
    r->ay = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
    r->az = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    r->gx = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    r->gy = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    r->gz = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);
    return 0;
}

static float wrap_deg180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

/* 采样均值校准陀螺仪零偏；set_level=1 时同步记录水平零点 */
static int32_t imu_do_calibrate(uint8_t set_level)
{
    const uint16_t samples = 60;
    uint16_t i, valid = 0;
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;
    imu_raw_t r;

    for (i = 0; i < samples; i++) {
        if (mpu_read_burst(&r) == 0) {
            float gx = (float)r.gx / 131.0f;
            float gy = (float)r.gy / 131.0f;
            float gz = (float)r.gz / 131.0f;
            /* 仅在近似静止 (< 20 dps) 时纳入零偏统计 */
            if (fabsf(gx) < 20.0f && fabsf(gy) < 20.0f && fabsf(gz) < 20.0f) {
                sum_gx += gx;
                sum_gy += gy;
                sum_gz += gz;
                sum_ax += (float)r.ax / 16384.0f;
                sum_ay += (float)r.ay / 16384.0f;
                sum_az += (float)r.az / 16384.0f;
                valid++;
            }
        }
        delay_ms(2);
    }

    if (valid < 20) {
        return -1;
    }

    s_gx_bias = sum_gx / (float)valid;
    s_gy_bias = sum_gy / (float)valid;
    s_gz_bias = sum_gz / (float)valid;
    s_calibrated = 1;

    if (set_level) {
        float ax = sum_ax / (float)valid;
        float ay = sum_ay / (float)valid;
        float az = sum_az / (float)valid;
        s_roll_zero  = atan2f(ay, az) * RAD2DEG;
        s_pitch_zero = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
        s_roll_deg   = 0.0f;
        s_pitch_deg  = 0.0f;
        s_yaw_deg    = 0.0f;
        s_inited_angles = 1;
    }

    s_last_ms = millis();
    return 0;
}

/* --------------------------- 对外接口 --------------------------- */

int32_t imu_init(void)
{
    uint8_t who = 0;

    RCC->APB2ENR |= (1UL << 3);  /* 确保 GPIOB 时钟已开 */
    gpio_cfg(IMU_SCL_GPIO, IMU_SCL_PIN, GPIO_CFG_OUT_OD, GPIO_CFG_SPEED_50M);
    gpio_cfg(IMU_SDA_GPIO, IMU_SDA_PIN, GPIO_CFG_OUT_OD, GPIO_CFG_SPEED_50M);
    i2c_scl_h();
    i2c_sda_h();

    memset(&s_latest, 0, sizeof(s_latest));
    s_imu_ok = 0;
    s_calibrated = 0;
    s_inited_angles = 0;

    /* 复位并唤醒 */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x80);
    delay_ms(100);
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x01);    /* PLL with X axis gyro ref (更稳定) */
    delay_ms(20);

    /* 采样率 1kHz, DLPF=3 (44Hz 加速度 / 42Hz 陀螺，滤除电机高频谐振) */
    mpu_write_reg(MPU_REG_SMPLRT_DIV, 0x00);
    mpu_write_reg(MPU_REG_CONFIG, 0x03);
    /* 量程: 陀螺 ±500dps (65.5 LSB/dps) 或 ±250dps (131 LSB/dps)；保持 ±250dps 高精度 */
    mpu_write_reg(MPU_REG_GYRO_CONFIG, 0x00);
    mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x00);

    if (mpu_read_regs(MPU_REG_WHO_AM_I, &who, 1)) {
        return -1;
    }
    if (who != 0x68U && who != 0x70U && who != 0x71U && who != 0x98U) {
        return -1;
    }

    s_imu_ok = 1;
    /* 上电静止自动校准陀螺零偏 (不覆盖自然重力水平角) */
    (void)imu_do_calibrate(0);
    s_last_ms = millis();
    (void)imu_read(NULL);
    return 0;
}

int32_t imu_calibrate_level(void)
{
    if (!s_imu_ok) return -1;
    return imu_do_calibrate(1);
}

int32_t imu_read(imu_data_t *out)
{
    imu_raw_t r;
    float ax, ay, az, gx, gy, gz;
    float roll_acc, pitch_acc, acc_norm, dt;
    uint32_t now;

    if (mpu_read_burst(&r) != 0) {
        s_imu_ok = 0;
        return -1;
    }
    s_imu_ok = 1;

    now = millis();
    dt = (float)(now - s_last_ms) * 0.001f;
    s_last_ms = now;
    if (dt <= 0.0f || dt > 0.2f) {
        dt = 0.005f;
    }

    ax = (float)r.ax / 16384.0f;
    ay = (float)r.ay / 16384.0f;
    az = (float)r.az / 16384.0f;

    gx = ((float)r.gx / 131.0f) - s_gx_bias;
    gy = ((float)r.gy / 131.0f) - s_gy_bias;
    gz = ((float)r.gz / 131.0f) - s_gz_bias;

    roll_acc  = atan2f(ay, az) * RAD2DEG - s_roll_zero;
    pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG - s_pitch_zero;

    if (!s_inited_angles) {
        s_roll_deg  = roll_acc;
        s_pitch_deg = pitch_acc;
        s_inited_angles = 1;
    } else {
        acc_norm = sqrtf(ax * ax + ay * ay + az * az);
        if (acc_norm > 0.75f && acc_norm < 1.25f) {
            s_roll_deg  = COMP_ALPHA * (s_roll_deg  + gx * dt) + (1.0f - COMP_ALPHA) * roll_acc;
            s_pitch_deg = COMP_ALPHA * (s_pitch_deg + gy * dt) + (1.0f - COMP_ALPHA) * pitch_acc;
        } else {
            /* 剧烈加减速期间仅信任陀螺积分 */
            s_roll_deg  += gx * dt;
            s_pitch_deg += gy * dt;
        }
    }

    /* 偏航积分 (抑制 <0.12 dps 静止微漂) */
    if (fabsf(gz) > 0.12f) {
        s_yaw_deg = wrap_deg180(s_yaw_deg + gz * dt);
    }

    s_latest.acc_x_g    = ax;
    s_latest.acc_y_g    = ay;
    s_latest.acc_z_g    = az;
    s_latest.gyro_x_dps = gx;
    s_latest.gyro_y_dps = gy;
    s_latest.gyro_z_dps = gz;
    s_latest.roll_deg   = s_roll_deg;
    s_latest.pitch_deg  = s_pitch_deg;
    s_latest.yaw_deg    = s_yaw_deg;

    if (out != NULL) {
        *out = s_latest;
    }
    return 0;
}

void imu_get_latest(imu_data_t *out)
{
    if (out != NULL) {
        *out = s_latest;
    }
}

uint8_t imu_ok(void)
{
    return s_imu_ok;
}

uint8_t imu_is_calibrated(void)
{
    return s_calibrated;
}
