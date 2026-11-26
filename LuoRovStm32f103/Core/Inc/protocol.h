/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : protocol.h
  * @brief          : 岸站通信协议：帧格式、命令、遥测结构与 CRC16
  *
  *  帧格式 (小端):
  *   [0] 0xAA  [1] 0x55  [2] 长度  [3] 类型  [4..4+len-1] 载荷
  *   [4+len] CRC低  [5+len] CRC高
  *   CRC16-Modbus 覆盖 [2..4+len-1] (长度、类型、载荷)。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "luorov_config.h"

/* ---------------------------- 同步字与限制 ---------------------------- */
#define PROTO_SYNC1         0xAA
#define PROTO_SYNC2         0x55
#define PROTO_MAX_PAYLOAD   64

/* ---------------------------- 上位机 -> MCU 命令 ---------------------------- */
#define CMD_PING            0x01   /* 请求遥测，MCU 回 STATUS */
#define CMD_SET_MOTOR       0x02   /* 调试：单电机转速 [idx u8][speed i16] */
#define CMD_SET_ALL         0x03   /* 设置全部 8 路 [8 x i16] */
#define CMD_STOP            0x04   /* 全停并失锁 */
#define CMD_ARM             0x05   /* 上锁/解锁 [arm u8] */
#define CMD_ZERO_DEPTH      0x06   /* 深度置零 */

/* ---------------------------- MCU -> 上位机 ---------------------------- */
#define TELE_STATUS         0x80   /* 遥测状态 */
#define TELE_ACK            0x81   /* 命令应答 [echo_type u8][result u8] */

/* ---------------------------- 遥测结构 (52 字节) ---------------------------- */
typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;        /* 上电毫秒 */
    int32_t  depth_mm;         /* 深度 mm */
    int32_t  pressure_pa;      /* 绝对压力 Pa */
    int16_t  temp_c100;        /* 温度 0.01℃ */
    int16_t  roll_cdeg;        /* 横滚 0.01° */
    int16_t  pitch_cdeg;       /* 俯仰 0.01° */
    int16_t  yaw_cdeg;         /* 偏航 0.01° */
    int16_t  gyro_x_dps100;    /* 角速度 0.01 dps */
    int16_t  gyro_y_dps100;
    int16_t  gyro_z_dps100;
    int16_t  acc_x_mg;         /* 加速度 mg (0.001 g) */
    int16_t  acc_y_mg;
    int16_t  acc_z_mg;
    uint16_t motor_us[8];      /* 各电调当前脉冲 us */
    uint8_t  armed;            /* 是否已解锁 */
    uint8_t  imu_ok;
    uint8_t  depth_ok;
    uint8_t  leak;
} telemetry_t;

/* ---------------------------- 解析帧结构 ---------------------------- */
typedef struct {
    uint8_t  type;
    uint8_t  payload[PROTO_MAX_PAYLOAD];
    uint16_t payload_len;
} proto_frame_t;

/* CRC16-Modbus (poly 0xA001, init 0xFFFF, 反射) */
uint16_t crc16_modbus(const uint8_t *data, uint16_t len);

/* 从岸站接收环缓冲解析一帧；返回 1=得到一帧，0=暂无可完整解析 */
int32_t proto_try_parse(proto_frame_t *f);

/* 构造 STATUS 遥测帧，返回完整帧长度 */
uint16_t proto_build_status(uint8_t *buf, const telemetry_t *t);

/* 构造 ACK 帧，返回完整帧长度 */
uint16_t proto_build_ack(uint8_t *buf, uint8_t echo_type, uint8_t result);

/* 复位解析器状态机 */
void proto_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* __PROTOCOL_H */
