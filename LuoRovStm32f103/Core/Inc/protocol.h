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
#define PROTO_MAX_PAYLOAD   96

/* ---------------------------- 上位机 -> MCU 命令 ---------------------------- */
#define CMD_PING            0x01   /* 请求遥测，MCU 回 TELE_STATUS */
#define CMD_SET_MOTOR       0x02   /* 调试：单电机直通转速 [idx u8][speed i16] */
#define CMD_SET_ALL         0x03   /* 调试：设置全部 8 路 [8 x i16] */
#define CMD_STOP            0x04   /* 全停并失锁 */
#define CMD_ARM             0x05   /* 上锁/解锁 [arm u8] */
#define CMD_ZERO_DEPTH      0x06   /* 深度置零 */
#define CMD_SET_CONTROL     0x07   /* 遥控指令 [mode u8][flags u8][vx,vy,vz,yaw,roll_cdeg,pitch_cdeg 6xi16] */
#define CMD_SET_TARGETS     0x08   /* 设定目标值 [depth_mm i32][yaw_cdeg i16][hover_trim i16] */
#define CMD_SET_PID         0x09   /* 设置单路 PID [loop_id u8][kp,ki,kd,max_i,max_out 5xf32] */
#define CMD_GET_PARAMS      0x0A   /* 读取参数包 [part u8] (0=PID0..3, 1=PID4..6+Mixer) */
#define CMD_SET_MIXER       0x0B   /* 设置推进器混控分配 [mixer_cfg_t 22B] */
#define CMD_FLASH_OP        0x0C   /* 系统操作 [op u8]: 1=存Flash, 2=恢复默认, 3=IMU水平与零偏校准 */

/* ---------------------------- MCU -> 上位机 ---------------------------- */
#define TELE_STATUS         0x80   /* 遥测状态 (68 字节) */
#define TELE_ACK            0x81   /* 命令应答 [echo_type u8][result u8] */
#define TELE_PARAMS         0x82   /* 参数回传包 [part u8][data...] */

/* ---------------------------- 遥测结构 (68 字节, packed) ---------------------------- */
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
    /* --- 扩展闭环控制遥测 (16 字节) --- */
    uint8_t  ctrl_mode;        /* 当前控制模式 (0..4, 0xFF=直通调试) */
    uint8_t  sys_flags;        /* bit0=陀螺已校准, bit1=Flash参数有效 */
    int16_t  target_yaw_cdeg;  /* 目标航向 0.01° */
    int32_t  target_depth_mm;  /* 目标深度 mm */
    int16_t  pid_depth_out;    /* 深度环输出 (-1000..+1000) */
    int16_t  pid_roll_out;     /* 横滚环输出 (-1000..+1000) */
    int16_t  pid_pitch_out;    /* 俯仰环输出 (-1000..+1000) */
    int16_t  pid_yaw_out;      /* 偏航环输出 (-1000..+1000) */
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

/* 通用组帧函数，返回完整帧长度 */
uint16_t proto_build_frame(uint8_t *buf, uint8_t type, const uint8_t *payload, uint8_t payload_len);

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
