/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : depth.c
  * @brief          : 深度计驱动 (Modbus RTU 主站, 非阻塞状态机)
  *
  *  请求: [addr][0x03][reg_hi][reg_lo][cnt_hi][cnt_lo][crc_lo][crc_hi]
  *  应答: [addr][0x03][byte_cnt][data...][crc_lo][crc_hi]
  *  数据按大端组合为 32 位原始压力值，再乘 DEPTH_PRESS_SCALE 得到 Pa。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "depth.h"
#include "rs485.h"
#include "protocol.h"
#include "luorov_system.h"

typedef enum { DEPTH_IDLE, DEPTH_WAIT } depth_state_t;

static depth_state_t s_state = DEPTH_IDLE;
static uint32_t     s_last_req_ms = 0;
static int32_t      s_pressure_pa = 0;
static int32_t      s_ref_pa = DEPTH_REF_PA_DEFAULT;
static int16_t      s_temp_c100 = 0;
static uint8_t      s_ok = 0;

void depth_init(void)
{
    s_state = DEPTH_IDLE;
    s_last_req_ms = 0;
    s_pressure_pa = 0;
    s_ref_pa = DEPTH_REF_PA_DEFAULT;
    s_temp_c100 = 0;
    s_ok = 0;
}

/* 发送一次读保持寄存器请求 */
static void depth_send_request(void)
{
    uint8_t f[8];
    uint16_t crc;

    f[0] = DEPTH_SLAVE_ADDR;
    f[1] = DEPTH_FUNC_READ;
    f[2] = (uint8_t)((DEPTH_REG_START >> 8) & 0xFF);
    f[3] = (uint8_t)(DEPTH_REG_START & 0xFF);
    f[4] = (uint8_t)((DEPTH_REG_COUNT >> 8) & 0xFF);
    f[5] = (uint8_t)(DEPTH_REG_COUNT & 0xFF);
    crc = crc16_modbus(f, 6);
    f[6] = (uint8_t)(crc & 0xFF);
    f[7] = (uint8_t)((crc >> 8) & 0xFF);

    rs485_depth_flush();
    rs485_depth_send(f, 8);
}

/* 解析应答：成功返回 1 */
static uint8_t depth_parse_response(void)
{
    /* 期望长度 = 从机1 + 功能1 + 字节数1 + 数据(2*count) + CRC2 */
    uint16_t expect = (uint16_t)(5 + 2 * DEPTH_REG_COUNT);
    uint8_t  buf[16];
    uint16_t i, crc;
    int32_t  raw = 0;

    if (rs485_depth_available() < expect) return 0;

    for (i = 0; i < expect; i++) {
        buf[i] = rs485_depth_read_byte();
    }

    /* 校验 CRC (覆盖前 expect-2 字节) */
    crc = (uint16_t)(buf[expect - 1] << 8) | buf[expect - 2];
    if (crc16_modbus(buf, (uint16_t)(expect - 2)) != crc) return 0;

    /* 校验从机地址、功能码与字节数 */
    if (buf[0] != DEPTH_SLAVE_ADDR) return 0;
    if (buf[1] != DEPTH_FUNC_READ) return 0;
    if (buf[2] != (uint8_t)(2 * DEPTH_REG_COUNT)) return 0;

    /* 组合数据 (大端) */
    if (DEPTH_REG_COUNT == 2) {
        raw = (int32_t)(((uint32_t)buf[3] << 24) | ((uint32_t)buf[4] << 16) |
                        ((uint32_t)buf[5] << 8)  | ((uint32_t)buf[6]));
    } else {
        raw = (int32_t)(((uint16_t)buf[3] << 8) | buf[4]);
    }

    s_pressure_pa = (int32_t)((float)raw * DEPTH_PRESS_SCALE);
    s_ok = 1;
    return 1;
}

void depth_poll(void)
{
    uint32_t now = millis();

    switch (s_state) {
    case DEPTH_IDLE:
        if ((now - s_last_req_ms) >= DEPTH_POLL_MS) {
            depth_send_request();
            s_last_req_ms = now;
            s_state = DEPTH_WAIT;
        }
        break;

    case DEPTH_WAIT:
        if (depth_parse_response()) {
            s_state = DEPTH_IDLE;
        } else if ((now - s_last_req_ms) >= DEPTH_RESP_TIMEOUT_MS) {
            rs485_depth_flush();
            s_ok = 0;
            s_state = DEPTH_IDLE;
        }
        break;
    }
}

void depth_zero(void)
{
    if (s_ok) {
        s_ref_pa = s_pressure_pa;
    }
}

int32_t depth_get_pressure_pa(void) { return s_pressure_pa; }

int32_t depth_get_depth_mm(void)
{
    int32_t diff = s_pressure_pa - s_ref_pa;
    /* depth_mm = diff(Pa) / (1000 kg/m^3 * 9.81 m/s^2) * 1000 mm/m
     *         = diff * 1000 / 9810 */
    return (int32_t)((diff * 1000) / 9810);
}

int16_t depth_get_temp_c100(void) { return s_temp_c100; }
uint8_t depth_ok(void)            { return s_ok; }
