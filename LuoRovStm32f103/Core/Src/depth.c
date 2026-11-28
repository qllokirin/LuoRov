/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : depth.c
  * @brief          : 深度计驱动 (BPM-TTR/BPM-TTS RS485 Modbus RTU 主站, 非阻塞状态机)
  *
  *  请求: [addr][0x03][reg_hi][reg_lo][cnt_hi][cnt_lo][crc_lo][crc_hi]
  *        默认 01 03 00 00 00 0B 04 0D (读 0x0000..0x000A 共 11 寄存器)
  *  应答: [addr][0x03][0x16][pv f32][pct f32][cur f32][temp f32][mv f32][unit u16][crc_lo][crc_hi]
  *        所有寄存器均为大端序 (Big-Endian)，主变量与温度为 IEEE-754 float32。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "depth.h"
#include "rs485.h"
#include "protocol.h"
#include "luorov_system.h"
#include <math.h>
#include <string.h>

typedef enum { DEPTH_IDLE, DEPTH_WAIT } depth_state_t;

#define DEPTH_FRAME_MAX_LEN   32U
#define DEPTH_MISS_LIMIT      3U

static depth_state_t s_state = DEPTH_IDLE;
static uint32_t      s_last_req_ms = 0;
static int32_t       s_pressure_pa = 0;
static int32_t       s_ref_pa = DEPTH_REF_PA_DEFAULT;
static int16_t       s_temp_c100 = 0;
static uint8_t       s_ok = 0;
static uint8_t       s_miss_cnt = 0;

/* 帧同步接收缓存（过滤半双工 DE 切换可能产生的前导毛刺字节） */
static uint8_t       s_rx_buf[DEPTH_FRAME_MAX_LEN];
static uint16_t      s_rx_len = 0;

static float read_be_f32(const uint8_t *p)
{
    uint32_t u = ((uint32_t)p[0] << 24)
               | ((uint32_t)p[1] << 16)
               | ((uint32_t)p[2] << 8)
               | ((uint32_t)p[3]);
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

static uint16_t read_be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/* 单位代码 -> 换算到 Pa 的系数（与 depth_reader.py UNIT_FACTORS 严格一致） */
static float unit_code_to_pa_scale(uint16_t code)
{
    switch (code) {
    case 11:  return 1.0f;          /* Pa */
    case 12:  return 1000.0f;       /* kPa (BPM-TTR 默认) */
    case 7:   return 100000.0f;     /* bar */
    case 8:   return 100.0f;        /* mbar */
    case 237: return 1000000.0f;    /* MPa */
    case 240:                       /* mH2O */
    case 45:  return 9806.65f;      /* m (水柱) */
    case 48:  return 98.0665f;      /* cm (水柱) */
    case 4:                         /* mmH2O */
    case 49:  return 9.80665f;      /* mm (水柱) */
    default:  return DEPTH_PRESS_SCALE;
    }
}

static int32_t round_to_i32(float v)
{
    return (v >= 0.0f) ? (int32_t)(v + 0.5f) : (int32_t)(v - 0.5f);
}

void depth_init(void)
{
    s_state = DEPTH_IDLE;
    s_last_req_ms = 0;
    s_pressure_pa = 0;
    s_ref_pa = DEPTH_REF_PA_DEFAULT;
    s_temp_c100 = 0;
    s_ok = 0;
    s_miss_cnt = 0;
    s_rx_len = 0;
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

    s_rx_len = 0;
    rs485_depth_flush();
    rs485_depth_send(f, 8);
    rs485_depth_flush();   /* 清除 DE 切换瞬间的残留回显/干扰字节 */
}

/* 流式同步并解析应答：成功返回 1 */
static uint8_t depth_parse_response(void)
{
    const uint16_t expect = (uint16_t)(5U + 2U * DEPTH_REG_COUNT);
    const uint8_t  expect_bytes = (uint8_t)(2U * DEPTH_REG_COUNT);

    if (expect > DEPTH_FRAME_MAX_LEN) {
        return 0;
    }

    while (rs485_depth_available() > 0U) {
        uint8_t b = rs485_depth_read_byte();

        if (s_rx_len == 0U) {
            if (b == DEPTH_SLAVE_ADDR) {
                s_rx_buf[s_rx_len++] = b;
            }
        } else if (s_rx_len == 1U) {
            if (b == DEPTH_FUNC_READ) {
                s_rx_buf[s_rx_len++] = b;
            } else if (b == DEPTH_SLAVE_ADDR) {
                s_rx_buf[0] = b;
                s_rx_len = 1U;
            } else {
                s_rx_len = 0U;
            }
        } else if (s_rx_len == 2U) {
            if (b == expect_bytes) {
                s_rx_buf[s_rx_len++] = b;
            } else if (b == DEPTH_SLAVE_ADDR) {
                s_rx_buf[0] = b;
                s_rx_len = 1U;
            } else {
                s_rx_len = 0U;
            }
        } else {
            s_rx_buf[s_rx_len++] = b;
            if (s_rx_len >= expect) {
                uint16_t crc = (uint16_t)(((uint16_t)s_rx_buf[expect - 1U] << 8)
                                          | (uint16_t)s_rx_buf[expect - 2U]);
                s_rx_len = 0U;

                if (crc16_modbus(s_rx_buf, (uint16_t)(expect - 2U)) != crc) {
                    continue;
                }

                /* 0x0000: 主变量压力 (大端 IEEE-754 float) */
                float pv = read_be_f32(&s_rx_buf[3]);
                if (isnan(pv) || isinf(pv)) {
                    continue;
                }

                /* 0x000A: 单位码 (若读满 11 寄存器则自动按单位换算为 Pa) */
                float scale_pa = DEPTH_PRESS_SCALE;
                if (DEPTH_REG_COUNT >= 11U) {
                    uint16_t unit_code = read_be_u16(&s_rx_buf[23]);
                    scale_pa = unit_code_to_pa_scale(unit_code);
                }

                s_pressure_pa = round_to_i32(pv * scale_pa);

                /* 0x0006: 温度 ℃ (大端 IEEE-754 float -> 0.01℃ int16) */
                if (DEPTH_REG_COUNT >= 8U) {
                    float temp_c = read_be_f32(&s_rx_buf[15]);
                    if (!isnan(temp_c) && !isinf(temp_c) &&
                        temp_c > -100.0f && temp_c < 200.0f) {
                        s_temp_c100 = (int16_t)round_to_i32(temp_c * 100.0f);
                    }
                }

                s_miss_cnt = 0U;
                s_ok = 1U;
                return 1U;
            }
        }
    }

    return 0U;
}

void depth_poll(void)
{
    uint32_t now = millis();

    switch (s_state) {
    case DEPTH_IDLE:
        if ((now - s_last_req_ms) >= DEPTH_POLL_MS) {
            depth_send_request();
            s_last_req_ms = millis();   /* 发送完成后开始计算应答超时 */
            s_state = DEPTH_WAIT;
        }
        break;

    case DEPTH_WAIT:
        if (depth_parse_response()) {
            s_state = DEPTH_IDLE;
        } else if ((now - s_last_req_ms) >= DEPTH_RESP_TIMEOUT_MS) {
            s_rx_len = 0U;
            rs485_depth_flush();
            if (s_miss_cnt < DEPTH_MISS_LIMIT) {
                s_miss_cnt++;
            }
            if (s_miss_cnt >= DEPTH_MISS_LIMIT) {
                s_ok = 0U;
            }
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
    if (!s_ok) {
        return 0;
    }
    /* depth_mm = (P - P0)[Pa] / (rho * g)[Pa/m] * 1000 */
    float diff_pa = (float)(s_pressure_pa - s_ref_pa);
    float pa_per_m = WATER_DENSITY_KG_M3 * GRAVITY_M_S2;
    return round_to_i32((diff_pa * 1000.0f) / pa_per_m);
}

int16_t depth_get_temp_c100(void) { return s_temp_c100; }
uint8_t depth_ok(void)            { return s_ok; }
