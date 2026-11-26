/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : protocol.c
  * @brief          : 岸站通信协议实现 (CRC16 + 组帧 + 解析)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "protocol.h"
#include "rs485.h"

/* CRC16-Modbus */
uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001);
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* 在帧尾追加 CRC16 (小端)。head = 帧起始索引，len = 已填充字节数 */
static void frame_append_crc(uint8_t *buf, uint16_t crc_start, uint16_t crc_len)
{
    uint16_t crc = crc16_modbus(&buf[crc_start], crc_len);
    buf[crc_start + crc_len]     = (uint8_t)(crc & 0xFF);
    buf[crc_start + crc_len + 1] = (uint8_t)((crc >> 8) & 0xFF);
}

uint16_t proto_build_status(uint8_t *buf, const telemetry_t *t)
{
    const uint8_t *src = (const uint8_t *)t;
    uint16_t i;

    buf[0] = PROTO_SYNC1;
    buf[1] = PROTO_SYNC2;
    buf[2] = (uint8_t)sizeof(telemetry_t);
    buf[3] = TELE_STATUS;
    for (i = 0; i < sizeof(telemetry_t); i++) {
        buf[4 + i] = src[i];
    }
    frame_append_crc(buf, 2, (uint16_t)(2 + sizeof(telemetry_t)));
    return (uint16_t)(6 + sizeof(telemetry_t));
}

uint16_t proto_build_ack(uint8_t *buf, uint8_t echo_type, uint8_t result)
{
    buf[0] = PROTO_SYNC1;
    buf[1] = PROTO_SYNC2;
    buf[2] = 2;                 /* 载荷 2 字节 */
    buf[3] = TELE_ACK;
    buf[4] = echo_type;
    buf[5] = result;
    frame_append_crc(buf, 2, 4);
    return 8;
}

/* ---------------------------- 流式解析 ---------------------------- */

static enum { P_SYNC1, P_SYNC2, P_LEN, P_TYPE, P_PAYLOAD, P_CRC1, P_CRC2 } s_state = P_SYNC1;
static proto_frame_t s_frame;
static uint16_t s_fill = 0;
static uint16_t s_crc = 0;

int32_t proto_try_parse(proto_frame_t *f)
{
    while (rs485_shore_available()) {
        uint8_t b = rs485_shore_read_byte();

        switch (s_state) {
        case P_SYNC1:
            if (b == PROTO_SYNC1) s_state = P_SYNC2;
            break;

        case P_SYNC2:
            if (b == PROTO_SYNC2) {
                s_state = P_LEN;
            } else if (b == PROTO_SYNC1) {
                s_state = P_SYNC2;         /* 再遇到 0xAA，保持等待 0x55 */
            } else {
                s_state = P_SYNC1;
            }
            break;

        case P_LEN:
            if (b > PROTO_MAX_PAYLOAD) { s_state = P_SYNC1; break; }
            s_frame.payload_len = b;
            s_fill = 0;
            s_state = P_TYPE;
            break;

        case P_TYPE:
            s_frame.type = b;
            s_state = (s_frame.payload_len > 0) ? P_PAYLOAD : P_CRC1;
            break;

        case P_PAYLOAD:
            s_frame.payload[s_fill++] = b;
            if (s_fill >= s_frame.payload_len) s_state = P_CRC1;
            break;

        case P_CRC1:
            s_crc = b;
            s_state = P_CRC2;
            break;

        case P_CRC2:
        {
            uint16_t crc = (uint16_t)(s_crc | ((uint16_t)b << 8));
            uint8_t tmp[2 + PROTO_MAX_PAYLOAD];
            uint16_t i;

            s_state = P_SYNC1;

            /* 计算 CRC：覆盖 [长度, 类型, 载荷] */
            tmp[0] = (uint8_t)s_frame.payload_len;
            tmp[1] = s_frame.type;
            for (i = 0; i < s_frame.payload_len; i++) {
                tmp[2 + i] = s_frame.payload[i];
            }
            if (crc16_modbus(tmp, (uint16_t)(2 + s_frame.payload_len)) == crc) {
                *f = s_frame;
                return 1;
            }
            break;
        }
        }
    }
    return 0;
}

/* 复位解析器（可选，掉线重连时调用） */
void proto_reset(void)
{
    s_state = P_SYNC1;
    s_fill = 0;
}
