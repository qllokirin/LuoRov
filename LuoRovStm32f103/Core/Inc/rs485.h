/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : rs485.h
  * @brief          : 两路 RS485 (USART1 岸站 / USART2 深度计) 驱动
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __RS485_H
#define __RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "luorov_config.h"

/* ---------------------------- 初始化 ---------------------------- */
void rs485_init(void);

/* ---------------------------- 发送 ---------------------------- */
void rs485_shore_send(const uint8_t *buf, uint16_t len);
void rs485_depth_send(const uint8_t *buf, uint16_t len);

/* ---------------------------- 岸站接收 ---------------------------- */
uint16_t rs485_shore_available(void);
uint8_t  rs485_shore_read_byte(void);
void     rs485_shore_flush(void);

/* ---------------------------- 深度计接收 ---------------------------- */
uint16_t rs485_depth_available(void);
uint8_t  rs485_depth_read_byte(void);
void     rs485_depth_flush(void);

/* ---------------------------- 中断服务 (由 it.c 调用) ---------------------------- */
void rs485_shore_isr(void);
void rs485_depth_isr(void);

#ifdef __cplusplus
}
#endif

#endif /* __RS485_H */
