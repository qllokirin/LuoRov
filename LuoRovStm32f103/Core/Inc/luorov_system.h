/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : luorov_system.h
  * @brief          : 毫秒计时原语 (SysTick)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __LUOROV_SYSTEM_H
#define __LUOROV_SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "luorov_config.h"

/* SysTick 中断累加的毫秒计数器 */
extern volatile uint32_t g_ms_tick;

/* 返回上电以来的毫秒数 */
uint32_t millis(void);

/* 忙等待毫秒 */
void delay_ms(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __LUOROV_SYSTEM_H */
