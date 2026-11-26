/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : LuoRov 主程序公共头文件
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx.h"
#include "luorov_config.h"
#include "luorov_gpio.h"
#include "luorov_system.h"
#include "motor.h"
#include "imu.h"
#include "depth.h"
#include "rs485.h"
#include "protocol.h"

void SystemClock_Config(void);
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
