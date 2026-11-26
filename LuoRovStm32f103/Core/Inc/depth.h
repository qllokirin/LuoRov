/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : depth.h
  * @brief          : 深度计驱动 (RS485 Modbus RTU 主站)
  *
  *  以主站身份定时读取压力变送器的保持寄存器(FC03)，
  *  换算绝对压力与水深。水深 = (压力 - 水面压力)/(ρg)。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __DEPTH_H
#define __DEPTH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "luorov_config.h"

/**
  * @brief 初始化深度计驱动状态。
  */
void depth_init(void);

/**
  * @brief 周期性轮询（状态机），需在主循环中反复调用。
  *        内部按 DEPTH_POLL_MS 周期发送请求并解析应答。
  */
void depth_poll(void);

/**
  * @brief 将当前压力设为水面零点。
  */
void depth_zero(void);

/**
  * @brief 当前绝对压力 (Pa)。
  */
int32_t depth_get_pressure_pa(void);

/**
  * @brief 当前水深 (mm)，未置零前按标准大气压估算。
  */
int32_t depth_get_depth_mm(void);

/**
  * @brief 温度 (0.01℃)。
  */
int16_t depth_get_temp_c100(void);

/**
  * @brief 最近一次读取是否有效。
  */
uint8_t depth_ok(void);

#ifdef __cplusplus
}
#endif

#endif /* __DEPTH_H */
