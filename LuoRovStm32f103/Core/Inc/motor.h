/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor.h
  * @brief          : 8 路电调 PWM 驱动 (TIM3 + TIM4)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "luorov_config.h"

/**
  * @brief 初始化 TIM3/TIM4 为 50Hz 舵机 PWM，八路全部输出中位(停转)。
  */
void motor_init(void);

/**
  * @brief 设置单路电调脉冲宽度 (微秒)。
  * @param ch   通道 0~7
  * @param us   脉冲宽度，限制在 ESC_MIN_US..ESC_MAX_US
  */
void motor_set_us(uint8_t ch, uint16_t us);

/**
  * @brief 按转速设置单路推进器。
  * @param ch    通道 0~7
  * @param speed -1000..+1000 (0=停转, 负=反向, 正=正向)
  */
void motor_set_speed(uint8_t ch, int16_t speed);

/**
  * @brief 全部回中停止。
  */
void motor_all_stop(void);

/**
  * @brief 读取某路当前脉冲宽度 (微秒)。
  */
uint16_t motor_get_us(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
