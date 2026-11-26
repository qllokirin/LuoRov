/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : luorov_gpio.h
  * @brief          : 寄存器级 GPIO 操作助手 (F1 系列)
  *
  *  F1 的 GPIO 引脚配置在 CRL(0~7)/CRH(8~15)，每脚 4 位：
  *    [3:2] CNF  [1:0] MODE
  *  输出: CNF 00=推挽 01=开漏 10=复用推挽 11=复用开漏
  *  输入: CNF 00=模拟 01=浮空 10=上/下拉(由ODR决定)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __LUOROV_GPIO_H
#define __LUOROV_GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx.h"

/* MODE 取值 */
#define GPIO_CFG_MODE_IN      0U   /* 输入 */
#define GPIO_CFG_SPEED_10M    1U   /* 输出 10MHz */
#define GPIO_CFG_SPEED_2M     2U   /* 输出 2MHz  */
#define GPIO_CFG_SPEED_50M    3U   /* 输出 50MHz */

/* 输出 CNF */
#define GPIO_CFG_OUT_PP       0U   /* 通用推挽 */
#define GPIO_CFG_OUT_OD       1U   /* 通用开漏 */
#define GPIO_CFG_AF_PP        2U   /* 复用推挽 */
#define GPIO_CFG_AF_OD        3U   /* 复用开漏 */

/* 输入 CNF */
#define GPIO_CFG_IN_ANALOG    0U
#define GPIO_CFG_IN_FLOAT     1U
#define GPIO_CFG_IN_PUPD      2U

/**
  * @brief 配置单个引脚模式。
  * @param port GPIOA/GPIOB/...
  * @param pin  位掩码，如 (1UL<<6)
  * @param cnf  CNF 取值
  * @param mode MODE 取值
  */
static inline void gpio_cfg(GPIO_TypeDef *port, uint32_t pin, uint32_t cnf, uint32_t mode)
{
    uint32_t n = 0U;
    uint32_t p = pin;
    while (!(p & 1UL)) { p >>= 1; n++; }

    volatile uint32_t *cr = (n < 8U) ? &port->CRL : &port->CRH;
    uint32_t sh = (n & 7U) * 4U;
    uint32_t val = ((cnf << 2) | mode) & 0xFU;

    *cr = (*cr & ~(0xFUL << sh)) | (val << sh);
}

static inline void gpio_set(GPIO_TypeDef *port, uint32_t pin)
{
    port->BSRR = pin;
}

static inline void gpio_clr(GPIO_TypeDef *port, uint32_t pin)
{
    port->BRR = pin;
}

static inline uint8_t gpio_read(GPIO_TypeDef *port, uint32_t pin)
{
    return (port->IDR & pin) ? 1U : 0U;
}

static inline void gpio_write(GPIO_TypeDef *port, uint32_t pin, uint8_t level)
{
    if (level) gpio_set(port, pin);
    else       gpio_clr(port, pin);
}

#ifdef __cplusplus
}
#endif

#endif /* __LUOROV_GPIO_H */
