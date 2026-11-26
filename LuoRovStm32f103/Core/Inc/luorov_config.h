/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : luorov_config.h
  * @brief          : LuoRov 八推进器 ROV 引脚线序与参数集中配置
  *
  *  本文件是整机接线与关键参数的唯一入口，改线或改参数只改这里。
  *  所有引脚均基于 STM32F103C8T6 (LQFP48, 蓝派/BluePill 同款封装)。
  *
  *  说明：
  *   - 8 路电调 PWM 使用 TIM3(CH1..CH4) 与 TIM4(CH1..CH4)，50Hz 舵机信号。
  *   - 岸站通信：USART1 + RS485 收发器（半双工，DE 方向脚 PA8）。
  *   - 深度计：USART2 + RS485 收发器（Modbus RTU 主站，DE 方向脚 PA1）。
  *   - 陀螺仪：软件 I2C (MPU6050)，SCL=PB10, SDA=PB11。
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __LUOROV_CONFIG_H
#define __LUOROV_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx.h"
#include <stdint.h>
#include "luorov_gpio.h"

/* ============================ 系统时钟 ============================ */
#define SYS_CLK_HZ           72000000UL   /* 72 MHz, HSE 8MHz * PLL9 */

/* ============================ 8 路电调 PWM ============================ */
/* 电调输入为 50Hz 标准舵机 PWM：周期 20ms，中位 1500us，1000~2000us。 */
#define ESC_TIMER_TICK_HZ    1000000UL    /* 定时器 1MHz (1us 分辨率) */
#define ESC_PERIOD_US        20000UL      /* 20 ms = 50 Hz */
#define ESC_NEUTRAL_US       1500UL       /* 中位，停转 */
#define ESC_MIN_US           1000UL
#define ESC_MAX_US           2000UL
#define ESC_DEADBAND_US      30UL         /* 中位死区(双向电调低速段) */

#define ESC_CHANNEL_COUNT    8

/* TIM3: 推进器 1~4 (水平四台) */
#define ESC1_GPIO            GPIOA
#define ESC1_PIN             (1UL<<6)     /* PA6  TIM3_CH1 */
#define ESC2_GPIO            GPIOA
#define ESC2_PIN             (1UL<<7)     /* PA7  TIM3_CH2 */
#define ESC3_GPIO            GPIOB
#define ESC3_PIN             (1UL<<0)     /* PB0  TIM3_CH3 */
#define ESC4_GPIO            GPIOB
#define ESC4_PIN             (1UL<<1)     /* PB1  TIM3_CH4 */

/* TIM4: 推进器 5~8 (垂直四台) */
#define ESC5_GPIO            GPIOB
#define ESC5_PIN             (1UL<<6)     /* PB6  TIM4_CH1 */
#define ESC6_GPIO            GPIOB
#define ESC6_PIN             (1UL<<7)     /* PB7  TIM4_CH2 */
#define ESC7_GPIO            GPIOB
#define ESC7_PIN             (1UL<<8)     /* PB8  TIM4_CH3 */
#define ESC8_GPIO            GPIOB
#define ESC8_PIN             (1UL<<9)     /* PB9  TIM4_CH4 */

/* ============================ 岸站通信 (USART1) ============================ */
/* 0 = TTL 串口直连 (无需 485 收发器与 DE 方向脚); 1 = RS485 半双工 (PA8 控制 DE/RE) */
#define SHORE_USE_RS485      0
#define SHORE_UART           USART1
#define SHORE_BAUDRATE       115200UL
#define SHORE_DE_GPIO        GPIOA
#define SHORE_DE_PIN         (1UL<<8)     /* PA8  485 方向: 高=发送 (SHORE_USE_RS485=1 时使用) */
#define SHORE_TX_GPIO        GPIOA
#define SHORE_TX_PIN         (1UL<<9)     /* PA9  USART1_TX */
#define SHORE_RX_GPIO        GPIOA
#define SHORE_RX_PIN         (1UL<<10)    /* PA10 USART1_RX */
#define SHORE_RX_BUF_SIZE    256

/* ============================ 深度计 RS485 (USART2) ============================ */
#define DEPTH_UART           USART2
#define DEPTH_BAUDRATE       9600UL
#define DEPTH_DE_GPIO        GPIOA
#define DEPTH_DE_PIN         (1UL<<1)     /* PA1  485 方向: 高=发送 */
#define DEPTH_TX_GPIO        GPIOA
#define DEPTH_TX_PIN         (1UL<<2)     /* PA2  USART2_TX */
#define DEPTH_RX_GPIO        GPIOA
#define DEPTH_RX_PIN         (1UL<<3)     /* PA3  USART2_RX */
#define DEPTH_RX_BUF_SIZE    64

/* ============================ 陀螺仪 MPU6050 (软件 I2C) ============================ */
#define IMU_SCL_GPIO         GPIOB
#define IMU_SCL_PIN          (1UL<<10)    /* PB10 软件 I2C SCL */
#define IMU_SDA_GPIO         GPIOB
#define IMU_SDA_PIN          (1UL<<11)    /* PB11 软件 I2C SDA */
#define IMU_I2C_ADDR         0x68         /* MPU6050 AD0=0 */

/* ============================ 板载 LED / 预留 ============================ */
#define LED_GPIO             GPIOC
#define LED_PIN              (1UL<<13)    /* PC13 蓝派 LED(低电平点亮) */

/* ============================ 深度计 Modbus 参数 ============================ */
#define DEPTH_SLAVE_ADDR       1            /* 从机地址，按实物改 */
#define DEPTH_FUNC_READ        0x03         /* 读保持寄存器 */
#define DEPTH_REG_START        0x0000       /* 起始寄存器 */
#define DEPTH_REG_COUNT        2            /* 读 2 个寄存器 => 32 位大端 */
#define DEPTH_POLL_MS          100          /* 轮询周期 10Hz */
#define DEPTH_RESP_TIMEOUT_MS  50           /* 应答超时 */

/* 压力原始值(int32)到 Pa 的换算系数，按传感器实际量纲校准：
 *   返回 Pa       -> 1.0f
 *   返回 0.1kPa   -> 100.0f
 *   返回 kPa      -> 1000.0f
 *   返回 0.1bar   -> 10000.0f
 */
#define DEPTH_PRESS_SCALE      1.0f
#define DEPTH_REF_PA_DEFAULT   101325      /* 未置零时按标准大气压估算 */
#define WATER_DENSITY_KG_M3    1000.0f     /* 淡水 */
#define GRAVITY_M_S2           9.81f

/* ============================ 控制与安全 ============================ */
#define CTRL_LOOP_MS           5           /* 200Hz 姿态解算与控制混控周期 */
#define CMD_TIMEOUT_MS         500         /* 超时未收到有效指令 -> 停桨+失锁 */
#define LED_BLINK_MS           500
#define FLASH_PARAM_ADDR       0x0800FC00UL /* STM32F103C8 末页 1KB 存放 PID/混控参数 */

#ifdef __cplusplus
}
#endif

#endif /* __LUOROV_CONFIG_H */
