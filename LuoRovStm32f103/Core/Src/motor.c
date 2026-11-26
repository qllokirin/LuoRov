/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : motor.c
  * @brief          : 8 路电调 PWM 驱动 (TIM3 + TIM4, 寄存器级)
  *
  *  TIM3 通道: CH1=PA6, CH2=PA7, CH3=PB0, CH4=PB1  -> 推进器 0..3
  *  TIM4 通道: CH1=PB6, CH2=PB7, CH3=PB8, CH4=PB9  -> 推进器 4..7
  *
  *  定时器时钟 = 72MHz (APB1 分频 2, 定时器 2 倍频)。预分频 72 -> 1MHz。
  *  周期 = 20000us (50Hz)。CCR 直接等于脉冲微秒数。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "motor.h"

/* TIM3/TIM4 属于 APB1；USART2 也在此使能，避免重复处理见 rs485.c */
static void motor_gpio_init(void)
{
    /* PA6,PA7,PB0,PB1,PB6,PB7,PB8,PB9 配置为复用推挽输出 (AF push-pull, 50MHz)
     * CNF=10, MODE=11 => 4bit 字段 = 0b1011 = 0xB */
    gpio_cfg(ESC1_GPIO, ESC1_PIN, GPIO_CFG_AF_PP, GPIO_CFG_SPEED_50M);
    gpio_cfg(ESC2_GPIO, ESC2_PIN, GPIO_CFG_AF_PP, GPIO_CFG_SPEED_50M);
    gpio_cfg(ESC3_GPIO, ESC3_PIN, GPIO_CFG_AF_PP, GPIO_CFG_SPEED_50M);
    gpio_cfg(ESC4_GPIO, ESC4_PIN, GPIO_CFG_AF_PP, GPIO_CFG_SPEED_50M);
    gpio_cfg(ESC5_GPIO, ESC5_PIN, GPIO_CFG_AF_PP, GPIO_CFG_SPEED_50M);
    gpio_cfg(ESC6_GPIO, ESC6_PIN, GPIO_CFG_AF_PP, GPIO_CFG_SPEED_50M);
    gpio_cfg(ESC7_GPIO, ESC7_PIN, GPIO_CFG_AF_PP, GPIO_CFG_SPEED_50M);
    gpio_cfg(ESC8_GPIO, ESC8_PIN, GPIO_CFG_AF_PP, GPIO_CFG_SPEED_50M);
}

static void tim3_init(void)
{
    /* 使能 TIM3 时钟 (APB1ENR bit1) */
    RCC->APB1ENR |= (1UL << 1);

    /* 预分频: 72MHz / 72 = 1MHz */
    TIM3->PSC = (uint16_t)(SYS_CLK_HZ / ESC_TIMER_TICK_HZ - 1);
    /* 自动重装载: 20000 - 1 => 20ms */
    TIM3->ARR = (uint16_t)(ESC_PERIOD_US - 1);
    TIM3->CCR1 = ESC_NEUTRAL_US;
    TIM3->CCR2 = ESC_NEUTRAL_US;
    TIM3->CCR3 = ESC_NEUTRAL_US;
    TIM3->CCR4 = ESC_NEUTRAL_US;

    /* CCMR1: OC1M=110(PWM1)+OC1PE | OC2M=110+OC2PE
     *   OC1M -> bits 6:4 = 0x6 ; OC1PE -> bit3
     *   OC2M -> bits 14:12 = 0x6 ; OC2PE -> bit11 */
    TIM3->CCMR1 = (6UL << 4) | (1UL << 3) | (6UL << 12) | (1UL << 11);
    /* CCMR2: OC3/OC4 同上 */
    TIM3->CCMR2 = (6UL << 4) | (1UL << 3) | (6UL << 12) | (1UL << 11);

    /* CCER: 使能 CH1..CH4 (bit0,4,8,12)，极性默认高有效 */
    TIM3->CCER = (1UL << 0) | (1UL << 4) | (1UL << 8) | (1UL << 12);

    /* CR1: ARPE 使能自动重装载预装载 */
    TIM3->CR1 = (1UL << 7);
    /* 生成更新事件，装载 PSC/ARR */
    TIM3->EGR = 0x1UL;
    /* 启动计数器 */
    TIM3->CR1 |= 0x1UL;
}

static void tim4_init(void)
{
    RCC->APB1ENR |= (1UL << 2);   /* TIM4 */

    TIM4->PSC = (uint16_t)(SYS_CLK_HZ / ESC_TIMER_TICK_HZ - 1);
    TIM4->ARR = (uint16_t)(ESC_PERIOD_US - 1);
    TIM4->CCR1 = ESC_NEUTRAL_US;
    TIM4->CCR2 = ESC_NEUTRAL_US;
    TIM4->CCR3 = ESC_NEUTRAL_US;
    TIM4->CCR4 = ESC_NEUTRAL_US;

    TIM4->CCMR1 = (6UL << 4) | (1UL << 3) | (6UL << 12) | (1UL << 11);
    TIM4->CCMR2 = (6UL << 4) | (1UL << 3) | (6UL << 12) | (1UL << 11);
    TIM4->CCER = (1UL << 0) | (1UL << 4) | (1UL << 8) | (1UL << 12);

    TIM4->CR1 = (1UL << 7);
    TIM4->EGR = 0x1UL;
    TIM4->CR1 |= 0x1UL;
}

void motor_init(void)
{
    motor_gpio_init();
    tim3_init();
    tim4_init();
}

void motor_set_us(uint8_t ch, uint16_t us)
{
    if (us < ESC_MIN_US) us = ESC_MIN_US;
    if (us > ESC_MAX_US) us = ESC_MAX_US;

    if (ch < 4) {
        volatile uint32_t *ccr[4] = { &TIM3->CCR1, &TIM3->CCR2, &TIM3->CCR3, &TIM3->CCR4 };
        *ccr[ch] = us;
    } else if (ch < 8) {
        volatile uint32_t *ccr[4] = { &TIM4->CCR1, &TIM4->CCR2, &TIM4->CCR3, &TIM4->CCR4 };
        *ccr[ch - 4] = us;
    }
}

void motor_set_speed(uint8_t ch, int16_t speed)
{
    int32_t s = speed;
    if (s > 1000) s = 1000;
    if (s < -1000) s = -1000;

    /* 中位死区 */
    if (s > -ESC_DEADBAND_US && s < ESC_DEADBAND_US) {
        motor_set_us(ch, ESC_NEUTRAL_US);
        return;
    }

    /* 线性映射 -1000..+1000 -> 1000..2000us，中位 1500us */
    int32_t us = ESC_NEUTRAL_US + (s * (int32_t)(ESC_MAX_US - ESC_NEUTRAL_US)) / 1000;
    motor_set_us(ch, (uint16_t)us);
}

void motor_all_stop(void)
{
    uint8_t i;
    for (i = 0; i < ESC_CHANNEL_COUNT; i++) {
        motor_set_us(i, ESC_NEUTRAL_US);
    }
}

uint16_t motor_get_us(uint8_t ch)
{
    if (ch < 4) {
        const volatile uint32_t *ccr[4] = { &TIM3->CCR1, &TIM3->CCR2, &TIM3->CCR3, &TIM3->CCR4 };
        return (uint16_t)*ccr[ch];
    } else if (ch < 8) {
        const volatile uint32_t *ccr[4] = { &TIM4->CCR1, &TIM4->CCR2, &TIM4->CCR3, &TIM4->CCR4 };
        return (uint16_t)*ccr[ch - 4];
    }
    return ESC_NEUTRAL_US;
}
