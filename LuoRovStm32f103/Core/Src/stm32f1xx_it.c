/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   LuoRov 中断服务函数 (寄存器级)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "stm32f1xx_it.h"

/* ---------------------------- SysTick: 毫秒计时 ---------------------------- */
void SysTick_Handler(void)
{
    g_ms_tick++;
}

/* ---------------------------- USART 接收中断 ---------------------------- */
void USART1_IRQHandler(void)
{
    rs485_shore_isr();
}

void USART2_IRQHandler(void)
{
    rs485_depth_isr();
}

/* ---------------------------- 系统异常 ---------------------------- */

void NMI_Handler(void)
{
    while (1) { }
}

void HardFault_Handler(void)
{
    while (1) { }
}

void MemManage_Handler(void)
{
    while (1) { }
}

void BusFault_Handler(void)
{
    while (1) { }
}

void UsageFault_Handler(void)
{
    while (1) { }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}
