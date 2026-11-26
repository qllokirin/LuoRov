/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : rs485.c
  * @brief          : 两路 RS485 驱动 (USART1 岸站 / USART2 深度计, 寄存器级)
  *
  *  半双工 RS485：发送前置高 DE，发送完成(TC)后拉低 DE 恢复接收。
  *  接收走环形缓冲，由 USART1/USART2 中断写入。
  ******************************************************************************
  */
/* USER CODE END Header */

#include "rs485.h"
#include "luorov_system.h"

/* ---------------------------- 环形缓冲 ---------------------------- */

typedef struct {
    uint8_t  buf[256];
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t size;
} ring_t;

static ring_t s_shore_rx;
static ring_t s_depth_rx;

static void ring_init(ring_t *r, uint16_t size)
{
    r->head = 0;
    r->tail = 0;
    r->size = size;
}

static uint16_t ring_avail(const ring_t *r)
{
    return (uint16_t)((r->head - r->tail) % r->size);
}

static uint8_t ring_push(ring_t *r, uint8_t b)
{
    uint16_t next = (uint16_t)((r->head + 1) % r->size);
    if (next == r->tail) return 0;          /* 满 */
    r->buf[r->head] = b;
    r->head = next;
    return 1;
}

static uint8_t ring_pop(ring_t *r)
{
    uint8_t b;
    if (r->head == r->tail) return 0;       /* 空 */
    b = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1) % r->size);
    return b;
}

static void ring_flush(ring_t *r)
{
    r->head = r->tail;
}

/* ---------------------------- USART 初始化 ---------------------------- */

static void usart_init(USART_TypeDef *u, uint32_t baud)
{
    /* BRR = 外设时钟 / 波特率 (72MHz 整分频) */
    u->BRR = (uint16_t)(SYS_CLK_HZ / baud);
    /* CR1: UE | TE | RE | RXNEIE */
    u->CR1 = (1UL << 13) | (1UL << 3) | (1UL << 2) | (1UL << 5);
}

static void rs485_gpio_init(void)
{
    /* 使能 GPIOA/B/C、AFIO、USART1、USART2 时钟 */
    RCC->APB2ENR |= (1UL << 0)   /* AFIO  */
                 |  (1UL << 2)   /* GPIOA */
                 |  (1UL << 3)   /* GPIOB */
                 |  (1UL << 4)   /* GPIOC */
                 |  (1UL << 14); /* USART1 */
    RCC->APB1ENR |= (1UL << 17); /* USART2 */

    /* 岸站: PA9=TX(AF PP), PA10=RX(浮空输入), PA8=DE(推挽输出) */
    gpio_cfg(SHORE_TX_GPIO, SHORE_TX_PIN, GPIO_CFG_AF_PP, GPIO_CFG_SPEED_50M);
    gpio_cfg(SHORE_RX_GPIO, SHORE_RX_PIN, GPIO_CFG_IN_FLOAT, GPIO_CFG_MODE_IN);
    gpio_cfg(SHORE_DE_GPIO, SHORE_DE_PIN, GPIO_CFG_OUT_PP, GPIO_CFG_SPEED_50M);
    gpio_clr(SHORE_DE_GPIO, SHORE_DE_PIN);   /* 默认接收 */

    /* 深度计: PA2=TX, PA3=RX, PA1=DE */
    gpio_cfg(DEPTH_TX_GPIO, DEPTH_TX_PIN, GPIO_CFG_AF_PP, GPIO_CFG_SPEED_50M);
    gpio_cfg(DEPTH_RX_GPIO, DEPTH_RX_PIN, GPIO_CFG_IN_FLOAT, GPIO_CFG_MODE_IN);
    gpio_cfg(DEPTH_DE_GPIO, DEPTH_DE_PIN, GPIO_CFG_OUT_PP, GPIO_CFG_SPEED_50M);
    gpio_clr(DEPTH_DE_GPIO, DEPTH_DE_PIN);   /* 默认接收 */
}

void rs485_init(void)
{
    ring_init(&s_shore_rx, SHORE_RX_BUF_SIZE);
    ring_init(&s_depth_rx, DEPTH_RX_BUF_SIZE);

    rs485_gpio_init();
    usart_init(SHORE_UART, SHORE_BAUDRATE);
    usart_init(DEPTH_UART, DEPTH_BAUDRATE);

    /* 使能接收中断 */
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_SetPriority(USART2_IRQn, 2);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
}

/* ---------------------------- 发送 ---------------------------- */

static void usart_send(USART_TypeDef *u, GPIO_TypeDef *de_port, uint32_t de_pin,
                       const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    gpio_set(de_port, de_pin);                       /* 进入发送方向 */
    for (i = 0; i < len; i++) {
        while (!(u->SR & USART_SR_TXE)) { }          /* 等待发送数据寄存器空 */
        u->DR = buf[i];
    }
    while (!(u->SR & USART_SR_TC)) { }               /* 等待移位寄存器发送完 */
    gpio_clr(de_port, de_pin);                       /* 恢复接收方向 */
}

void rs485_shore_send(const uint8_t *buf, uint16_t len)
{
    usart_send(SHORE_UART, SHORE_DE_GPIO, SHORE_DE_PIN, buf, len);
}

void rs485_depth_send(const uint8_t *buf, uint16_t len)
{
    usart_send(DEPTH_UART, DEPTH_DE_GPIO, DEPTH_DE_PIN, buf, len);
}

/* ---------------------------- 岸站接收 ---------------------------- */

uint16_t rs485_shore_available(void) { return ring_avail(&s_shore_rx); }
uint8_t  rs485_shore_read_byte(void) { return ring_pop(&s_shore_rx); }
void     rs485_shore_flush(void)     { ring_flush(&s_shore_rx); }

/* ---------------------------- 深度计接收 ---------------------------- */

uint16_t rs485_depth_available(void) { return ring_avail(&s_depth_rx); }
uint8_t  rs485_depth_read_byte(void) { return ring_pop(&s_depth_rx); }
void     rs485_depth_flush(void)     { ring_flush(&s_depth_rx); }

/* ---------------------------- 中断服务 ---------------------------- */

void rs485_shore_isr(void)
{
    if (SHORE_UART->SR & USART_SR_RXNE) {
        ring_push(&s_shore_rx, (uint8_t)(SHORE_UART->DR & 0xFFU));
    }
    if (SHORE_UART->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        (void)(SHORE_UART->DR & 0xFFU);              /* 清错误标志 */
    }
}

void rs485_depth_isr(void)
{
    if (DEPTH_UART->SR & USART_SR_RXNE) {
        ring_push(&s_depth_rx, (uint8_t)(DEPTH_UART->DR & 0xFFU));
    }
    if (DEPTH_UART->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        (void)(DEPTH_UART->DR & 0xFFU);
    }
}
