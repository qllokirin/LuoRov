/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LuoRov 八推进器 ROV 主程序 (STM32F103C8T6, 寄存器级)
  *
  *  功能：
  *   - 8 路电调 PWM (TIM3/TIM4)
  *   - MPU6050 陀螺仪 (软件 I2C)
  *   - 深度计 (RS485 Modbus RTU 主站)
  *   - 岸站通信 (RS485, 轮询-应答协议)
  *   - 失联停桨安全看门狗
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>

/* ---------------------------- 全局计时 ---------------------------- */
volatile uint32_t g_ms_tick = 0;

uint32_t millis(void) { return g_ms_tick; }

void delay_ms(uint32_t ms)
{
    uint32_t start = millis();
    while ((millis() - start) < ms) { }
}

extern uint32_t SystemCoreClock;

/* ---------------------------- 运行状态 ---------------------------- */
static uint8_t  s_armed = 0;
static uint32_t s_last_cmd_ms = 0;
static uint32_t s_led_toggle_ms = 0;

/* ---------------------------- 时钟: HSE 8MHz -> PLL*9 = 72MHz ---------------------------- */
void SystemClock_Config(void)
{
    /* 使能 HSE */
    RCC->CR |= (1UL << 16);                     /* HSEON */
    while (!(RCC->CR & (1UL << 17))) { }        /* 等待 HSERDY */

    /* Flash: 2 等待周期 + 预取 */
    FLASH->ACR = (2UL << 0) | (1UL << 4);       /* LATENCY=2, PRFTBE=1 */

    /* PLL: 时钟源 HSE, 倍频 9 */
    RCC->CFGR = (RCC->CFGR & ~((0xFUL << 18) | (1UL << 16)))
              | (1UL << 16)                     /* PLLSRC = HSE */
              | (7UL << 18);                    /* PLLMULL = 9 */

    /* APB1 = DIV2 (36MHz, 定时器 2 倍频 = 72MHz) */
    RCC->CFGR = (RCC->CFGR & ~(0x7UL << 8)) | (0x4UL << 8);

    /* 使能 PLL */
    RCC->CR |= (1UL << 24);                     /* PLLON */
    while (!(RCC->CR & (1UL << 25))) { }        /* 等待 PLLRDY */

    /* 切换系统时钟到 PLL */
    RCC->CFGR = (RCC->CFGR & ~0x3UL) | 0x2UL;   /* SW = PLL */
    while ((RCC->CFGR & 0xCUL) != 0x8UL) { }    /* 等待 SWS = PLL */

    SystemCoreClock = SYS_CLK_HZ;
}

static void systick_init(void)
{
    SysTick->LOAD = (uint32_t)(SystemCoreClock / 1000) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x7UL;                       /* CLKSOURCE | TICKINT | ENABLE */
}

static void led_init(void)
{
    RCC->APB2ENR |= (1UL << 4);                  /* GPIOC */
    gpio_cfg(LED_GPIO, LED_PIN, GPIO_CFG_OUT_PP, GPIO_CFG_SPEED_2M);
    gpio_set(LED_GPIO, LED_PIN);                 /* 低电平点亮，先熄灭 */
}

/* ---------------------------- 遥测组装 ---------------------------- */
static void build_telemetry(telemetry_t *t)
{
    imu_data_t imu;
    uint8_t i;

    memset(t, 0, sizeof(*t));

    t->uptime_ms   = millis();
    t->pressure_pa = depth_get_pressure_pa();
    t->depth_mm    = depth_get_depth_mm();
    t->temp_c100   = depth_get_temp_c100();
    t->depth_ok    = depth_ok();
    t->armed       = s_armed;
    t->leak        = 0;

    for (i = 0; i < ESC_CHANNEL_COUNT; i++) {
        t->motor_us[i] = motor_get_us(i);
    }

    if (imu_ok() && (imu_read(&imu) == 0)) {
        t->imu_ok       = 1;
        t->roll_cdeg    = (int16_t)(imu.roll_deg  * 100.0f);
        t->pitch_cdeg   = (int16_t)(imu.pitch_deg * 100.0f);
        t->yaw_cdeg     = (int16_t)(imu.yaw_deg   * 100.0f);
        t->gyro_x_dps100 = (int16_t)(imu.gyro_x_dps * 100.0f);
        t->gyro_y_dps100 = (int16_t)(imu.gyro_y_dps * 100.0f);
        t->gyro_z_dps100 = (int16_t)(imu.gyro_z_dps * 100.0f);
        t->acc_x_mg     = (int16_t)(imu.acc_x_g * 1000.0f);
        t->acc_y_mg     = (int16_t)(imu.acc_y_g * 1000.0f);
        t->acc_z_mg     = (int16_t)(imu.acc_z_g * 1000.0f);
    } else {
        t->imu_ok = 0;
    }
}

/* ---------------------------- 命令处理 ---------------------------- */
static void send_status(void)
{
    telemetry_t t;
    uint8_t buf[8 + sizeof(telemetry_t)];
    uint16_t len;

    build_telemetry(&t);
    len = proto_build_status(buf, &t);
    rs485_shore_send(buf, len);
}

static void send_ack(uint8_t type, uint8_t result)
{
    uint8_t buf[8];
    uint16_t len = proto_build_ack(buf, type, result);
    rs485_shore_send(buf, len);
}

static int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void handle_frame(const proto_frame_t *f)
{
    uint8_t i;

    switch (f->type) {
    case CMD_PING:
        send_status();
        break;

    case CMD_ARM:
        if (f->payload_len >= 1) {
            if (f->payload[0]) {
                s_armed = 1;
            } else {
                s_armed = 0;
                motor_all_stop();
            }
            send_ack(CMD_ARM, 0);
        } else {
            send_ack(CMD_ARM, 1);
        }
        break;

    case CMD_STOP:
        s_armed = 0;
        motor_all_stop();
        send_ack(CMD_STOP, 0);
        break;

    case CMD_SET_MOTOR:
        if (f->payload_len >= 3) {
            uint8_t idx = f->payload[0];
            int16_t spd = read_i16_le(&f->payload[1]);
            if (s_armed && idx < ESC_CHANNEL_COUNT) {
                motor_set_speed(idx, spd);
                send_ack(CMD_SET_MOTOR, 0);
            } else {
                send_ack(CMD_SET_MOTOR, 1);
            }
        } else {
            send_ack(CMD_SET_MOTOR, 1);
        }
        break;

    case CMD_SET_ALL:
        if (f->payload_len >= (2 * ESC_CHANNEL_COUNT)) {
            if (s_armed) {
                for (i = 0; i < ESC_CHANNEL_COUNT; i++) {
                    motor_set_speed(i, read_i16_le(&f->payload[i * 2]));
                }
                send_ack(CMD_SET_ALL, 0);
            } else {
                send_ack(CMD_SET_ALL, 1);
            }
        } else {
            send_ack(CMD_SET_ALL, 1);
        }
        break;

    case CMD_ZERO_DEPTH:
        depth_zero();
        send_ack(CMD_ZERO_DEPTH, 0);
        break;

    default:
        send_ack(f->type, 1);
        break;
    }
}

/* ---------------------------- 看门狗 ---------------------------- */
static void watchdog_check(void)
{
    if (s_armed && ((millis() - s_last_cmd_ms) >= CMD_TIMEOUT_MS)) {
        s_armed = 0;
        motor_all_stop();
    }
}

/* ---------------------------- 主函数 ---------------------------- */
int main(void)
{
    proto_frame_t frame;

    SystemClock_Config();
    systick_init();
    led_init();

    motor_init();
    rs485_init();
    depth_init();
    imu_init();

    /* 启动时确保停桨 */
    motor_all_stop();

    while (1) {
        /* 1. 深度计轮询（独立 485 总线） */
        depth_poll();

        /* 2. 岸站帧解析与命令处理 */
        while (proto_try_parse(&frame) == 1) {
            s_last_cmd_ms = millis();
            handle_frame(&frame);
        }

        /* 3. 失联停桨 */
        watchdog_check();

        /* 4. 心跳 LED */
        if ((millis() - s_led_toggle_ms) >= LED_BLINK_MS) {
            s_led_toggle_ms = millis();
            /* PC13 低电平点亮：有电且运行中则闪烁 */
            gpio_write(LED_GPIO, LED_PIN, gpio_read(LED_GPIO, LED_PIN) ? 0 : 1);
        }
    }
}

/* ---------------------------- 错误处理 ---------------------------- */
void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
