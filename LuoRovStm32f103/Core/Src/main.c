/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : LuoRov 八推进器 ROV 主程序 (STM32F103C8T6, 寄存器级)
  *
  *  功能：
  *   - 8 路电调 PWM (TIM3/TIM4) + FinsROV 八推进器矢量/垂直推力分配
  *   - MPU6050 200Hz 互补滤波姿态解算 + 零偏/水平在线校准
  *   - 多模式控制: 纯手动 / 姿态自稳 / 定深遥控 / 定航自稳 / 全闭环悬停
  *   - 7 路串级 PID 在线调参与 STM32 Flash 掉电保存
  *   - 深度计 (RS485 Modbus RTU 主站) & 失联停桨安全看门狗
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
static uint32_t s_last_ctrl_ms = 0;
static uint32_t s_led_toggle_ms = 0;

/* ---------------------------- 时钟: HSE 8MHz -> PLL*9 = 72MHz ---------------------------- */
void SystemClock_Config(void)
{
    RCC->CR |= (1UL << 16);                     /* HSEON */
    while (!(RCC->CR & (1UL << 17))) { }        /* 等待 HSERDY */

    FLASH->ACR = (2UL << 0) | (1UL << 4);       /* LATENCY=2, PRFTBE=1 */

    RCC->CFGR = (RCC->CFGR & ~((0xFUL << 18) | (1UL << 16)))
              | (1UL << 16)                     /* PLLSRC = HSE */
              | (7UL << 18);                    /* PLLMULL = 9 */

    RCC->CFGR = (RCC->CFGR & ~(0x7UL << 8)) | (0x4UL << 8);

    RCC->CR |= (1UL << 24);                     /* PLLON */
    while (!(RCC->CR & (1UL << 25))) { }        /* 等待 PLLRDY */

    RCC->CFGR = (RCC->CFGR & ~0x3UL) | 0x2UL;   /* SW = PLL */
    while ((RCC->CFGR & 0xCUL) != 0x8UL) { }    /* 等待 SWS = PLL */

    SystemCoreClock = SYS_CLK_HZ;
}

static void systick_init(void)
{
    SysTick->LOAD = (uint32_t)(SystemCoreClock / 1000) - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = 0x7UL;
}

static void led_init(void)
{
    RCC->APB2ENR |= (1UL << 4);                  /* GPIOC */
    gpio_cfg(LED_GPIO, LED_PIN, GPIO_CFG_OUT_PP, GPIO_CFG_SPEED_2M);
    gpio_set(LED_GPIO, LED_PIN);
}

/* ---------------------------- 小端读取辅助 ---------------------------- */
static int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int32_t read_i32_le(const uint8_t *p)
{
    return (int32_t)((uint32_t)p[0]
                   | ((uint32_t)p[1] << 8)
                   | ((uint32_t)p[2] << 16)
                   | ((uint32_t)p[3] << 24));
}

/* ---------------------------- 遥测组装与应答 ---------------------------- */
static void build_telemetry(telemetry_t *t)
{
    imu_data_t imu;
    ctrl_telemetry_t ct;
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

    if (imu_ok()) {
        imu_get_latest(&imu);
        t->imu_ok        = 1;
        t->roll_cdeg     = (int16_t)(imu.roll_deg  * 100.0f);
        t->pitch_cdeg    = (int16_t)(imu.pitch_deg * 100.0f);
        t->yaw_cdeg      = (int16_t)(imu.yaw_deg   * 100.0f);
        t->gyro_x_dps100 = (int16_t)(imu.gyro_x_dps * 100.0f);
        t->gyro_y_dps100 = (int16_t)(imu.gyro_y_dps * 100.0f);
        t->gyro_z_dps100 = (int16_t)(imu.gyro_z_dps * 100.0f);
        t->acc_x_mg      = (int16_t)(imu.acc_x_g * 1000.0f);
        t->acc_y_mg      = (int16_t)(imu.acc_y_g * 1000.0f);
        t->acc_z_mg      = (int16_t)(imu.acc_z_g * 1000.0f);
    } else {
        t->imu_ok = 0;
    }

    control_get_telemetry(&ct);
    t->ctrl_mode       = ct.mode_reported;
    t->sys_flags       = (uint8_t)((imu_is_calibrated() ? 0x01U : 0U)
                                 | (ct.flash_loaded     ? 0x02U : 0U));
    t->target_yaw_cdeg = ct.target_yaw_cdeg;
    t->target_depth_mm = ct.target_depth_mm;
    t->pid_depth_out   = ct.pid_depth_out;
    t->pid_roll_out    = ct.pid_roll_out;
    t->pid_pitch_out   = ct.pid_pitch_out;
    t->pid_yaw_out     = ct.pid_yaw_out;
}

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

static void send_params_pack(uint8_t part)
{
    uint8_t payload[88];
    uint8_t frame_buf[96];
    uint8_t pos = 0;
    uint8_t i;
    pid_param_t p;
    mixer_cfg_t m;

    payload[pos++] = part;
    if (part == 0) {
        /* 分包 0: PID 0..3 (80B) -> 共 81B */
        for (i = 0; i < 4; i++) {
            control_get_pid(i, &p);
            memcpy(&payload[pos], &p, sizeof(p));
            pos = (uint8_t)(pos + sizeof(p));
        }
    } else {
        /* 分包 1: PID 4..6 (60B) + mixer_cfg_t (22B) -> 共 83B */
        for (i = 4; i < PID_LOOP_COUNT; i++) {
            control_get_pid(i, &p);
            memcpy(&payload[pos], &p, sizeof(p));
            pos = (uint8_t)(pos + sizeof(p));
        }
        control_get_mixer(&m);
        memcpy(&payload[pos], &m, sizeof(m));
        pos = (uint8_t)(pos + sizeof(m));
    }

    rs485_shore_send(frame_buf, proto_build_frame(frame_buf, TELE_PARAMS, payload, pos));
}

/* ---------------------------- 命令分发 ---------------------------- */
static void handle_frame(const proto_frame_t *f)
{
    uint8_t i;

    switch (f->type) {
    case CMD_PING:
        send_status();
        break;

    case CMD_ARM:
        if (f->payload_len >= 1) {
            s_armed = f->payload[0] ? 1U : 0U;
            control_on_arm_changed(s_armed);
            send_ack(CMD_ARM, 0);
        } else {
            send_ack(CMD_ARM, 1);
        }
        break;

    case CMD_STOP:
        s_armed = 0;
        control_on_arm_changed(0);
        send_ack(CMD_STOP, 0);
        break;

    case CMD_SET_MOTOR:
        if (f->payload_len >= 3) {
            uint8_t idx = f->payload[0];
            int16_t spd = read_i16_le(&f->payload[1]);
            if (s_armed && idx < ESC_CHANNEL_COUNT) {
                control_enter_direct_debug();
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
                control_enter_direct_debug();
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
        control_on_depth_zeroed();
        send_ack(CMD_ZERO_DEPTH, 0);
        break;

    case CMD_SET_CONTROL:
        if (f->payload_len >= 14) {
            rc_input_t rc;
            uint8_t mode  = f->payload[0];
            uint8_t flags = f->payload[1];
            rc.vx = read_i16_le(&f->payload[2]);
            rc.vy = read_i16_le(&f->payload[4]);
            rc.vz = read_i16_le(&f->payload[6]);
            rc.yaw = read_i16_le(&f->payload[8]);
            rc.target_roll_deg  = (float)read_i16_le(&f->payload[10]) * 0.01f;
            rc.target_pitch_deg = (float)read_i16_le(&f->payload[12]) * 0.01f;
            control_set_rc(mode, flags, &rc);
            /* 高频操控帧不回 ACK 以节省串口带宽，靠 PING/STATUS 闭环同步 */
        } else {
            send_ack(CMD_SET_CONTROL, 1);
        }
        break;

    case CMD_SET_TARGETS:
        if (f->payload_len >= 8) {
            int32_t d_mm   = read_i32_le(&f->payload[0]);
            int16_t y_cdeg = read_i16_le(&f->payload[4]);
            int16_t trim   = read_i16_le(&f->payload[6]);
            control_set_targets(d_mm, (float)y_cdeg * 0.01f, trim);
            send_ack(CMD_SET_TARGETS, 0);
        } else {
            send_ack(CMD_SET_TARGETS, 1);
        }
        break;

    case CMD_SET_PID:
        if (f->payload_len >= (1 + sizeof(pid_param_t))) {
            pid_param_t p;
            uint8_t id = f->payload[0];
            memcpy(&p, &f->payload[1], sizeof(p));
            send_ack(CMD_SET_PID, (control_set_pid(id, &p) == 0) ? 0 : 1);
        } else {
            send_ack(CMD_SET_PID, 1);
        }
        break;

    case CMD_GET_PARAMS:
        if (f->payload_len >= 1) {
            send_params_pack(f->payload[0] ? 1U : 0U);
        } else {
            send_params_pack(0);
        }
        break;

    case CMD_SET_MIXER:
        if (f->payload_len >= sizeof(mixer_cfg_t)) {
            mixer_cfg_t m;
            memcpy(&m, &f->payload[0], sizeof(m));
            control_set_mixer(&m);
            send_ack(CMD_SET_MIXER, 0);
        } else {
            send_ack(CMD_SET_MIXER, 1);
        }
        break;

    case CMD_FLASH_OP:
        if (f->payload_len >= 1) {
            uint8_t op = f->payload[0];
            if (op == 1U) {
                /* 仅允许上锁安全状态下写 Flash (擦写期间中断暂停约 20ms) */
                if (!s_armed && control_save_to_flash() == 0) {
                    send_ack(CMD_FLASH_OP, 0);
                } else {
                    send_ack(CMD_FLASH_OP, 1);
                }
            } else if (op == 2U) {
                control_load_defaults();
                send_ack(CMD_FLASH_OP, 0);
            } else if (op == 3U) {
                /* IMU 静止零偏与水平校准 */
                if (!s_armed && imu_calibrate_level() == 0) {
                    control_reset_state();
                    send_ack(CMD_FLASH_OP, 0);
                } else {
                    send_ack(CMD_FLASH_OP, 1);
                }
            } else {
                send_ack(CMD_FLASH_OP, 1);
            }
        } else {
            send_ack(CMD_FLASH_OP, 1);
        }
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
        control_on_arm_changed(0);
    }
}

/* ---------------------------- 主函数 ---------------------------- */
int main(void)
{
    proto_frame_t frame;
    uint32_t now;

    SystemClock_Config();
    systick_init();
    led_init();

    motor_init();
    rs485_init();
    depth_init();
    imu_init();
    control_init();

    motor_all_stop();
    s_last_ctrl_ms = millis();

    while (1) {
        /* 1. 深度计轮询 (独立 485 总线, 10Hz 非阻塞) */
        depth_poll();

        /* 2. 岸站帧解析与命令处理 */
        while (proto_try_parse(&frame) == 1) {
            s_last_cmd_ms = millis();
            handle_frame(&frame);
        }

        /* 3. 200Hz (5ms) 姿态采样滤波与多模式闭环推力分配 */
        now = millis();
        if ((now - s_last_ctrl_ms) >= CTRL_LOOP_MS) {
            s_last_ctrl_ms = now;
            (void)imu_read(NULL);
            control_update(s_armed);
        }

        /* 4. 失联停桨看门狗 */
        watchdog_check();

        /* 5. 心跳 LED */
        if ((now - s_led_toggle_ms) >= LED_BLINK_MS) {
            s_led_toggle_ms = now;
            gpio_write(LED_GPIO, LED_PIN, gpio_read(LED_GPIO, LED_PIN) ? 0 : 1);
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}
