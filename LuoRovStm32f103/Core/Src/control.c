/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : control.c
  * @brief          : 多模式串级 PID 控制 + FinsROV 八推进器混控分配 + Flash 参数固化
  ******************************************************************************
  */
/* USER CODE END Header */

#include "control.h"
#include "motor.h"
#include "imu.h"
#include "depth.h"
#include "protocol.h"
#include "luorov_system.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ---------------------------- PID 运行态 ---------------------------- */
typedef struct {
    pid_param_t cfg;
    float       integral;
    float       prev_meas;
    float       d_filt;
    uint8_t     has_prev;
} pid_state_t;

/* ---------------------------- Flash 存储镜像 ---------------------------- */
#define FLASH_PARAM_MAGIC   0x4C525631UL   /* "LRV1" */

typedef struct __attribute__((packed)) {
    uint32_t    magic;
    pid_param_t pids[PID_LOOP_COUNT];
    mixer_cfg_t mixer;
    uint16_t    crc16;
    uint16_t    pad;
} flash_param_blob_t;

/* ---------------------------- 模块静态状态 ---------------------------- */
static pid_state_t s_pids[PID_LOOP_COUNT];
static mixer_cfg_t s_mixer;
static rc_input_t  s_rc;

static uint8_t     s_mode = CTRL_MODE_MANUAL;
static uint8_t     s_direct_debug = 0;
static uint8_t     s_flash_loaded = 0;
static uint8_t     s_outer_div = 0;

static float       s_target_depth_mm = 0.0f;
static float       s_target_yaw_deg  = 0.0f;
static float       s_target_roll_dps  = 0.0f;
static float       s_target_pitch_dps = 0.0f;
static float       s_target_yaw_dps   = 0.0f;

static int16_t     s_out_depth = 0;
static int16_t     s_out_roll  = 0;
static int16_t     s_out_pitch = 0;
static int16_t     s_out_yaw   = 0;

static uint32_t    s_last_ctrl_ms = 0;

/* ---------------------------- 工具函数 ---------------------------- */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float wrap_deg180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static void pid_reset(pid_state_t *p)
{
    p->integral = 0.0f;
    p->prev_meas = 0.0f;
    p->d_filt = 0.0f;
    p->has_prev = 0;
}

/* 抗积分饱和 + 测量值微分先行 (避免目标值突变引起微分冲击) */
static float pid_step(pid_state_t *p, float err, float meas, float dt)
{
    float p_term, d_raw, out;

    p_term = p->cfg.kp * err;

    if (p->cfg.ki > 0.00001f && p->cfg.max_i > 0.0f) {
        p->integral += err * p->cfg.ki * dt;
        p->integral = clampf(p->integral, -p->cfg.max_i, p->cfg.max_i);
    } else {
        p->integral = 0.0f;
    }

    if (p->has_prev && dt > 0.0005f) {
        d_raw = -(meas - p->prev_meas) / dt;
        p->d_filt = 0.7f * p->d_filt + 0.3f * d_raw;
    } else {
        p->d_filt = 0.0f;
        p->has_prev = 1;
    }
    p->prev_meas = meas;

    out = p_term + p->integral + p->cfg.kd * p->d_filt;
    return clampf(out, -p->cfg.max_out, p->cfg.max_out);
}

/* ---------------------------- 默认参数 ---------------------------- */
void control_load_defaults(void)
{
    uint8_t i;

    /* 0: 深度位置环 (cm -> 推力 -1000..+1000) */
    s_pids[PID_DEPTH].cfg      = (pid_param_t){ 10.0f, 1.2f, 4.0f, 180.0f, 500.0f };
    /* 1,2: 横滚串级双环 (外环 deg->dps, 内环 dps->推力) */
    s_pids[PID_ROLL_ANG].cfg   = (pid_param_t){  3.5f, 0.15f, 0.0f,  25.0f, 120.0f };
    s_pids[PID_ROLL_RATE].cfg  = (pid_param_t){  2.6f, 0.0f,  0.04f, 80.0f, 380.0f };
    /* 3,4: 俯仰串级双环 (外环 deg->dps, 内环 dps->推力) */
    s_pids[PID_PITCH_ANG].cfg  = (pid_param_t){  3.5f, 0.15f, 0.0f,  25.0f, 120.0f };
    s_pids[PID_PITCH_RATE].cfg = (pid_param_t){  2.6f, 0.0f,  0.04f, 80.0f, 380.0f };
    /* 5,6: 偏航串级双环 (外环 deg->dps, 内环 dps->推力) */
    s_pids[PID_YAW_ANG].cfg    = (pid_param_t){  2.5f, 0.05f, 0.0f,  20.0f,  90.0f };
    s_pids[PID_YAW_RATE].cfg   = (pid_param_t){  3.0f, 0.0f,  0.05f, 80.0f, 380.0f };

    for (i = 0; i < PID_LOOP_COUNT; i++) {
        pid_reset(&s_pids[i]);
    }

    /* 默认通道映射与接线文档一致:
     *   外圈水平 [H_FL, H_RL, H_FR, H_RR] -> M1(0), M3(2), M2(1), M4(3)
     *   内圈垂直 [V_FL, V_RL, V_FR, V_RR] -> M5(4), M7(6), M6(5), M8(7)
     * (可在上位机一键切换为 FinsROV V33 原版映射 Out:{0,3,7,4} / In:{1,2,6,5})
     */
    s_mixer.map[0] = 0;  /* H_FL -> M1 */
    s_mixer.map[1] = 2;  /* H_RL -> M3 */
    s_mixer.map[2] = 1;  /* H_FR -> M2 */
    s_mixer.map[3] = 3;  /* H_RR -> M4 */
    s_mixer.map[4] = 4;  /* V_FL -> M5 */
    s_mixer.map[5] = 6;  /* V_RL -> M7 */
    s_mixer.map[6] = 5;  /* V_FR -> M6 */
    s_mixer.map[7] = 7;  /* V_RR -> M8 */

    for (i = 0; i < 8; i++) {
        s_mixer.dir[i] = 1;
    }
    s_mixer.deadzone_comp    = 40;     /* 对应 20us 死区跨越补偿 */
    s_mixer.hover_trim       = 0;      /* 净浮力前馈偏置 */
    s_mixer.max_power_permil = 1000;   /* 100% 功率上限 */
}

/* ---------------------------- STM32 Flash 读写 ---------------------------- */
#define FLASH_KEY1_VAL   0x45670123UL
#define FLASH_KEY2_VAL   0xCDEF89ABUL

static int32_t flash_load_params(void)
{
    const flash_param_blob_t *blob = (const flash_param_blob_t *)FLASH_PARAM_ADDR;
    uint16_t calc_crc;
    uint8_t i;

    if (blob->magic != FLASH_PARAM_MAGIC) {
        return -1;
    }
    calc_crc = crc16_modbus((const uint8_t *)blob,
                            (uint16_t)(sizeof(flash_param_blob_t) - 4U));
    if (calc_crc != blob->crc16) {
        return -1;
    }

    for (i = 0; i < PID_LOOP_COUNT; i++) {
        s_pids[i].cfg = blob->pids[i];
        pid_reset(&s_pids[i]);
    }
    s_mixer = blob->mixer;
    for (i = 0; i < 8; i++) {
        if (s_mixer.map[i] >= 8) s_mixer.map[i] = i;
        s_mixer.dir[i] = (s_mixer.dir[i] < 0) ? -1 : 1;
    }
    if (s_mixer.max_power_permil < 100 || s_mixer.max_power_permil > 1000) {
        s_mixer.max_power_permil = 1000;
    }
    return 0;
}

int32_t control_save_to_flash(void)
{
    flash_param_blob_t blob;
    const uint16_t *src = (const uint16_t *)&blob;
    volatile uint16_t *dst = (volatile uint16_t *)FLASH_PARAM_ADDR;
    uint16_t words = (uint16_t)(sizeof(flash_param_blob_t) / 2U);
    uint16_t i;
    uint32_t timeout;

    memset(&blob, 0, sizeof(blob));
    blob.magic = FLASH_PARAM_MAGIC;
    for (i = 0; i < PID_LOOP_COUNT; i++) {
        blob.pids[i] = s_pids[i].cfg;
    }
    blob.mixer = s_mixer;
    blob.crc16 = crc16_modbus((const uint8_t *)&blob,
                              (uint16_t)(sizeof(flash_param_blob_t) - 4U));

    /* 解锁 Flash */
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1_VAL;
        FLASH->KEYR = FLASH_KEY2_VAL;
    }

    /* 等待空闲并擦除末页 (1KB) */
    timeout = 500000UL;
    while ((FLASH->SR & FLASH_SR_BSY) && --timeout) { }
    if (!timeout) return -1;

    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR  = FLASH_PARAM_ADDR;
    FLASH->CR |= FLASH_CR_STRT;
    timeout = 500000UL;
    while ((FLASH->SR & FLASH_SR_BSY) && --timeout) { }
    FLASH->CR &= ~FLASH_CR_PER;
    if (!timeout) { FLASH->CR |= FLASH_CR_LOCK; return -1; }

    /* 半字编程写入 */
    FLASH->CR |= FLASH_CR_PG;
    for (i = 0; i < words; i++) {
        dst[i] = src[i];
        timeout = 200000UL;
        while ((FLASH->SR & FLASH_SR_BSY) && --timeout) { }
        if (!timeout || dst[i] != src[i]) {
            FLASH->CR &= ~FLASH_CR_PG;
            FLASH->CR |= FLASH_CR_LOCK;
            return -1;
        }
    }
    FLASH->CR &= ~FLASH_CR_PG;
    FLASH->CR |= FLASH_CR_LOCK;

    s_flash_loaded = 1;
    return 0;
}

/* ---------------------------- 状态与目标管理 ---------------------------- */
void control_reset_state(void)
{
    uint8_t i;
    imu_data_t imu;

    for (i = 0; i < PID_LOOP_COUNT; i++) {
        pid_reset(&s_pids[i]);
    }
    s_outer_div = 0;
    s_target_roll_dps  = 0.0f;
    s_target_pitch_dps = 0.0f;
    s_target_yaw_dps   = 0.0f;
    s_out_depth = 0;
    s_out_roll  = 0;
    s_out_pitch = 0;
    s_out_yaw   = 0;

    if (depth_ok()) {
        s_target_depth_mm = (float)depth_get_depth_mm();
    }
    if (imu_ok()) {
        imu_get_latest(&imu);
        s_target_yaw_deg = imu.yaw_deg;
    }
}

void control_init(void)
{
    memset(&s_rc, 0, sizeof(s_rc));
    control_load_defaults();
    if (flash_load_params() == 0) {
        s_flash_loaded = 1;
    } else {
        s_flash_loaded = 0;
    }
    s_mode = CTRL_MODE_MANUAL;
    s_direct_debug = 0;
    s_last_ctrl_ms = millis();
    control_reset_state();
}

void control_on_arm_changed(uint8_t armed)
{
    control_reset_state();
    if (!armed) {
        memset(&s_rc, 0, sizeof(s_rc));
        motor_all_stop();
    }
}

void control_enter_direct_debug(void)
{
    s_direct_debug = 1;
}

void control_set_rc(uint8_t mode, uint8_t flags, const rc_input_t *rc)
{
    if (mode < CTRL_MODE_COUNT && mode != s_mode) {
        /* 切换模式时平滑锁存当前深度/航向并清零积分，防止冲击 */
        s_mode = mode;
        control_reset_state();
    }
    if (flags & 0x01U) {
        s_direct_debug = 0;
    }
    if (flags & 0x02U) {
        control_reset_state();
    }
    if (rc != NULL) {
        s_rc = *rc;
    }
}

void control_set_targets(int32_t depth_mm, float yaw_deg, int16_t hover_trim)
{
    s_target_depth_mm = clampf((float)depth_mm, -500.0f, 50000.0f);
    s_target_yaw_deg  = wrap_deg180(yaw_deg);
    s_mixer.hover_trim = (int16_t)clampf((float)hover_trim, -500.0f, 500.0f);
}

void control_on_depth_zeroed(void)
{
    s_target_depth_mm = 0.0f;
    pid_reset(&s_pids[PID_DEPTH]);
}

/* ---------------------------- FinsROV 推力分配与限幅 ---------------------------- */
static void desaturate4(float v[4], float limit)
{
    uint8_t i;
    float peak = 0.0f;
    for (i = 0; i < 4; i++) {
        float a = fabsf(v[i]);
        if (a > peak) peak = a;
    }
    if (peak > limit && peak > 1.0f) {
        float scale = limit / peak;
        for (i = 0; i < 4; i++) {
            v[i] *= scale;
        }
    }
}

static int16_t apply_dir_and_comp(float effort, int8_t dir, uint16_t comp, float limit)
{
    float v = effort * (float)((dir < 0) ? -1 : 1);
    if (fabsf(v) < 12.0f) {
        return 0;
    }
    if (v > 0.0f) {
        v += (float)comp;
    } else {
        v -= (float)comp;
    }
    v = clampf(v, -limit, limit);
    return (int16_t)v;
}

/* ---------------------------- 200Hz 主控制更新 ---------------------------- */
void control_update(uint8_t armed)
{
    uint32_t now = millis();
    float dt = (float)(now - s_last_ctrl_ms) * 0.001f;
    imu_data_t imu;
    uint8_t use_stab, use_depth, use_yaw;
    float vx, vy, heave, yaw_out, roll_out, pitch_out;
    float horiz[4], vert[4];
    float max_lim;
    uint8_t i;

    s_last_ctrl_ms = now;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.005f;
    }

    if (!armed) {
        s_out_depth = s_out_roll = s_out_pitch = s_out_yaw = 0;
        motor_all_stop();
        return;
    }

    /* 若当前处于上位机单电机滑块直通调试模式，不覆盖各电调输出 */
    if (s_direct_debug) {
        return;
    }

    imu_get_latest(&imu);

    use_stab  = (s_mode == CTRL_MODE_STABILIZE ||
                 s_mode == CTRL_MODE_DEPTH_HOLD ||
                 s_mode == CTRL_MODE_HEADING_HOLD ||
                 s_mode == CTRL_MODE_FULL_HOVER) && imu_ok();

    use_depth = (s_mode == CTRL_MODE_DEPTH_HOLD ||
                 s_mode == CTRL_MODE_FULL_HOVER);

    use_yaw   = (s_mode == CTRL_MODE_HEADING_HOLD ||
                 s_mode == CTRL_MODE_FULL_HOVER) && imu_ok();

    /* 外环 3 分频 (~66Hz)，与 FinsROV 双环 3:1 频率比一致 */
    s_outer_div++;
    if (s_outer_div >= 3) {
        s_outer_div = 0;
        if (use_stab) {
            float err_r = s_rc.target_roll_deg  - imu.roll_deg;
            float err_p = s_rc.target_pitch_deg - imu.pitch_deg;
            s_target_roll_dps  = pid_step(&s_pids[PID_ROLL_ANG],  err_r, imu.roll_deg,  dt * 3.0f);
            s_target_pitch_dps = pid_step(&s_pids[PID_PITCH_ANG], err_p, imu.pitch_deg, dt * 3.0f);
        }
        if (use_yaw) {
            if (abs(s_rc.yaw) <= 35) {
                float err_y = wrap_deg180(s_target_yaw_deg - imu.yaw_deg);
                s_target_yaw_dps = pid_step(&s_pids[PID_YAW_ANG], err_y, 0.0f, dt * 3.0f);
            }
        }
    }

    /* 1. 横滚 / 俯仰控制量 */
    if (use_stab) {
        roll_out  = pid_step(&s_pids[PID_ROLL_RATE],
                             s_target_roll_dps - imu.gyro_x_dps, imu.gyro_x_dps, dt);
        pitch_out = pid_step(&s_pids[PID_PITCH_RATE],
                             s_target_pitch_dps - imu.gyro_y_dps, imu.gyro_y_dps, dt);
    } else {
        /* 纯手动模式下允许通过 target_roll/pitch 做轻微开环倾角配平 */
        roll_out  = clampf(s_rc.target_roll_deg  * 8.0f, -400.0f, 400.0f);
        pitch_out = clampf(s_rc.target_pitch_deg * 8.0f, -400.0f, 400.0f);
    }

    /* 2. 升沉 / 定深控制量 (+heave = 上浮, -heave = 下潜) */
    if (use_depth) {
        /* 推升降杆时按最大 0.4m/s 连续调节目标深度 (上推减小深度, 下拉增加深度) */
        if (abs(s_rc.vz) > 40) {
            s_target_depth_mm -= ((float)s_rc.vz / 1000.0f) * 400.0f * dt;
            if (s_target_depth_mm < 0.0f) s_target_depth_mm = 0.0f;
        }
        if (depth_ok()) {
            /* err_cm > 0 表示目标更深 -> 需要下潜 (负 heave) */
            float cur_cm = (float)depth_get_depth_mm() * 0.1f;
            float tgt_cm = s_target_depth_mm * 0.1f;
            float d_out  = pid_step(&s_pids[PID_DEPTH], tgt_cm - cur_cm, cur_cm, dt);
            heave = -d_out;
        } else {
            /* 压力计离线时安全退回手动升沉，防止盲目积分飞车 */
            pid_reset(&s_pids[PID_DEPTH]);
            heave = (float)s_rc.vz;
        }
    } else {
        heave = (float)s_rc.vz;
        if (depth_ok()) {
            s_target_depth_mm = (float)depth_get_depth_mm();
        }
    }

    /* 3. 偏航控制量 (+yaw_out = 顺时针右转) */
    if (use_yaw) {
        if (abs(s_rc.yaw) > 35) {
            /* 摇杆打舵时按角速度闭环旋转，并实时跟随更新目标航向 */
            float cmd_dps = ((float)s_rc.yaw / 1000.0f) * 90.0f;
            /* 注意: Z 轴朝上时右转顺时针 gyro_z 为负，故统一以右转为正 */
            yaw_out = pid_step(&s_pids[PID_YAW_RATE],
                               cmd_dps - (-imu.gyro_z_dps), -imu.gyro_z_dps, dt);
            s_target_yaw_deg = imu.yaw_deg;
            pid_reset(&s_pids[PID_YAW_ANG]);
        } else {
            /* 注意: imu.yaw_deg 由 +gz 积分(左转为正)，故右转修正量取 -s_target_yaw_dps */
            yaw_out = pid_step(&s_pids[PID_YAW_RATE],
                               (-s_target_yaw_dps) - (-imu.gyro_z_dps), -imu.gyro_z_dps, dt);
        }
    } else {
        yaw_out = (float)s_rc.yaw;
        if (imu_ok()) {
            s_target_yaw_deg = imu.yaw_deg;
        }
    }

    vx = (float)s_rc.vx;
    vy = (float)s_rc.vy;

    s_out_depth = (int16_t)heave;
    s_out_roll  = (int16_t)roll_out;
    s_out_pitch = (int16_t)pitch_out;
    s_out_yaw   = (int16_t)yaw_out;

    max_lim = clampf((float)s_mixer.max_power_permil, 100.0f, 1000.0f);

    /* 4. 外圈 4 台水平推进器矢量分配 (FinsROV X 型布局: H_FL, H_RL, H_FR, H_RR) */
    horiz[0] =  vx + vy + yaw_out;   /* H_FL 左前 */
    horiz[1] =  vx - vy + yaw_out;   /* H_RL 左后 */
    horiz[2] =  vx - vy - yaw_out;   /* H_FR 右前 */
    horiz[3] =  vx + vy - yaw_out;   /* H_RR 右后 */
    desaturate4(horiz, max_lim);

    /* 5. 内圈 4 台垂直推进器推力分配 (FinsROV 侧置对称布局: V_FL, V_RL, V_FR, V_RR) */
    vert[0] = heave + roll_out - pitch_out + (float)s_mixer.hover_trim;  /* V_FL 左前 */
    vert[1] = heave + roll_out + pitch_out + (float)s_mixer.hover_trim;  /* V_RL 左后 */
    vert[2] = heave - roll_out - pitch_out + (float)s_mixer.hover_trim;  /* V_FR 右前 */
    vert[3] = heave - roll_out + pitch_out + (float)s_mixer.hover_trim;  /* V_RR 右后 */
    desaturate4(vert, max_lim);

    /* 6. 映射到物理通道、叠加正反桨符号与死区补偿并输出 PWM */
    for (i = 0; i < 4; i++) {
        uint8_t ch_h = s_mixer.map[i] & 0x07U;
        uint8_t ch_v = s_mixer.map[4 + i] & 0x07U;
        int16_t spd_h = apply_dir_and_comp(horiz[i], s_mixer.dir[ch_h], s_mixer.deadzone_comp, max_lim);
        int16_t spd_v = apply_dir_and_comp(vert[i],  s_mixer.dir[ch_v], s_mixer.deadzone_comp, max_lim);
        motor_set_speed(ch_h, spd_h);
        motor_set_speed(ch_v, spd_v);
    }
}

/* ---------------------------- 参数访问接口 ---------------------------- */
int32_t control_set_pid(uint8_t loop_id, const pid_param_t *p)
{
    if (loop_id >= PID_LOOP_COUNT || p == NULL) {
        return -1;
    }
    s_pids[loop_id].cfg = *p;
    if (s_pids[loop_id].cfg.max_i < 0.0f)   s_pids[loop_id].cfg.max_i = 0.0f;
    if (s_pids[loop_id].cfg.max_out < 0.0f) s_pids[loop_id].cfg.max_out = 0.0f;
    pid_reset(&s_pids[loop_id]);
    return 0;
}

int32_t control_get_pid(uint8_t loop_id, pid_param_t *out)
{
    if (loop_id >= PID_LOOP_COUNT || out == NULL) {
        return -1;
    }
    *out = s_pids[loop_id].cfg;
    return 0;
}

void control_set_mixer(const mixer_cfg_t *m)
{
    uint8_t i;
    if (m == NULL) return;
    s_mixer = *m;
    for (i = 0; i < 8; i++) {
        if (s_mixer.map[i] >= 8) s_mixer.map[i] = i;
        s_mixer.dir[i] = (s_mixer.dir[i] < 0) ? -1 : 1;
    }
    if (s_mixer.deadzone_comp > 300) s_mixer.deadzone_comp = 300;
    if (s_mixer.max_power_permil < 100)  s_mixer.max_power_permil = 100;
    if (s_mixer.max_power_permil > 1000) s_mixer.max_power_permil = 1000;
}

void control_get_mixer(mixer_cfg_t *out)
{
    if (out != NULL) {
        *out = s_mixer;
    }
}

void control_get_telemetry(ctrl_telemetry_t *out)
{
    if (out == NULL) return;
    out->mode_reported   = s_direct_debug ? CTRL_MODE_DIRECT_DEBUG : s_mode;
    out->flash_loaded    = s_flash_loaded;
    out->target_depth_mm = (int32_t)s_target_depth_mm;
    out->target_yaw_cdeg = (int16_t)(s_target_yaw_deg * 100.0f);
    out->pid_depth_out   = s_out_depth;
    out->pid_roll_out    = s_out_roll;
    out->pid_pitch_out   = s_out_pitch;
    out->pid_yaw_out     = s_out_yaw;
}
