/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : control.h
  * @brief          : 多模式运动控制 + 串级 PID + FinsROV 八推进器推力分配 + Flash 存储
  *
  *  对标交大 IWIN-FINS FinsROV 推进器几何与控制架构：
  *   - 外圈 4 台 ±45° X 型矢量水平推进器: [左前 H_FL, 左后 H_RL, 右前 H_FR, 右后 H_RR]
  *   - 内圈 4 台 两侧对称垂直推进器:     [左前 V_FL, 左后 V_RL, 右前 V_FR, 右后 V_RR]
  *   - 支持 5 种遥控模式 + 单电机直通调试模式
  *   - 支持在线 PID / 混控调参并持久化保存至 STM32 末页 Flash
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __CONTROL_H
#define __CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "luorov_config.h"

/* ---------------------------- 运动控制模式 ---------------------------- */
typedef enum {
    CTRL_MODE_MANUAL       = 0,  /* 纯手动开环遥控 (Vx, Vy, Vz, Yaw 直通混控) */
    CTRL_MODE_STABILIZE    = 1,  /* 姿态自稳遥控   (Roll/Pitch 双环串级自稳 + 手动平移升降转向) */
    CTRL_MODE_DEPTH_HOLD   = 2,  /* 定深自稳遥控   (Depth 闭环 + Roll/Pitch 自稳，对标 FinsROV 悬浮模式) */
    CTRL_MODE_HEADING_HOLD = 3,  /* 定航自稳遥控   (Roll/Pitch/Yaw 三轴双环串级自稳) */
    CTRL_MODE_FULL_HOVER   = 4,  /* 全闭环悬停遥控 (Depth + Roll/Pitch/Yaw 四自由度全闭环) */
    CTRL_MODE_COUNT        = 5
} ctrl_mode_t;

#define CTRL_MODE_DIRECT_DEBUG  0xFFU  /* 单电机滑块直通测试态 */

/* ---------------------------- 7 路可调 PID 环序号 ---------------------------- */
typedef enum {
    PID_DEPTH      = 0,  /* 深度位置环:     err(cm)  -> 推力(-1000..+1000) */
    PID_ROLL_ANG   = 1,  /* 横滚外环(角度): err(deg) -> 目标角速度(dps)    */
    PID_ROLL_RATE  = 2,  /* 横滚内环(速率): err(dps) -> 推力(-1000..+1000) */
    PID_PITCH_ANG  = 3,  /* 俯仰外环(角度): err(deg) -> 目标角速度(dps)    */
    PID_PITCH_RATE = 4,  /* 俯仰内环(速率): err(dps) -> 推力(-1000..+1000) */
    PID_YAW_ANG    = 5,  /* 偏航外环(角度): err(deg) -> 目标角速度(dps)    */
    PID_YAW_RATE   = 6,  /* 偏航内环(速率): err(dps) -> 推力(-1000..+1000) */
    PID_LOOP_COUNT = 7
} pid_id_t;

/* 单个 PID 参数结构 (20 字节, IEEE754 小端 float) */
typedef struct __attribute__((packed)) {
    float kp;
    float ki;
    float kd;
    float max_i;      /* 积分限幅 */
    float max_out;    /* 总输出限幅 */
} pid_param_t;

/* 推进器混控与正反桨参数 (22 字节, packed)
 * 逻辑槽位顺序 (与 FinsROV 一致):
 *   map[0..3] = 外圈水平 [左前 H_FL, 左后 H_RL, 右前 H_FR, 右后 H_RR] -> 物理电调 0..7
 *   map[4..7] = 内圈垂直 [左前 V_FL, 左后 V_RL, 右前 V_FR, 右后 V_RR] -> 物理电调 0..7
 *   dir[0..7] = 物理电调 0..7 的正反桨符号 (+1 或 -1)
 */
typedef struct __attribute__((packed)) {
    uint8_t  map[8];             /* 逻辑位置 -> 物理通道 0..7 */
    int8_t   dir[8];             /* 物理通道 0..7 方向 (+1 / -1) */
    uint16_t deadzone_comp;      /* 死区补偿 (0..200, 对应 -1000..+1000 标度) */
    int16_t  hover_trim;         /* 垂直净浮力前馈配平 (-500..+500, 正=上浮偏置) */
    uint16_t max_power_permil;   /* 全局总推力限幅 (100..1000, 1000=100%) */
} mixer_cfg_t;

/* 遥控输入量 */
typedef struct {
    int16_t vx;                  /* 前进+ / 后退-  (-1000..+1000) */
    int16_t vy;                  /* 右移+ / 左移-  (-1000..+1000) */
    int16_t vz;                  /* 上浮+ / 下潜-  (-1000..+1000) */
    int16_t yaw;                 /* 顺时针右转+ / 逆时针左转- (-1000..+1000) */
    float   target_roll_deg;     /* 横滚目标角 (默认 0°) */
    float   target_pitch_deg;    /* 俯仰目标角 (默认 0°) */
} rc_input_t;

/* 闭环状态快照 (供遥测回传与上位机示波器绘图) */
typedef struct {
    uint8_t mode_reported;       /* 0..4 或 0xFF(直通调试) */
    uint8_t flash_loaded;        /* 1=已从 Flash 加载用户参数 */
    int32_t target_depth_mm;     /* 当前目标深度 mm */
    int16_t target_yaw_cdeg;     /* 当前目标航向 0.01° */
    int16_t pid_depth_out;       /* 深度环输出 */
    int16_t pid_roll_out;        /* 横滚环总输出 */
    int16_t pid_pitch_out;       /* 俯仰环总输出 */
    int16_t pid_yaw_out;         /* 偏航环总输出 */
} ctrl_telemetry_t;

/* ---------------------------- 对外接口 ---------------------------- */
void control_init(void);
void control_reset_state(void);
void control_on_arm_changed(uint8_t armed);

/* 切换进入单电机直通调试 (CMD_SET_MOTOR / CMD_SET_ALL 调用) */
void control_enter_direct_debug(void);

/* 更新遥控指令与工作模式 (CMD_SET_CONTROL 调用) */
void control_set_rc(uint8_t mode, uint8_t flags, const rc_input_t *rc);

/* 直接设置目标深度/目标航向/浮力配平 (CMD_SET_TARGETS 调用) */
void control_set_targets(int32_t depth_mm, float yaw_deg, int16_t hover_trim);

/* 水面深度置零时同步复位目标深度 */
void control_on_depth_zeroed(void);

/* 200Hz 主控制与推力分配循环 */
void control_update(uint8_t armed);

/* 参数读写与 Flash 持久化 */
int32_t control_set_pid(uint8_t loop_id, const pid_param_t *p);
int32_t control_get_pid(uint8_t loop_id, pid_param_t *out);
void    control_set_mixer(const mixer_cfg_t *m);
void    control_get_mixer(mixer_cfg_t *out);
void    control_load_defaults(void);
int32_t control_save_to_flash(void);

/* 获取遥测快照 */
void    control_get_telemetry(ctrl_telemetry_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_H */
