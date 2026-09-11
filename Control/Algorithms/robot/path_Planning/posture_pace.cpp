#include "posture_pace.h"
#include "math_support.h"

/*
 * 平面五连杆不能主动控制 Y；这里的 Y 固定为各腿实际安装平面，
 * 仅用于坐标完整性、调试显示和后续姿态算法。站姿 X 与电机锚点对齐。
 */
/* 站姿足端目标需要现场调参，所以放在 RAM 中，方便 Keil Debug/Watch 在线修改。 */
fp32 robot_stand_foot[ROBOT8_LEG_COUNT][3] = {
    { 233.25f, -70.0f, -180.0f },
    { 233.25f,  70.0f, -180.0f },
    {-233.25f, -70.0f, -180.0f },
    {-233.25f,  70.0f, -180.0f },
};

const fp32 robot_down_foot[ROBOT8_LEG_COUNT][3] = {
    { 233.25f, -70.0f, -130.0f },
    { 233.25f,  70.0f, -130.0f },
    {-233.25f, -70.0f, -130.0f },
    {-233.25f,  70.0f, -130.0f },
};

/* 对角小跑相位：RF+LH 同相，LF+RH 滞后半周期；相位差不随摆动相占比变化。 */
const fp32 robot_gait_phase_offset[ROBOT8_LEG_COUNT] = {
    0.0f, 0.5f, 0.5f, 0.0f,
};

fp32 xyz_Dog_control_debug[ROBOT8_LEG_COUNT][3] = {
    { 175.0f, -70.0f, -260.0f },
    { 175.0f,  70.0f, -260.0f },
    {-175.0f, -70.0f, -260.0f },
    {-175.0f,  70.0f, -260.0f },
};

/* 控制周期与姿态过渡参数：时间单位 s，角度门槛单位 rad。 */
fp32 g_control_dt_s          = 0.001f;
fp32 g_getup_duration_s      = 2.5f;
fp32 g_stand_transition_s    = 0.7f;
fp32 g_getdown_duration_s    = 1.8f;
fp32 g_pose_max_delta_rad    = 1.20f;
/* Debug 开关：1=启用起立/站姿目标角差保护；0=临时关闭，仅用于架空排查。 */
volatile uint8_t g_pose_delta_limit_enable = 0;
/* Debug 重算请求：Watch 中改为 1 后，上层会用当前姿态参数重新生成电机目标。 */
volatile uint8_t g_pose_debug_retarget_request = 0U;
/* Debug 重算序号：Watch 中每次改成不同数值都会触发一次重算，不会被程序清零。 */
volatile uint32_t g_pose_debug_retarget_seq = 0U;

/* 保守首轮步态：只有完成八电机零偏、方向和顺序标定后才能逐步增大。 */
fp32 g_gait_swing_ratio      = 0.38f;
fp32 g_gait_period_min_s     = 1.50f;
fp32 g_gait_period_max_s     = 2.20f;
fp32 g_gait_ramp_s           = 3.00f;
fp32 g_walk_step_max_mm      = 12.0f;
fp32 g_turn_step_max_mm      = 8.0f;
fp32 g_gait_height_mm        = 8.0f;
/* 调试阶段先关闭“摇杆幅度越大速度越快”：0=固定小幅动作，1=恢复幅度调速。 */
volatile uint8_t g_gait_joystick_speed_enable = 1U;
fp32 g_gait_fixed_command     = 0.15f;

/* 各模式电机位置控制增益；起立阶段从低增益开始，避免突然拉扯闭链。 */
fp32 g_getup_kp_start        = 4.0f;
fp32 g_getup_kd_start        = 0.8f;
fp32 g_stand_kp              = 18.0f;
fp32 g_stand_kd              = 1.5f;
fp32 g_walk_kp               = 18.0f;
fp32 g_walk_kd               = 1.6f;
fp32 g_down_kp               = 8.0f;
fp32 g_down_kd               = 1.2f;
fp32 g_safe_hold_kd          = 1.0f;

/* 发送侧安全上限：位置单位 rad，前馈力矩单位 N*m。 */
fp32 g_safe_pos_max          = 210.0f / 180.0f * PI;
fp32 g_safe_pos_min          = -210.0f / 180.0f * PI;
fp32 g_safe_kp_max           = 24.0f;
fp32 g_safe_kd_max           = 3.0f;
fp32 g_safe_tff_max          = 2.0f;
