#ifndef POSTURE_PACE_H
#define POSTURE_PACE_H

#include "struct_typedef.h"

#define ROBOT8_LEG_COUNT        4U
#define ROBOT8_MOTORS_PER_LEG   2U

enum robot8_leg_index
{
    ROBOT8_RF = 0,
    ROBOT8_LF = 1,
    ROBOT8_RH = 2,
    ROBOT8_LH = 3,
};

/* 机身坐标系足端目标，单位 mm；+X 向前、+Y 向左、+Z 向上，行顺序为 RF/LF/RH/LH。 */
extern fp32 robot_stand_foot[ROBOT8_LEG_COUNT][3];
extern const fp32 robot_down_foot[ROBOT8_LEG_COUNT][3];
extern const fp32 robot_gait_phase_offset[ROBOT8_LEG_COUNT];

/* 台架标定目标需要在线调整，因此保留为可写数组。 */
extern fp32 xyz_Dog_control_debug[ROBOT8_LEG_COUNT][3];

/* 控制周期、姿态过渡时间单位 s；姿态端点允许的单电机最大差值单位 rad。 */
extern fp32 g_control_dt_s;
extern fp32 g_getup_duration_s;
extern fp32 g_stand_transition_s;
extern fp32 g_getdown_duration_s;
extern fp32 g_pose_max_delta_rad;
extern volatile uint8_t g_pose_delta_limit_enable;
extern volatile uint8_t g_pose_debug_retarget_request;
extern volatile uint32_t g_pose_debug_retarget_seq;

/* 摆动相占比无量纲；周期和渐入时间单位 s；步长、转向步长和步高单位 mm。 */
extern fp32 g_gait_swing_ratio;
extern fp32 g_gait_period_min_s;
extern fp32 g_gait_period_max_s;
extern fp32 g_gait_ramp_s;
extern fp32 g_walk_step_max_mm;
extern fp32 g_turn_step_max_mm;
extern fp32 g_gait_height_mm;
extern volatile uint8_t g_gait_joystick_speed_enable;
extern fp32 g_gait_fixed_command;

/* 电机位置模式增益；起立时从低增益平滑过渡到站立增益。 */
extern fp32 g_getup_kp_start;
extern fp32 g_getup_kd_start;
extern fp32 g_stand_kp;
extern fp32 g_stand_kd;
extern fp32 g_walk_kp;
extern fp32 g_walk_kd;
extern fp32 g_down_kp;
extern fp32 g_down_kd;
extern fp32 g_safe_hold_kd;

/* 电机位置限位单位 rad，其余为发送增益/前馈力矩安全上限。 */
extern fp32 g_safe_pos_max;
extern fp32 g_safe_pos_min;
extern fp32 g_safe_kp_max;
extern fp32 g_safe_kd_max;
extern fp32 g_safe_tff_max;

#endif
