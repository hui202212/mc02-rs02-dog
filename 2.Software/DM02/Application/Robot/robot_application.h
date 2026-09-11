#ifndef ROBOT_APPLICATION_H
#define ROBOT_APPLICATION_H

#include "robot_config.h"
#include "struct_typedef.h"

enum SimpleRobotState
{
    /* Keil Watch 中 simple_robot_state_watch 对应以下数值。 */
    SIMPLE_STATE_WAIT = 0,
    SIMPLE_STATE_GET_UP = 1,
    SIMPLE_STATE_STAND = 2,
    SIMPLE_STATE_WALK = 3,
    SIMPLE_STATE_TURN = 4,
    SIMPLE_STATE_GET_DOWN = 5,
    SIMPLE_STATE_CRAWL = 6,
    SIMPLE_STATE_FAULT = 7,
    SIMPLE_STATE_JUMP = 8,
    SIMPLE_STATE_DANCE = 9,
};

enum SimpleRobotFault
{
    /* 故障只锁存第一个原因，断电排查后再重新上电。 */
    SIMPLE_FAULT_NONE = 0,
    SIMPLE_FAULT_NOT_CALIBRATED = 1,
    SIMPLE_FAULT_MOTOR = 2,
    SIMPLE_FAULT_FEEDBACK = 3,
    SIMPLE_FAULT_IK = 4,
    SIMPLE_FAULT_TARGET_LIMIT = 5,
};

enum SimpleObstacleMode
{
    SIMPLE_OBSTACLE_NORMAL = 0,          /* 高速转场、直角绕杆。 */
    SIMPLE_OBSTACLE_PIT = 1,             /* L形砂砾碎木坑。 */
    SIMPLE_OBSTACLE_LIMIT_BAR = 2,       /* 300mm限高杆。 */
    SIMPLE_OBSTACLE_CROSS_SLOPE_10 = 3,  /* 右高左低横穿10度长坡。 */
    SIMPLE_OBSTACLE_BRIDGE_A = 4,        /* 木桥A：加速稳健四拍。 */
    SIMPLE_OBSTACLE_BRIDGE_B = 5         /* 木桥B：高抬脚四拍走桥。 */
};

enum SimpleJumpMode
{
    SIMPLE_JUMP_VERTICAL = 0, /* 历史原地跳参数，演示版无遥控入口。 */
    SIMPLE_JUMP_FORWARD = 1,  /* 历史普通前跳参数，演示版无遥控入口。 */
    SIMPLE_JUMP_BRIDGE_B = 2, /* 演示版R1：四腿同步向前小跳。 */
    SIMPLE_JUMP_T_STEP_UP = 3,/* 比赛版T台上台参数，演示版无遥控入口。 */
    SIMPLE_JUMP_T_STEP_DOWN = 4,/* 比赛版T台下台参数，演示版无遥控入口。 */
    SIMPLE_JUMP_WALL = 5      /* 历史高墙参数，演示版无遥控入口。 */
};

/* 以下变量只用于 Keil Watch，不参与控制决策。 */
extern volatile uint8_t simple_robot_state_watch;
extern volatile uint8_t simple_robot_fault_watch;
extern volatile uint8_t simple_robot_fault_leg_watch;
extern volatile uint8_t simple_robot_fault_motor_watch;
extern volatile uint8_t simple_robot_obstacle_mode_watch;
extern volatile fp32 simple_robot_side_z_comp_mm_watch;
extern volatile uint8_t simple_robot_imu_ready_watch;
extern volatile fp32 simple_robot_roll_raw_watch;
extern volatile fp32 simple_robot_roll_zero_watch;
extern volatile fp32 simple_robot_roll_relative_watch;
extern volatile fp32 simple_robot_imu_comp_add_mm_watch;
/* 普通跳跃/木桥B前跳0~7；T台上台0~13；T台下台0~11；255表示无动作。 */
extern volatile uint8_t simple_robot_jump_mode_watch;
extern volatile uint8_t simple_robot_jump_stage_watch;
extern volatile uint8_t simple_robot_jump_abort_watch;
extern volatile uint32_t simple_robot_jump_count_watch;
extern volatile uint8_t simple_robot_dance_stage_watch;
extern volatile uint32_t simple_robot_dance_count_watch;
extern volatile fp32 simple_robot_foot_x_watch[SIMPLE_LEG_COUNT];
extern volatile fp32 simple_robot_foot_z_watch[SIMPLE_LEG_COUNT];
extern volatile fp32 simple_robot_motor_target_watch[SIMPLE_LEG_COUNT][2];

/* Application boundary for the robot behavior state machine. */
void robot_application_init(void);
void robot_application_step_1ms(void);

#endif
