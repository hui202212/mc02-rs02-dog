#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "struct_typedef.h"
#include "control_behaviour.h"
#include "ik.h"

extern kinematics_connection_12bot_robot robot_dog_ik[4];

extern volatile uint8_t robot_control_fault_code;
extern volatile uint8_t robot_control_fault_leg;
extern volatile uint8_t robot_control_fault_joint;
extern volatile uint8_t robot_control_getup_stage_watch;
extern volatile fp32 robot_control_fault_start_pos;
extern volatile fp32 robot_control_fault_target_pos;
extern volatile fp32 robot_control_fault_delta_pos;

/* 上层控制故障码，可直接加入 Keil Watch。故障一旦产生，由 control_task
 * 统一进入 Dog_control_fault 并调用 motor_task_disarm_all()。 */
enum robot_control_fault
{
    ROBOT_CONTROL_FAULT_NONE = 0,
    ROBOT_CONTROL_FAULT_FEEDBACK = 1,      /* 电机反馈包含 NaN/Inf */
    ROBOT_CONTROL_FAULT_IK = 2,            /* 足端不可达、支链跳变或闭链无解 */
    ROBOT_CONTROL_FAULT_TARGET_LIMIT = 3,  /* 目标角非法或任一电机过渡角差超限 */
};

void robot_control_init(void);
void robot_control_enter_mode(remote_control_mode mode);

void robot_control_safe_hold(void);
void robot_control_Dog_control_Get_down(fp32 dt);
void robot_control_Dog_control_Get_up(fp32 dt);
void robot_control_Dog_control_stand(fp32 dt);
void robot_control_Dog_control_walk(fp32 dt, fp32 forward_command);
void robot_control_Dog_control_turn(fp32 dt, fp32 turn_command);

uint8_t robot_control_pose_done(void);
uint8_t robot_control_faulted(void);

/* 台架调试入口：不经过姿态过渡，只允许在架空、受控的标定固件中调用。 */
void robot_control_Dog_control_debug(void);

#endif
