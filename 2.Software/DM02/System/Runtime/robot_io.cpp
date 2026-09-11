#include "robot_io.h"

#include "motor_task.h"
#include "robot_config.h"

#include <math.h>

static const uint8_t kLegBus[SIMPLE_LEG_COUNT] = {
    SIMPLE_RF_BUS, SIMPLE_LF_BUS, SIMPLE_RH_BUS, SIMPLE_LH_BUS,
};

static const uint8_t kLegMotorIndex[SIMPLE_LEG_COUNT][SIMPLE_MOTORS_PER_LEG] = {
    {SIMPLE_RF_MOTOR1_INDEX, SIMPLE_RF_MOTOR2_INDEX},
    {SIMPLE_LF_MOTOR1_INDEX, SIMPLE_LF_MOTOR2_INDEX},
    {SIMPLE_RH_MOTOR1_INDEX, SIMPLE_RH_MOTOR2_INDEX},
    {SIMPLE_LH_MOTOR1_INDEX, SIMPLE_LH_MOTOR2_INDEX},
};

static motor_lz_control *robot_io_motor(uint8_t leg, uint8_t joint)
{
    if (leg >= SIMPLE_LEG_COUNT || joint >= SIMPLE_MOTORS_PER_LEG) {
        return 0;
    }

    const uint8_t index = kLegMotorIndex[leg][joint];
    return kLegBus[leg] == 0U ? &motor_lz_data_can1[index]
                              : &motor_lz_data_can2[index];
}

fp32 robot_io_joint_position(uint8_t leg, uint8_t joint)
{
    motor_lz_control *motor = robot_io_motor(leg, joint);
    return motor == 0 ? NAN : motor->recv.Now_Pos;
}

uint8_t robot_io_joint_feedback_valid(uint8_t leg, uint8_t joint)
{
    return isfinite(robot_io_joint_position(leg, joint)) ? 1U : 0U;
}

void robot_io_set_joint_target(uint8_t leg, uint8_t joint, fp32 position,
                               fp32 velocity, fp32 kp, fp32 kd, fp32 torque)
{
    motor_lz_control *motor = robot_io_motor(leg, joint);
    if (motor == 0) {
        return;
    }

    motor->send.Pos = position;
    motor->send.W = velocity;
    motor->send.Kp = kp;
    motor->send.Kd = kd;
    motor->send.T_ff = torque;
}

uint8_t robot_io_all_ready(void)
{
    return motor_task_all_ready();
}

uint8_t robot_io_faulted(void)
{
    return motor_task_faulted();
}

void robot_io_disarm(void)
{
    motor_task_disarm_all();
}
