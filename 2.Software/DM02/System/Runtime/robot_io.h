#ifndef ROBOT_IO_H
#define ROBOT_IO_H

#include "struct_typedef.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Motion-control port.  The application and control layers address joints by
 * leg/joint only; CAN bus, motor ID, and LZ protocol details stay in Runtime.
 */
fp32 robot_io_joint_position(uint8_t leg, uint8_t joint);
uint8_t robot_io_joint_feedback_valid(uint8_t leg, uint8_t joint);
void robot_io_set_joint_target(uint8_t leg, uint8_t joint, fp32 position,
                               fp32 velocity, fp32 kp, fp32 kd, fp32 torque);

uint8_t robot_io_all_ready(void);
uint8_t robot_io_faulted(void);
void robot_io_disarm(void);

#ifdef __cplusplus
}
#endif

#endif
