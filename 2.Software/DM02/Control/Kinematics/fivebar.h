#ifndef FIVEBAR_H
#define FIVEBAR_H

#include "robot_config.h"
#include "struct_typedef.h"

enum SimpleLeg
{
    /* 固定腿序：右前、左前、右后、左后。Watch 中的 leg 也使用这个编号。 */
    SIMPLE_RF = 0,
    SIMPLE_LF = 1,
    SIMPLE_RH = 2,
    SIMPLE_LH = 3,
};

struct FiveBarState
{
    /* 上一次电机目标用于连续性检查，防止命令单周期突跳。 */
    fp32 reference_motor[2];
    uint8_t reference_valid;
};

void fivebar_set_reference(FiveBarState *state, fp32 motor1, fp32 motor2);

/* 本腿局部足端目标 (+X 向前、+Z_DOWN 向下) -> 按 CAN 顺序排列的两个电机角。 */
uint8_t fivebar_inverse(FiveBarState *state, uint8_t leg,
                        fp32 foot_x, fp32 foot_z_down,
                        fp32 motor_out[2]);

/* 起立/趴下插值时检查当前两个电机角能否组成合法闭链。 */
uint8_t fivebar_motor_pose_valid(uint8_t leg, fp32 motor1, fp32 motor2);

#endif
