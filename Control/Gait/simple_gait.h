#ifndef SIMPLE_GAIT_H
#define SIMPLE_GAIT_H

#include "fivebar.h"

struct SimpleFootTarget
{
    /* 一只脚在本腿平面内的位置，单位 mm。 */
    fp32 x;
    fp32 z_down;
};

struct SimpleGaitState
{
    /* phase: 当前走到一个周期的哪里；ramp: 刚起步时从 0 缓慢放大步幅。 */
    fp32 phase;
    fp32 ramp;
    /* 摇杆斜坡后的指令，避免手抖造成步幅和周期瞬间跳变。 */
    fp32 forward_command;
    fp32 turn_command;
};

enum SimpleGaitPattern
{
    SIMPLE_GAIT_TROT = 0,       /* 正常行走：两组对角腿交替。 */
    SIMPLE_GAIT_CRAWL = 1,      /* 低姿慢速匍匐：每次只抬一条腿。 */
    SIMPLE_GAIT_FAST_CRAWL = 2, /* 低姿快速小碎步：两组对角腿交替。 */
    SIMPLE_GAIT_FAST_TRANSIT = 3,/* L1高速转场：独立的更短周期。 */
    SIMPLE_GAIT_BRIDGE_A = 4,   /* L2木桥A：独立周期的稳健四拍。 */
    SIMPLE_GAIT_BRIDGE_B = 5    /* R1木桥B：独立周期的高抬脚四拍。 */
};

void simple_gait_reset(SimpleGaitState *state);

/* 每 1 ms 调用一次，输出四只脚下一时刻的位置。输入指令范围 [-1, 1]。 */
void simple_gait_step(SimpleGaitState *state, fp32 dt,
                      fp32 forward_command, fp32 turn_command,
                      SimpleGaitPattern pattern,
                      fp32 base_z_down, fp32 walk_step_max,
                      fp32 turn_step_max, fp32 step_height,
                      SimpleFootTarget foot[SIMPLE_LEG_COUNT]);

#endif
