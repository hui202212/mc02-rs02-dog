/**
 * @file    mahony_filter.h
 * @brief   Mahony 互补滤波姿态解算算法（独立平台版本）
 *
 * 从原项目中提取，移除了 stm32h7xx.h 和 arm_math.h 依赖。
 * 需配合标准 C 数学库 <math.h>。
 */
#ifndef _MAHONY_FILTER_H
#define _MAHONY_FILTER_H

#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#define DEG2RAD 0.0174533f
#define RAD2DEG 57.295671f

typedef struct Axis3f_t
{
    float x;
    float y;
    float z;
} Axis3f;

struct MAHONY_FILTER_t
{
    float Kp, Ki;
    float dt;
    Axis3f gyro, acc;

    float exInt, eyInt, ezInt;
    float q0, q1, q2, q3;
    float rMat[3][3];

    float pitch, roll, yaw;

    void (*mahony_init)(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);
    void (*mahony_input)(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);
    void (*mahony_update)(struct MAHONY_FILTER_t *mahony_filter);
    void (*mahony_output)(struct MAHONY_FILTER_t *mahony_filter);
    void (*RotationMatrix_update)(struct MAHONY_FILTER_t *mahony_filter);
};

void mahony_init(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt);
void mahony_input(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc);
void mahony_update(struct MAHONY_FILTER_t *mahony_filter);
void mahony_output(struct MAHONY_FILTER_t *mahony_filter);
void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter);

#endif /* _MAHONY_FILTER_H */
