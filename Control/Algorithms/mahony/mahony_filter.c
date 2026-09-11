/**
 * @file    mahony_filter.c
 * @brief   Mahony 互补滤波算法实现
 *
 * 从原项目提取，仅依赖 <math.h>，无任何平台相关代码。
 */
#include "mahony_filter.h"

void RotationMatrix_update(struct MAHONY_FILTER_t *mahony_filter)
{
    float q1q1 = mahony_filter->q1 * mahony_filter->q1;
    float q2q2 = mahony_filter->q2 * mahony_filter->q2;
    float q3q3 = mahony_filter->q3 * mahony_filter->q3;

    float q0q1 = mahony_filter->q0 * mahony_filter->q1;
    float q0q2 = mahony_filter->q0 * mahony_filter->q2;
    float q0q3 = mahony_filter->q0 * mahony_filter->q3;
    float q1q2 = mahony_filter->q1 * mahony_filter->q2;
    float q1q3 = mahony_filter->q1 * mahony_filter->q3;
    float q2q3 = mahony_filter->q2 * mahony_filter->q3;

    mahony_filter->rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    mahony_filter->rMat[0][1] = 2.0f * (q1q2 - q0q3);
    mahony_filter->rMat[0][2] = 2.0f * (q1q3 + q0q2);

    mahony_filter->rMat[1][0] = 2.0f * (q1q2 + q0q3);
    mahony_filter->rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    mahony_filter->rMat[1][2] = 2.0f * (q2q3 - q0q1);

    mahony_filter->rMat[2][0] = 2.0f * (q1q3 - q0q2);
    mahony_filter->rMat[2][1] = 2.0f * (q2q3 + q0q1);
    mahony_filter->rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void mahony_input(struct MAHONY_FILTER_t *mahony_filter, Axis3f gyro, Axis3f acc)
{
    mahony_filter->gyro = gyro;
    mahony_filter->acc = acc;
}

void mahony_update(struct MAHONY_FILTER_t *mahony_filter)
{
    float normalise;
    float ex, ey, ez;

    normalise = sqrtf(mahony_filter->acc.x * mahony_filter->acc.x
                    + mahony_filter->acc.y * mahony_filter->acc.y
                    + mahony_filter->acc.z * mahony_filter->acc.z);

    mahony_filter->acc.x = mahony_filter->acc.x / normalise;
    mahony_filter->acc.y = mahony_filter->acc.y / normalise;
    mahony_filter->acc.z = mahony_filter->acc.z / normalise;

    /* 加速度计方向与旋转矩阵重力方向叉积 = 误差 */
    ex = (mahony_filter->acc.y * mahony_filter->rMat[2][2] - mahony_filter->acc.z * mahony_filter->rMat[2][1]);
    ey = (mahony_filter->acc.z * mahony_filter->rMat[2][0] - mahony_filter->acc.x * mahony_filter->rMat[2][2]);
    ez = (mahony_filter->acc.x * mahony_filter->rMat[2][1] - mahony_filter->acc.y * mahony_filter->rMat[2][0]);

    /* 积分误差 */
    mahony_filter->exInt += mahony_filter->Ki * ex * mahony_filter->dt;
    mahony_filter->eyInt += mahony_filter->Ki * ey * mahony_filter->dt;
    mahony_filter->ezInt += mahony_filter->Ki * ez * mahony_filter->dt;

    /* PI 补偿修正陀螺仪 */
    mahony_filter->gyro.x += mahony_filter->Kp * ex + mahony_filter->exInt;
    mahony_filter->gyro.y += mahony_filter->Kp * ey + mahony_filter->eyInt;
    mahony_filter->gyro.z += mahony_filter->Kp * ez + mahony_filter->ezInt;

    /* 一阶龙格-库塔更新四元数 */
    float q0Last = mahony_filter->q0;
    float q1Last = mahony_filter->q1;
    float q2Last = mahony_filter->q2;
    float q3Last = mahony_filter->q3;
    float halfT = mahony_filter->dt * 0.5f;

    mahony_filter->q0 += (-q1Last * mahony_filter->gyro.x - q2Last * mahony_filter->gyro.y - q3Last * mahony_filter->gyro.z) * halfT;
    mahony_filter->q1 += (q0Last * mahony_filter->gyro.x + q2Last * mahony_filter->gyro.z - q3Last * mahony_filter->gyro.y) * halfT;
    mahony_filter->q2 += (q0Last * mahony_filter->gyro.y - q1Last * mahony_filter->gyro.z + q3Last * mahony_filter->gyro.x) * halfT;
    mahony_filter->q3 += (q0Last * mahony_filter->gyro.z + q1Last * mahony_filter->gyro.y - q2Last * mahony_filter->gyro.x) * halfT;

    /* 归一化四元数 */
    normalise = sqrtf(mahony_filter->q0 * mahony_filter->q0
                    + mahony_filter->q1 * mahony_filter->q1
                    + mahony_filter->q2 * mahony_filter->q2
                    + mahony_filter->q3 * mahony_filter->q3);

    mahony_filter->q0 /= normalise;
    mahony_filter->q1 /= normalise;
    mahony_filter->q2 /= normalise;
    mahony_filter->q3 /= normalise;

    /* 更新旋转矩阵 */
    mahony_filter->RotationMatrix_update(mahony_filter);
}

void mahony_output(struct MAHONY_FILTER_t *mahony_filter)
{
    mahony_filter->pitch = asinf(mahony_filter->rMat[2][0]);
    mahony_filter->roll  = atan2f(mahony_filter->rMat[2][1], mahony_filter->rMat[2][2]);
    mahony_filter->yaw   = atan2f(mahony_filter->rMat[1][0], mahony_filter->rMat[0][0]);
}

void mahony_init(struct MAHONY_FILTER_t *mahony_filter, float Kp, float Ki, float dt)
{
    mahony_filter->Kp = Kp;
    mahony_filter->Ki = Ki;
    mahony_filter->dt = dt;
    mahony_filter->q0 = 1;
    mahony_filter->q1 = 0;
    mahony_filter->q2 = 0;
    mahony_filter->q3 = 0;
    mahony_filter->exInt = 0;
    mahony_filter->eyInt = 0;
    mahony_filter->ezInt = 0;

    mahony_filter->mahony_init            = mahony_init;
    mahony_filter->mahony_input           = mahony_input;
    mahony_filter->mahony_update          = mahony_update;
    mahony_filter->mahony_output          = mahony_output;
    mahony_filter->RotationMatrix_update  = RotationMatrix_update;
}
