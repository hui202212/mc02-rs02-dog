#include "attitude_helper.h"
#include "arm_math.h"

/*==============================================================================
 *  机体→导航系（四元数旋转）
 *
 *  旋转矩阵 R(q) 将机体系向量转到导航系：
 *    R = [1-2(q2²+q3²),  2(q1q2-q0q3),  2(q1q3+q0q2)
 *         2(q1q2+q0q3),  1-2(q1²+q3²),  2(q2q3-q0q1)
 *         2(q1q3-q0q2),  2(q2q3+q0q1),  1-2(q1²+q2²)]
 *============================================================================*/
void BodyFrameToEarthFrame(const fp32 *vecBF, fp32 *vecEF, const fp32 *q)
{
    fp32 q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    fp32 x = vecBF[0], y = vecBF[1], z = vecBF[2];

    vecEF[0] = (1.0f - 2.0f*(q2*q2 + q3*q3)) * x
             + 2.0f * (q1*q2 - q0*q3) * y
             + 2.0f * (q1*q3 + q0*q2) * z;

    vecEF[1] = 2.0f * (q1*q2 + q0*q3) * x
             + (1.0f - 2.0f*(q1*q1 + q3*q3)) * y
             + 2.0f * (q2*q3 - q0*q1) * z;

    vecEF[2] = 2.0f * (q1*q3 - q0*q2) * x
             + 2.0f * (q2*q3 + q0*q1) * y
             + (1.0f - 2.0f*(q1*q1 + q2*q2)) * z;
}

/*==============================================================================
 *  导航系→机体（逆变换，R^T）
 *============================================================================*/
void EarthFrameToBodyFrame(const fp32 *vecEF, fp32 *vecBF, const fp32 *q)
{
    fp32 q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    fp32 x = vecEF[0], y = vecEF[1], z = vecEF[2];

    vecBF[0] = (1.0f - 2.0f*(q2*q2 + q3*q3)) * x
             + 2.0f * (q1*q2 + q0*q3) * y
             + 2.0f * (q1*q3 - q0*q2) * z;

    vecBF[1] = 2.0f * (q1*q2 - q0*q3) * x
             + (1.0f - 2.0f*(q1*q1 + q3*q3)) * y
             + 2.0f * (q2*q3 + q0*q1) * z;

    vecBF[2] = 2.0f * (q1*q3 + q0*q2) * x
             + 2.0f * (q2*q3 - q0*q1) * y
             + (1.0f - 2.0f*(q1*q1 + q2*q2)) * z;
}

/*==============================================================================
 *  四元数 → 欧拉角
 *
 *  roll  = atan2( 2(q0q1 + q2q3),  1 - 2(q1² + q2²))
 *  pitch = asin( 2(q0q2 - q1q3) )
 *  yaw   = atan2( 2(q0q3 + q1q2),  1 - 2(q2² + q3²))
 *============================================================================*/
void QuaternionToEuler(const fp32 *q, fp32 *roll, fp32 *pitch, fp32 *yaw)
{
    fp32 q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

    *roll  = atan2f(2.0f * (q0*q1 + q2*q3),
                    1.0f - 2.0f * (q1*q1 + q2*q2));
    *pitch = asinf(2.0f * (q0*q2 - q1*q3));
    *yaw   = atan2f(2.0f * (q0*q3 + q1*q2),
                    1.0f - 2.0f * (q2*q2 + q3*q3));
}

/*==============================================================================
 *  欧拉角 → 四元数
 *============================================================================*/
void EulerToQuaternion(fp32 roll, fp32 pitch, fp32 yaw, fp32 *q)
{
    fp32 cr = arm_cos_f32(roll * 0.5f);
    fp32 sr = arm_sin_f32(roll * 0.5f);
    fp32 cp = arm_cos_f32(pitch * 0.5f);
    fp32 sp = arm_sin_f32(pitch * 0.5f);
    fp32 cy = arm_cos_f32(yaw * 0.5f);
    fp32 sy = arm_sin_f32(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

/*==============================================================================
 *  机身姿态自稳补偿
 *
 *  当机身绕 X 轴倾斜 (roll) 或绕 Y 轴倾斜 (pitch) 时，
 *  足端在机体坐标系中的目标位置需要反向旋转。
 *
 *  等效于：保持足端在世界坐标系中的位置不变。
 *    foot_corrected = R^T(roll, pitch) * foot_original
 *    其中 R = Ry(pitch) * Rx(roll)，所以 R^T = Rx(-roll) * Ry(-pitch)
 *
 *  lean_comp_height：控制补偿强度的权重高度。
 *  设为 ~140mm（机身重心到地面高度）在物理上就是正确的。
 *  增大 → 恢复力更强，但可能振荡。
 *============================================================================*/
void BodyAttitudeCompensate(fp32 *foot_x, fp32 *foot_y, fp32 *foot_z,
                            fp32 roll, fp32 pitch,
                            fp32 lean_comp_height)
{
    fp32 sr = arm_sin_f32(roll);
    fp32 cr = arm_cos_f32(roll);
    fp32 sp = arm_sin_f32(pitch);
    fp32 cp = arm_cos_f32(pitch);

    fp32 x = *foot_x;
    fp32 y = *foot_y;
    fp32 z = *foot_z;

    /* 1) 绕 X 轴反向旋转 -roll */
    fp32 y1 =  y * cr + z * sr;
    fp32 z1 = -y * sr + z * cr;

    /* 2) 绕 Y 轴反向旋转 -pitch */
    fp32 x1 =  x * cp - z1 * sp;
    fp32 z2 =  x * sp + z1 * cp;

    /* 3) 附加恢复力项：机身倾斜时额外偏移足端 */
    /*    偏移量 = lean_comp_height × tilt_angle */
    /*    这块是"比例控制"的物理解释 */
    fp32 dx = lean_comp_height * pitch;  /* pitch → X 方向偏移 */
    fp32 dy = lean_comp_height * roll;   /* roll  → Y 方向偏移 */

    *foot_x = x1 + dx;
    *foot_y = y1 + dy;
    *foot_z = z2;
}
