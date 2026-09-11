#ifndef ATTITUDE_HELPER_H
#define ATTITUDE_HELPER_H

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief  将向量从机体坐标系 (Body Frame) 转换到导航系 (Earth Frame)
 * @param  vecBF  输入：机体系向量 [3]
 * @param  vecEF  输出：导航系向量 [3]
 * @param  q      四元数 [q0, q1, q2, q3]
 */
void BodyFrameToEarthFrame(const fp32 *vecBF, fp32 *vecEF, const fp32 *q);

/**
 * @brief  将向量从导航系 (Earth Frame) 转换到机体坐标系 (Body Frame)
 * @param  vecEF  输入：导航系向量 [3]
 * @param  vecBF  输出：机体系向量 [3]
 * @param  q      四元数 [q0, q1, q2, q3]
 */
void EarthFrameToBodyFrame(const fp32 *vecEF, fp32 *vecBF, const fp32 *q);

/**
 * @brief  从四元数提取欧拉角 (rad)
 * @param  q      四元数 [q0,q1,q2,q3]
 * @param  roll   输出：横滚角 (rad)
 * @param  pitch  输出：俯仰角 (rad)
 * @param  yaw    输出：偏航角 (rad)
 */
void QuaternionToEuler(const fp32 *q, fp32 *roll, fp32 *pitch, fp32 *yaw);

/**
 * @brief  欧拉角转四元数
 * @param  roll   横滚角 (rad)
 * @param  pitch  俯仰角 (rad)
 * @param  yaw    偏航角 (rad)
 * @param  q      输出：四元数 [q0,q1,q2,q3]
 */
void EulerToQuaternion(fp32 roll, fp32 pitch, fp32 yaw, fp32 *q);

/**
 * @brief  机身姿态自稳补偿
 *
 * 当机身倾斜（roll/pitch）时，修正足端目标位置，
 * 使足端在世界坐标系中保持不变，从而让机身恢复水平。
 *
 * 原理：将足端坐标绕机身旋转的反向旋转回去。
 *
 * @param  foot_x, foot_y, foot_z  IN/OUT: 足端位置 (mm)，机体坐标系
 * @param  roll   机身横滚角 (rad)
 * @param  pitch  机身俯仰角 (rad)
 * @param  lean_comp_height  补偿权重高度 (mm)，约等于机身重心到地面高度
 */
void BodyAttitudeCompensate(fp32 *foot_x, fp32 *foot_y, fp32 *foot_z,
                            fp32 roll, fp32 pitch,
                            fp32 lean_comp_height);

#ifdef __cplusplus
}
#endif


#endif
