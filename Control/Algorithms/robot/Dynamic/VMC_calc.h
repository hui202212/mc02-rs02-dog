#ifndef VMC_CALC_H
#define VMC_CALC_H

/**
 * @file    VMC_calc.h
 * @brief   虚拟模型控制 (Virtual Model Control) — 四足机器人
 *
 * 每条腿抽象为一根"虚拟腿"（髋→足端连线），
 * 用极坐标 (L0, theta, phi_roll) 描述，IMU 融合姿态。
 * 虚拟弹簧-阻尼产生力/力矩，经简化雅可比映射到关节力矩。
 */

#include "struct_typedef.h"
#include "imu_task.h"

#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 *  单腿 VMC 数据结构
 *============================================================================*/
typedef struct {
    /* --- 虚拟腿极坐标 --- */
    fp32 L0;            /* 虚拟腿长度 (m) */
    fp32 theta;         /* 虚拟腿前后倾角 (rad), 前倾为正 */
    fp32 phi_roll;      /* 虚拟腿侧倾角 (rad), 右倾为正 */

    /* --- 一阶导数 --- */
    fp32 d_L0;          /* 伸缩速度 (m/s) */
    fp32 d_theta;       /* 前后角速度 (rad/s) */
    fp32 d_phi_roll;    /* 侧向角速度 (rad/s) */

    /* --- 二阶导数 --- */
    fp32 dd_L0;         /* 伸缩加速度 (m/s²) */
    fp32 dd_theta;      /* 角加速度 (rad/s²) */

    /* --- 状态缓存（后向差分用） --- */
    fp32 last_L0;
    fp32 last_theta;
    fp32 last_phi_roll;
    fp32 last_d_theta;
    fp32 last_d_L0;

    /* --- 足端位置缓存 (mm) --- */
    fp32 foot_x, foot_y, foot_z;

    /* --- 输出：关节力矩 (N·m) --- */
    fp32 torque_set[3]; /* [hip_roll, hip_pitch, knee] */

    /* --- 地面接触估计 --- */
    fp32 FN;            /* 法向支撑力估计 (N) */
    uint8_t contact;    /* 1=触地 */

    /* --- 初始化标志 --- */
    uint8_t first_flag;
} vmc_leg_t;

/*==============================================================================
 *  API
 *============================================================================*/

/**
 * @brief  初始化虚拟腿
 */
void VMC_Init(vmc_leg_t *vmc);

/**
 * @brief  重置虚拟腿微分缓存（模式切换时调用，避免一阶导数瞬态毛刺）
 *         将 last_* 设为当前值，d_* 清 0
 */
void VMC_ResetDerivatives(vmc_leg_t *vmc);

/**
 * @brief  根据足端位置 + IMU 更新虚拟腿状态
 *
 * @param  vmc             单腿 VMC 数据
 * @param  foot_x,y,z      足端位置 (mm)
 * @param  ins             IMU 数据（用于 Gyro 导数修正）
 * @param  dt              采样周期 (s)
 * @param  leg_id          腿编号 [0,3]，用于 theta 镜像
 */
void VMC_Update(vmc_leg_t *vmc,
                fp32 foot_x, fp32 foot_y, fp32 foot_z,
                const IMU_t *ins, fp32 dt, uint8_t leg_id);

/**
 * @brief  根据虚拟腿力/力矩计算关节力矩
 *
 * @param  vmc    单腿数据（需先调用 VMC_Update）
 * @param  F0     虚拟腿轴向力 (N)，正=压缩
 * @param  Tp     虚拟腿俯仰力矩 (N·m)，正=前倾
 * @param  Tr     虚拟腿侧倾力矩 (N·m)，正=右倾
 */
void VMC_ComputeTorque(vmc_leg_t *vmc, fp32 F0, fp32 Tp, fp32 Tr);

/**
 * @brief  地面接触检测
 */
uint8_t VMC_GroundDetection(vmc_leg_t *vmc);

#ifdef __cplusplus
}
#endif

#endif /* VMC_CALC_H */
