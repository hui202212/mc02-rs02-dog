/**
 * @file    VMC_calc.c
 * @brief   虚拟模型控制实现 — 虚拟腿极坐标 + 简化雅可比力矩映射
 *
 * ============================================================================
 * 坐标系约定
 * ============================================================================
 *  足端位置 (mm) 来自 IK 逆解, 以髋关节为原点:
 *    +X = 前向, +Y = 左向, +Z = 向上
 *    （注意 foot_z 相对机身向下为负值）
 *
 * 虚拟腿极坐标 (以髋为原点):
 *    L0      = sqrt(x² + y² + z²)           腿长 (m)
 *    theta   = atan2(x, -z)                  前后倾角 (前倾=正)
 *    phi_r   = atan2(y, -z)                  侧倾角 (右倾=正)
 *
 * 雅可比 J (虚拟空间 → 关节空间):
 *    [τ_hip_roll ]   =  J^T  ×  [F0, Tp, Tr]^T
 *    [τ_hip_pitch]
 *    [τ_knee     ]
 *
 *  对于简化虚拟腿模型（仅有 L0/theta/phi_roll 三个自由度）:
 *    J(L0, θ, φ) = d(foot_xyz) / d(L0, θ, φ)
 *
 *    foot_x = L0 × sin(θ) × cos(φ)  ≈  L0 × sin(θ)
 *    foot_y = L0 × sin(φ)            ≈  L0 × φ
 *    foot_z = -L0 × cos(θ) × cos(φ) ≈ -L0 × cos(θ)
 *
 *    J = [sin(θ),  L0×cos(θ),  0           ]
 *        [0,        0,          L0         ]
 *        [-cos(θ),  L0×sin(θ),  0           ]
 *
 * 力矩映射:
 *    τ_hip_roll  = Tr                      (侧倾直接映射)
 *    τ_hip_pitch = F0 × L0 × sin(θ)
 *                + Tp × cos(θ)            (轴向力+俯仰力矩合成)
 *    τ_knee      = Tp × (L0×sin(θ)/物理膝距)
 *                + F0 × L0 × sin(θ) × 分配比
 *
 *  实际实现中采用等效力臂法，比雅可比更直观且调参容易。
 * ============================================================================
 */

#include "VMC_calc.h"
#include "arm_math.h"

/*==============================================================================
 *  初始化
 *============================================================================*/
void VMC_Init(vmc_leg_t *vmc)
{
    vmc->L0          = 0.0f;
    vmc->theta       = 0.0f;
    vmc->phi_roll    = 0.0f;
    vmc->d_L0        = 0.0f;
    vmc->d_theta     = 0.0f;
    vmc->d_phi_roll  = 0.0f;
    vmc->dd_L0       = 0.0f;
    vmc->dd_theta    = 0.0f;

    vmc->last_L0       = 0.0f;
    vmc->last_theta    = 0.0f;
    vmc->last_phi_roll = 0.0f;
    vmc->last_d_theta  = 0.0f;
    vmc->last_d_L0     = 0.0f;

    vmc->first_flag    = 1;
    vmc->contact       = 0;
    vmc->FN            = 0.0f;

    for (uint8_t i = 0; i < 3; i++)
        vmc->torque_set[i] = 0.0f;
}

/*==============================================================================
 *  重置微分缓存
 *============================================================================*/
void VMC_ResetDerivatives(vmc_leg_t *vmc)
{
    vmc->last_L0       = vmc->L0;
    vmc->last_theta    = vmc->theta;
    vmc->last_phi_roll = vmc->phi_roll;
    vmc->d_L0          = 0.0f;
    vmc->d_theta       = 0.0f;
    vmc->d_phi_roll    = 0.0f;
    vmc->last_d_theta  = 0.0f;
    vmc->last_d_L0     = 0.0f;
    vmc->first_flag    = 0;
}

/*==============================================================================
 *  更新虚拟腿状态
 *
 *  输入:
 *    foot_x, foot_y, foot_z — 当前足端位置 (mm, 髋关节为原点)
 *    ins                    — IMU 数据 (用于机身角速度反馈)
 *    dt                     — 采样周期 (s)
 *    leg_id                 — 腿编号 0~3 (保留, 未使用)
 *============================================================================*/
void VMC_Update(vmc_leg_t *vmc,
                fp32 foot_x, fp32 foot_y, fp32 foot_z,
                const IMU_t *ins, fp32 dt, uint8_t leg_id)
{
    fp32 fx = foot_x * 0.001f;   /* mm → m */
    fp32 fy = foot_y * 0.001f;
    fp32 fz = foot_z * 0.001f;   /* 注意: 机身下负值 */

    /* ---- 极坐标 ---- */
    arm_sqrt_f32(fx*fx + fy*fy + fz*fz, &vmc->L0);
    if (vmc->L0 < 0.001f) vmc->L0 = 0.001f;

    /* theta: 前后角。atan2(x, -z): fz 为负时 theta 为正 */
    /* ★ 所有腿使用一致的物理角度, 不再分左右镜像。*/
    /*   符号修正统一由 robot_vmc_apply_torque 的 VMC_LEG_SIGN 表在输出端完成。 */
    vmc->theta = atan2f(fx, -fz);

    /* phi_roll: 侧倾角 */
    vmc->phi_roll = atan2f(fy, -fz);

    /* ---- 数值微分 (后向差分 + 低通) ---- */
    static const fp32 ALPHA = 0.3f;   /* 微分低通 (越小越平滑) */
    fp32 dt_inv = 1.0f / dt;

    if (vmc->first_flag) {
        vmc->last_L0       = vmc->L0;
        vmc->last_theta    = vmc->theta;
        vmc->last_phi_roll = vmc->phi_roll;
        vmc->d_L0          = 0.0f;
        vmc->d_theta       = 0.0f;
        vmc->d_phi_roll    = 0.0f;
        vmc->first_flag    = 0;
    }

    vmc->d_L0  = ALPHA * (vmc->L0 - vmc->last_L0) * dt_inv
               + (1.0f - ALPHA) * vmc->d_L0;

    vmc->d_theta  = ALPHA * (vmc->theta - vmc->last_theta) * dt_inv
                  + (1.0f - ALPHA) * vmc->d_theta;

    vmc->d_phi_roll = ALPHA * (vmc->phi_roll - vmc->last_phi_roll) * dt_inv
                     + (1.0f - ALPHA) * vmc->d_phi_roll;

    /* ---- 二阶导数 (用于惯性补偿) ---- */
    vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) * dt_inv;
    vmc->dd_L0    = (vmc->d_L0 - vmc->last_d_L0) * dt_inv;

    /* ---- 缓存 ---- */
    vmc->last_L0       = vmc->L0;
    vmc->last_theta    = vmc->theta;
    vmc->last_phi_roll = vmc->phi_roll;
    vmc->last_d_theta  = vmc->d_theta;
    vmc->last_d_L0     = vmc->d_L0;

    vmc->foot_x = foot_x;
    vmc->foot_y = foot_y;
    vmc->foot_z = foot_z;
}

/*==============================================================================
 *  虚拟力 → 关节力矩
 *
 *  虚拟弹簧-阻尼模型:
 *    F0 = K_L0 × (L0_target - L0)  +  B_L0 × (0 - d_L0)
 *    Tp = K_th × (th_target - th)  +  B_th × (0 - d_theta)
 *    Tr = K_ph × (ph_target - ph)  +  B_ph × (0 - d_phi_roll)
 *
 *  注意: F0/Tp/Tr 在外部计算好传入, 本函数只做映射
 *
 *  映射方法——力臂法(比雅可比更直观):
 *
 *  虚拟腿受力:
 *    F0 沿腿轴向, Tp 绕 Y 轴(俯仰), Tr 绕 X 轴(侧倾)
 *
 *  关节力矩 = 力 × 力臂:
 *    τ_hip_roll  = Tr                          (直接映射)
 *    τ_hip_pitch = F0 × L0 × sin(θ)            (轴向力产生髋关节力矩)
 *                + Tp × (L0/L_k)               (俯仰力矩分配到髋)
 *    τ_knee      = Tp × (L0/L_k) × 分配系数    (俯仰力矩分配到膝)
 *                + F0 × L0 × sin(θ) × 分配系数
 *
 *  其中 L_k 为膝关节到足端的长度, 取物理估测值
 *============================================================================*/

/* 膝关节到足端长度估测 (mm→m), 根据机械结构应 ~0.18m */
#define VMC_KNEE_LEN_M  0.24854f

/* 俯仰力矩到髋/膝的分配比: 髋关节承担 40%, 膝关节承担 60% */
#define VMC_HIP_RATIO   0.40f
#define VMC_KNEE_RATIO  0.60f

void VMC_ComputeTorque(vmc_leg_t *vmc, fp32 F0, fp32 Tp, fp32 Tr)
{
    fp32 L0    = vmc->L0;
    fp32 theta = vmc->theta;

    /* --- 髋关节侧倾：直接映射 --- */
    vmc->torque_set[0] = Tr;

    /* --- 髋关节俯仰 + 膝关节：力臂法 --- */
    /* 轴向力 F0 在髋关节产生的力矩臂 = L0 × sin(θ) */
    fp32 sin_t = arm_sin_f32(theta);
    fp32 cos_t = arm_cos_f32(theta);

    fp32 tau_F0_hip = F0 * L0 * sin_t;

    /* 俯仰力矩 Tp 在髋和膝之间的力臂分配 */
    fp32 tau_Tp_hip  = -Tp * cos_t * VMC_HIP_RATIO;
    fp32 tau_Tp_knee =  Tp * cos_t * VMC_KNEE_RATIO;

    vmc->torque_set[1] = tau_F0_hip  + tau_Tp_hip;   /* hip_pitch */
    vmc->torque_set[2] = tau_Tp_knee;                 /* knee */

    /* --- 法向支撑力估计(正=触地压缩) --- */
    vmc->FN = F0 * cos_t + Tp / L0 * sin_t;
    if (vmc->FN < 0.0f) vmc->FN = 0.0f;
}

/*==============================================================================
 *  地面接触检测
 *============================================================================*/
#define VMC_FN_THRESHOLD  3.0f   /* 支撑力阈值 (N) */

uint8_t VMC_GroundDetection(vmc_leg_t *vmc)
{
    vmc->contact = (vmc->FN > VMC_FN_THRESHOLD) ? 1 : 0;
    return vmc->contact;
}
