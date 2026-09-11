#include "ik.h"
#include "math_support.h"
#include "math.h"

/* 每行依次对应 RF/LF/RH/LH，长度单位均为 mm。 */
static const fp32 robot8_parallel_motor_half_dis[4] = {
    57.5f, 57.5f, 57.5f, 57.5f,
};

static const fp32 robot8_parallel_active_len[4] = {
    110.0f, 110.0f, 110.0f, 110.0f,
};

static const fp32 robot8_parallel_passive_len[4] = {
    200.0f, 200.0f, 200.0f, 200.0f,
};

/*
 * 电机反馈为 0 时对应的主动杆几何角，单位 rad。
 * 行顺序为 RF/LF/RH/LH，列顺序固定为 {左主动杆, 右主动杆}，不是 CAN ID 顺序。
 * 八个值必须在实机装配后标定；未完成零偏/方向标定前不得调站姿和步态。
 */
static const fp32 robot8_parallel_motor_zero[4][2] = {
    {0.0f, 0.0f},   /* RF */
    {0.0f, 0.0f},   /* LF */
    {0.0f, 0.0f},   /* RH */
    {0.0f, 0.0f},   /* LH */
};

//右前 左前 右后 左后.
static const fp32 robot8_parallel_left_sign[4] = {
    1.0f, -1.0f, 1.0f, -1.0f,
};

static const fp32 robot8_parallel_right_sign[4] = {
    1.0f, -1.0f, 1.0f, -1.0f,
};

/* 工作空间裕量和最小向下距离单位 mm；单周期电机连续性阈值单位 rad。 */
static const fp32 robot8_parallel_workspace_margin = 10.0f;
static const fp32 robot8_parallel_min_z_down = 20.0f;
static const fp32 robot8_parallel_max_motor_step = 0.35f;

/* 兼容旧接口：五连杆只读取每张表中的机身锚点 b、腿平面 W 和高度 h。 */
const fp32 DH1[5][4]={
       PI/2,      0,      233.25,         0,
       PI/2,  -PI/2,      -70,           0,
       0,     -PI/2,     53.5,          14,
       0,         0,      190,        82.5,
       0,         0,      248.54f,       0,
};

const fp32 DH2[5][4]={
       PI/2,      0,      233.25,         0,
       PI/2,  -PI/2,       70,           0,
       0,     -PI/2,     53.5,          14,
       0,         0,      190,        82.5,
       0,         0,      248.54f,       0,
};

const fp32 DH3[5][4]={
       PI/2,      0,     -233.25,         0,
       PI/2,  -PI/2,      -70,           0,
       0,     -PI/2,     53.5,          14,
       0,         0,      190,        82.5,
       0,         0,      248.54f,       0,
};

const fp32 DH4[5][4]={
       PI/2,      0,     -233.25,         0,
       PI/2,  -PI/2,       70,           0,
       0,     -PI/2,     53.5,          14,
       0,         0,      190,        82.5,
       0,         0,      248.54f,       0,
};

static fp32 clamp_unit(fp32 v)
{
    if (v > 1.0f) return 1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

/*
 * 几何角到电机命令角：motor = sign * (q_geometric - q_zero)。
 * 先按左/右主动杆换算，再重排为该腿实际 CAN 对内的 pos1/pos2 顺序。
 */
static void parallel_geometric_to_motor(uint8_t leg, uint8_t idx,
                                        fp32 q_left, fp32 q_right,
                                        fp32 *motor_out1, fp32 *motor_out2)
{
    fp32 motor_left = robot8_parallel_left_sign[idx] *
                      (q_left - robot8_parallel_motor_zero[idx][0]);
    fp32 motor_right = robot8_parallel_right_sign[idx] *
                       (q_right - robot8_parallel_motor_zero[idx][1]);

    /* CAN 对内顺序：RF/RH 为右、左；LF/LH 为左、右。 */
    if (leg == 1U || leg == 3U) {
        *motor_out1 = motor_right;
        *motor_out2 = motor_left;
    } else {
        *motor_out1 = motor_left;
        *motor_out2 = motor_right;
    }
}

/* 电机反馈到几何角，严格使用上面同一份顺序、方向和零偏的逆映射。 */
static void parallel_motor_to_geometric(uint8_t leg, uint8_t idx,
                                        fp32 motor_out1, fp32 motor_out2,
                                        fp32 *q_left, fp32 *q_right)
{
    fp32 motor_left;
    fp32 motor_right;

    if (leg == 1U || leg == 3U) {
        motor_right = motor_out1;
        motor_left = motor_out2;
    } else {
        motor_left = motor_out1;
        motor_right = motor_out2;
    }

    *q_left = motor_left / robot8_parallel_left_sign[idx] +
              robot8_parallel_motor_zero[idx][0];
    *q_right = motor_right / robot8_parallel_right_sign[idx] +
               robot8_parallel_motor_zero[idx][1];
}

void kinematics_connection_12bot_robot::Init(const uint8_t &__leg,const fp32 __DH[5][4])
{
    b = __DH[0][2];
    W = __DH[1][2];
    h = __DH[0][3];
    L1 = robot8_parallel_active_len[__leg - 1];
    L2 = robot8_parallel_passive_len[__leg - 1];
    L3 = 0.0f;
    h1 = 0.0f;
    h2 = 0.0f;
    leg = __leg;

    control.set_x = 0.0f;
    control.set_y = 0.0f;
    control.set_z = 0.0f;
    control.x = 0.0f;
    control.y = 0.0f;
    control.z = 0.0f;
    control.motor_out_pos1 = 0.0f;
    control.motor_out_pos2 = 0.0f;
    control.motor_out_pos3 = 0.0f;
    control.out_pos1 = 0.0f;
    control.out_pos2 = 0.0f;
    control.out_pos3 = 0.0f;
    control.out_Angle1 = 0.0f;
    control.out_Angle2 = 0.0f;
    control.out_Angle3 = 0.0f;
    last_target_valid = false;
    motor_reference_valid = false;
    branch_locked = false;
    locked_left_branch = 0U;
    locked_right_branch = 0U;
}

void kinematics_connection_12bot_robot::Set_Motor_Reference(const fp32 &__pos1,
                                                            const fp32 &__pos2)
{
    if (!isfinite(__pos1) || !isfinite(__pos2)) {
        motor_reference_valid = false;
        last_target_valid = false;
        return;
    }

    control.motor_out_pos1 = __pos1;
    control.motor_out_pos2 = __pos2;
    control.motor_out_pos3 = 0.0f;
    motor_reference_valid = true;
    branch_locked = false;
    last_target_valid = false;
}

bool kinematics_connection_12bot_robot::leg_inverse_calculation(const fp32 &__x,const fp32 &__y,const fp32 &__z)
{
    uint8_t idx = leg - 1;
    fp32 d = robot8_parallel_motor_half_dis[idx];
    fp32 l1 = robot8_parallel_active_len[idx];
    fp32 l2 = robot8_parallel_passive_len[idx];

    if (!motor_reference_valid) {
        last_target_valid = false;
        return false;
    }

    if (!isfinite(__x) || !isfinite(__y) || !isfinite(__z)) {
        last_target_valid = false;
        return false;
    }

    /* 从机身坐标转为腿平面局部坐标；内部 z_down 以向下为正。 */
    fp32 foot_x = __x - b;
    fp32 foot_z_down = h - __z;
    if (foot_z_down < robot8_parallel_min_z_down) {
        last_target_valid = false;
        return false;
    }

    fp32 dx_left = foot_x + d;
    fp32 dx_right = foot_x - d;
    fp32 r_left;
    fp32 r_right;
    arm_sqrt_f32(dx_left * dx_left + foot_z_down * foot_z_down, &r_left);
    arm_sqrt_f32(dx_right * dx_right + foot_z_down * foot_z_down, &r_right);

    /* 两侧主动链都必须落在同一目标点对应的圆环工作区内，不做静默夹边。 */
    fp32 min_r = fabsf(l2 - l1) + robot8_parallel_workspace_margin;
    fp32 max_r = l1 + l2 - robot8_parallel_workspace_margin;
    if (!isfinite(r_left) || !isfinite(r_right) ||
        r_left < min_r || r_left > max_r ||
        r_right < min_r || r_right > max_r) {
        last_target_valid = false;
        return false;
    }

    fp32 phi_left = atan2f(foot_z_down, dx_left);
    fp32 phi_right = atan2f(foot_z_down, dx_right);
    fp32 cosine_left = (l1*l1 + r_left*r_left - l2*l2) / (2.0f*l1*r_left);
    fp32 cosine_right = (l1*l1 + r_right*r_right - l2*l2) / (2.0f*l1*r_right);
    if (!isfinite(cosine_left) || !isfinite(cosine_right) ||
        cosine_left < -1.0001f || cosine_left > 1.0001f ||
        cosine_right < -1.0001f || cosine_right > 1.0001f) {
        last_target_valid = false;
        return false;
    }

    fp32 alpha_left = acosf(clamp_unit(cosine_left));
    fp32 alpha_right = acosf(clamp_unit(cosine_right));

    /* 左、右主动杆各有两个肘形候选，组合后共四条装配支链。 */
    fp32 q_left_candidate[2] = {
        phi_left - alpha_left,
        phi_left + alpha_left,
    };
    fp32 q_right_candidate[2] = {
        phi_right - alpha_right,
        phi_right + alpha_right,
    };

    fp32 q_left = 0.0f;
    fp32 q_right = 0.0f;
    fp32 motor_out1 = 0.0f;
    fp32 motor_out2 = 0.0f;
    fp32 best_score = 1.0e30f;
    uint8_t selected_left_branch = 0U;
    uint8_t selected_right_branch = 0U;
    bool solution_found = false;

    for (uint8_t li = 0; li < 2; li++) {
        for (uint8_t ri = 0; ri < 2; ri++) {
            if (branch_locked &&
                (li != locked_left_branch || ri != locked_right_branch)) {
                continue;
            }

            fp32 mo1;
            fp32 mo2;
            parallel_geometric_to_motor(leg, idx,
                                        q_left_candidate[li], q_right_candidate[ri],
                                        &mo1, &mo2);

            fp32 delta1 = fabsf(mo1 - control.motor_out_pos1);
            fp32 delta2 = fabsf(mo2 - control.motor_out_pos2);
            /*
             * 未锁定时按真实电机参考选择最近的实机支链，供上层平滑过渡使用；
             * 锁定后只允许同一支链，并拒绝单周期电机角跳变过大的目标。
             */
            if (branch_locked &&
                (delta1 > robot8_parallel_max_motor_step ||
                 delta2 > robot8_parallel_max_motor_step)) {
                continue;
            }

            fp32 score = delta1 + delta2;
            if (score < best_score) {
                best_score = score;
                q_left = q_left_candidate[li];
                q_right = q_right_candidate[ri];
                motor_out1 = mo1;
                motor_out2 = mo2;
                selected_left_branch = li;
                selected_right_branch = ri;
                solution_found = true;
            }
        }
    }

    /* 此前未写 control；失败时输出仍是上一条有效命令。 */
    if (!solution_found) {
        last_target_valid = false;
        return false;
    }

    control.set_x = __x;
    control.set_y = __y;
    control.set_z = __z;
    control.out_pos1 = q_left;
    control.out_pos2 = q_right;
    control.out_pos3 = 0.0f;
    control.motor_out_pos1 = motor_out1;
    control.motor_out_pos2 = motor_out2;
    control.motor_out_pos3 = 0.0f;

    control.out_Angle1 = control.out_pos1 / MATH_RPM_TO_RADPS;
    control.out_Angle2 = control.out_pos2 / MATH_RPM_TO_RADPS;
    control.out_Angle3 = 0.0f;

    control.x = __x;
    control.y = __y;
    control.z = __z;
    /* 成功后以本次命令作为连续性参考，并锁住所选左右肘形。 */
    motor_reference_valid = true;
    branch_locked = true;
    locked_left_branch = selected_left_branch;
    locked_right_branch = selected_right_branch;
    last_target_valid = true;
    return true;
}

void kinematics_connection_12bot_robot::leg_correct_solution(const fp32 &__pos1,const fp32 &__pos2,const fp32 &__pos3)
{
    (void)__pos3;

    uint8_t idx = leg - 1;
    fp32 d = robot8_parallel_motor_half_dis[idx];
    fp32 l1 = robot8_parallel_active_len[idx];
    fp32 l2 = robot8_parallel_passive_len[idx];

    if (!isfinite(__pos1) || !isfinite(__pos2)) {
        last_target_valid = false;
        return;
    }

    Set_Motor_Reference(__pos1, __pos2);

    fp32 q_left;
    fp32 q_right;
    parallel_motor_to_geometric(leg, idx, __pos1, __pos2,
                                &q_left, &q_right);

    fp32 ax = -d;
    fp32 az = 0.0f;
    fp32 bx = d;
    fp32 bz = 0.0f;
    fp32 cx = ax + l1 * cosf(q_left);
    fp32 cz = az + l1 * sinf(q_left);
    fp32 dx = bx + l1 * cosf(q_right);
    fp32 dz = bz + l1 * sinf(q_right);

    fp32 vx = dx - cx;
    fp32 vz = dz - cz;
    fp32 dist;
    arm_sqrt_f32(vx * vx + vz * vz, &dist);
    if (!isfinite(dist) || dist < 1.0e-4f || dist > 2.0f * l2) {
        last_target_valid = false;
        return;
    }

    /* 两根等长被动杆的圆交点；最终选取位于电机轴下方的足端装配解。 */
    fp32 half = dist * 0.5f;
    fp32 h_intersect_sq = l2*l2 - half*half;
    if (h_intersect_sq < -1.0e-3f) {
        last_target_valid = false;
        return;
    }
    if (h_intersect_sq < 0.0f) h_intersect_sq = 0.0f;
    fp32 h_intersect;
    arm_sqrt_f32(h_intersect_sq, &h_intersect);

    fp32 mx = (cx + dx) * 0.5f;
    fp32 mz = (cz + dz) * 0.5f;
    fp32 nx = -vz / dist;
    fp32 nz = vx / dist;
    fp32 foot_x = mx + nx * h_intersect;
    fp32 foot_z_down = mz + nz * h_intersect;
    if (foot_z_down < 0.0f) {
        foot_x = mx - nx * h_intersect;
        foot_z_down = mz - nz * h_intersect;
    }

    control.out_pos1 = q_left;
    control.out_pos2 = q_right;
    control.out_pos3 = 0.0f;
    control.motor_out_pos1 = __pos1;
    control.motor_out_pos2 = __pos2;
    control.motor_out_pos3 = 0.0f;
    control.out_Angle1 = q_left / MATH_RPM_TO_RADPS;
    control.out_Angle2 = q_right / MATH_RPM_TO_RADPS;
    control.out_Angle3 = 0.0f;
    control.x = b + foot_x;
    control.y = W;
    control.z = h - foot_z_down;
    last_target_valid = true;
}
