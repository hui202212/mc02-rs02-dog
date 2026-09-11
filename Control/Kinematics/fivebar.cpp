#include "fivebar.h"

#include <math.h>

static const fp32 HALF_PI = 1.5707963267948966f;

/*
 * 同轴五连杆侧视图：两个电机面对面安装，输出轴在侧视投影中是同一点 O。
 * 两根主动杆分别经过各自的从动杆连接到同一个足端 P。
 *
 * 参考旧工程 CartesianToTheta()：
 *   L = sqrt(x^2 + z^2)
 *   N = asin(x / L)
 *   M = acos((L^2 + L1^2 - L2^2) / (2 L1 L))
 *   A1 = M - N, A2 = M + N
 *   theta = A - 90 deg
 *
 * 本文件统一使用 mm 和 rad，+X 向前，+Z_DOWN 向下。
 */

/* 电机反馈为 0 时对应的 theta1/theta2 软件角，每行 {theta1, theta2}。
 * theta1 对应该腿的“左”主动杆，theta2 对应“右”主动杆。 */
static const fp32 motor_zero[SIMPLE_LEG_COUNT][2] = {
    {SIMPLE_ZERO_RF_LEFT_RAD, SIMPLE_ZERO_RF_RIGHT_RAD},
    {SIMPLE_ZERO_LF_LEFT_RAD, SIMPLE_ZERO_LF_RIGHT_RAD},
    {SIMPLE_ZERO_RH_LEFT_RAD, SIMPLE_ZERO_RH_RIGHT_RAD},
    {SIMPLE_ZERO_LH_LEFT_RAD, SIMPLE_ZERO_LH_RIGHT_RAD},
};

/* motor = sign * (theta - zero)。面对面电机从外侧看旋向相反，
 * 但同一条腿的两个编码器在旧工程映射中使用相同数学符号。 */
static const fp32 motor_sign[SIMPLE_LEG_COUNT][2] = {
    {SIMPLE_SIGN_RF_LEFT, SIMPLE_SIGN_RF_RIGHT},
    {SIMPLE_SIGN_LF_LEFT, SIMPLE_SIGN_LF_RIGHT},
    {SIMPLE_SIGN_RH_LEFT, SIMPLE_SIGN_RH_RIGHT},
    {SIMPLE_SIGN_LH_LEFT, SIMPLE_SIGN_LH_RIGHT},
};

/* CAN 对内顺序：RF/RH 是 {theta2, theta1}，LF/LH 是 {theta1, theta2}。 */
static const uint8_t theta2_first[SIMPLE_LEG_COUNT] = {
    SIMPLE_RF_RIGHT_MOTOR_FIRST, SIMPLE_LF_RIGHT_MOTOR_FIRST,
    SIMPLE_RH_RIGHT_MOTOR_FIRST, SIMPLE_LH_RIGHT_MOTOR_FIRST,
};

static fp32 clamp_unit(fp32 value)
{
    if (value < -1.0f) return -1.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

static uint8_t is_right_side_leg(uint8_t leg)
{
    return leg == SIMPLE_RF || leg == SIMPLE_RH;
}

static void theta_to_motors(uint8_t leg, fp32 theta1, fp32 theta2,
                            fp32 motor[2])
{
    fp32 value1 = motor_sign[leg][0] * (theta1 - motor_zero[leg][0]);
    fp32 value2 = motor_sign[leg][1] * (theta2 - motor_zero[leg][1]);

    if (theta2_first[leg]) {
        motor[0] = value2;
        motor[1] = value1;
    } else {
        motor[0] = value1;
        motor[1] = value2;
    }
}

static void motors_to_theta(uint8_t leg, fp32 motor1, fp32 motor2,
                            fp32 *theta1, fp32 *theta2)
{
    fp32 value1 = theta2_first[leg] ? motor2 : motor1;
    fp32 value2 = theta2_first[leg] ? motor1 : motor2;
    *theta1 = value1 / motor_sign[leg][0] + motor_zero[leg][0];
    *theta2 = value2 / motor_sign[leg][1] + motor_zero[leg][1];
}

void fivebar_set_reference(FiveBarState *state, fp32 motor1, fp32 motor2)
{
    if (state == 0) return;
    state->reference_motor[0] = motor1;
    state->reference_motor[1] = motor2;
    state->reference_valid = 0U;
}

uint8_t fivebar_inverse(FiveBarState *state, uint8_t leg,
                        fp32 foot_x, fp32 foot_z_down,
                        fp32 motor_out[2])
{
    const fp32 active = SIMPLE_ACTIVE_LENGTH_MM;
    const fp32 passive = SIMPLE_PASSIVE_LENGTH_MM;
    const fp32 min_radius = fabsf(passive - active) + SIMPLE_WORKSPACE_MARGIN_MM;
    const fp32 max_radius = passive + active - SIMPLE_WORKSPACE_MARGIN_MM;

    if (state == 0 || motor_out == 0 || leg >= SIMPLE_LEG_COUNT ||
        !isfinite(foot_x) || !isfinite(foot_z_down) ||
        foot_z_down < SIMPLE_MIN_Z_DOWN_MM) {
        return 0U;
    }

    fp32 radius = sqrtf(foot_x * foot_x + foot_z_down * foot_z_down);
    if (!isfinite(radius) || radius < min_radius || radius > max_radius) {
        return 0U;
    }

    fp32 cosine = (radius * radius + active * active - passive * passive)
                / (2.0f * active * radius);
    if (!isfinite(cosine) || cosine < -1.0001f || cosine > 1.0001f) {
        return 0U;
    }

    fp32 n = SIMPLE_COAXIAL_X_SIGN * asinf(clamp_unit(foot_x / radius));
    fp32 m = acosf(clamp_unit(cosine));
    fp32 a1 = m - n;
    fp32 a2 = m + n;
    /* 参考算法角度先乘实机约定符号；零位夹具处 A1=A2=90deg，
     * 因此修改该符号不会改变已经采集的零位。 */
    fp32 low = SIMPLE_COAXIAL_THETA_SIGN * (a1 - HALF_PI);
    fp32 high = SIMPLE_COAXIAL_THETA_SIGN * (a2 - HALF_PI);

    /* 原算法的 leg_direction：右侧腿 theta1=A2、theta2=A1；
     * 左侧腿交换。随后再按每条腿的实际 CAN 顺序排列。 */
    fp32 theta1 = is_right_side_leg(leg) ? high : low;
    fp32 theta2 = is_right_side_leg(leg) ? low : high;
    fp32 candidate[2];
    theta_to_motors(leg, theta1, theta2, candidate);

    if (state->reference_valid &&
        (fabsf(candidate[0] - state->reference_motor[0]) > SIMPLE_MAX_IK_STEP_RAD ||
         fabsf(candidate[1] - state->reference_motor[1]) > SIMPLE_MAX_IK_STEP_RAD)) {
        return 0U;
    }

    motor_out[0] = candidate[0];
    motor_out[1] = candidate[1];
    state->reference_motor[0] = candidate[0];
    state->reference_motor[1] = candidate[1];
    state->reference_valid = 1U;
    return 1U;
}

uint8_t fivebar_motor_pose_valid(uint8_t leg, fp32 motor1, fp32 motor2)
{
    if (leg >= SIMPLE_LEG_COUNT || !isfinite(motor1) || !isfinite(motor2)) {
        return 0U;
    }

    fp32 theta1;
    fp32 theta2;
    motors_to_theta(leg, motor1, motor2, &theta1, &theta2);

    /* 从 theta1/theta2 还原旧算法中的 M、N，再检查是否存在合法足端。
     * 右腿 high=theta1、low=theta2；左腿相反。 */
    fp32 low = is_right_side_leg(leg) ? theta2 : theta1;
    fp32 high = is_right_side_leg(leg) ? theta1 : theta2;
    /* 先还原为参考旧算法的 theta，再恢复 M、N。 */
    low /= SIMPLE_COAXIAL_THETA_SIGN;
    high /= SIMPLE_COAXIAL_THETA_SIGN;
    fp32 m = 0.5f * (low + high) + HALF_PI;
    fp32 n = 0.5f * (high - low) / SIMPLE_COAXIAL_X_SIGN;
    fp32 active = SIMPLE_ACTIVE_LENGTH_MM;
    fp32 passive = SIMPLE_PASSIVE_LENGTH_MM;
    fp32 discriminant = passive * passive
                      - active * active * sinf(m) * sinf(m);
    if (!isfinite(discriminant) || discriminant < 0.0f || cosf(n) <= 0.0f) {
        return 0U;
    }

    fp32 radius = active * cosf(m) + sqrtf(discriminant);
    fp32 z_down = radius * cosf(n);
    fp32 min_radius = fabsf(passive - active) + SIMPLE_WORKSPACE_MARGIN_MM;
    fp32 max_radius = passive + active - SIMPLE_WORKSPACE_MARGIN_MM;
    return isfinite(radius) && isfinite(z_down) &&
           radius >= min_radius && radius <= max_radius &&
           z_down >= SIMPLE_MIN_Z_DOWN_MM;
}
