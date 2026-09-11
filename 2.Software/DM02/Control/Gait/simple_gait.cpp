#include "simple_gait.h"

#include <math.h>

#define SIMPLE_PI 3.14159265358979323846f

/*
 * 步态模块只负责“画脚尖轨迹”，完全不接触电机：
 * 摆动相：脚离地，从后向前移动；支撑相：脚着地，从前向后移动。
 * 正常行走使用对角小跑；低姿匍匐使用每次只抬一条腿的四拍慢步。
 */

static fp32 clamp(fp32 value, fp32 low, fp32 high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static fp32 cycloid(fp32 u)
{
    /* 旋轮线进度在起点/终点速度为 0，可减小离地和落地冲击。 */
    u = clamp(u, 0.0f, 1.0f);
    return u - sinf(2.0f * SIMPLE_PI * u) / (2.0f * SIMPLE_PI);
}

static fp32 approach(fp32 current, fp32 target, fp32 max_delta)
{
    fp32 delta = target - current;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    return current + delta;
}

static SimpleFootTarget foot_curve(fp32 phase, fp32 swing_ratio,
                                   fp32 step, fp32 height,
                                   fp32 base_z_down)
{
    SimpleFootTarget foot;
    fp32 start_x = SIMPLE_STAND_X_MM - 0.5f * step;
    fp32 end_x = SIMPLE_STAND_X_MM + 0.5f * step;

    while (phase >= 1.0f) phase -= 1.0f;
    while (phase < 0.0f) phase += 1.0f;

    if (phase < swing_ratio) {
        /* 摆动相：x 从后走到前，z_down 变小表示脚抬高。 */
        fp32 u = phase / swing_ratio;
        fp32 progress = cycloid(u);
#if SIMPLE_ARTICLE_TRAJECTORY_ENABLE
        /* 改进复合摆线：起落脚竖直速度为 0，减少触地冲击。 */
        fp32 lift = 0.5f * (1.0f - cosf(2.0f * SIMPLE_PI * u));
#else
        /* 前 1/4 抬起、中间 1/2 保持最高、后 1/4 落下，避免脚尖擦地。 */
        fp32 lift;
        if (u < 0.25f) {
            lift = 0.5f * (1.0f - cosf(SIMPLE_PI * u / 0.25f));
        } else if (u <= 0.75f) {
            lift = 1.0f;
        } else {
            lift = 0.5f * (1.0f + cosf(SIMPLE_PI * (u - 0.75f) / 0.25f));
        }
#endif
        foot.x = start_x + (end_x - start_x) * progress;
        foot.z_down = base_z_down - height * lift;
    } else {
        /* 支撑相：脚保持地面高度，x 从前回到后，推动机身。 */
        fp32 u = (phase - swing_ratio) / (1.0f - swing_ratio);
#if SIMPLE_ARTICLE_TRAJECTORY_ENABLE
        /* 匀速后移对应近似匀速的机身前进，减少支撑期反复加减速。 */
        fp32 progress = u;
#else
        fp32 progress = cycloid(u);
#endif
        foot.x = end_x + (start_x - end_x) * progress;
        foot.z_down = base_z_down;
    }
    return foot;
}

void simple_gait_reset(SimpleGaitState *state)
{
    state->phase = 0.0f;
    state->ramp = 0.0f;
    state->forward_command = 0.0f;
    state->turn_command = 0.0f;
}

void simple_gait_step(SimpleGaitState *state, fp32 dt,
                      fp32 forward_command, fp32 turn_command,
                      SimpleGaitPattern pattern,
                      fp32 base_z_down, fp32 walk_step_max,
                      fp32 turn_step_max, fp32 step_height,
                      SimpleFootTarget foot[SIMPLE_LEG_COUNT])
{
    forward_command = clamp(forward_command, -1.0f, 1.0f);
    turn_command = clamp(turn_command, -1.0f, 1.0f);
    /* 先限制摇杆变化率，再用平滑后的指令计算周期和步幅。 */
    if (SIMPLE_COMMAND_SLEW_S > 0.0f) {
        fp32 max_delta = dt / SIMPLE_COMMAND_SLEW_S;
        state->forward_command = approach(state->forward_command,
                                          forward_command, max_delta);
        state->turn_command = approach(state->turn_command,
                                       turn_command, max_delta);
    } else {
        state->forward_command = forward_command;
        state->turn_command = turn_command;
    }
    forward_command = state->forward_command;
    turn_command = state->turn_command;

    /* 摇杆越大周期越短；ramp 让刚进入步态时不会瞬间给满步长。 */
    fp32 magnitude = fmaxf(fabsf(forward_command), fabsf(turn_command));
    fp32 swing_ratio;
    fp32 period_min;
    fp32 period_max;
    if (pattern == SIMPLE_GAIT_CRAWL) {
        swing_ratio = SIMPLE_CRAWL_SWING_RATIO;
        period_min = SIMPLE_CRAWL_PERIOD_MIN_S;
        period_max = SIMPLE_CRAWL_PERIOD_MAX_S;
    } else if (pattern == SIMPLE_GAIT_BRIDGE_A) {
        swing_ratio = SIMPLE_BRIDGE_A_SWING_RATIO;
        period_min = SIMPLE_BRIDGE_A_PERIOD_MIN_S;
        period_max = SIMPLE_BRIDGE_A_PERIOD_MAX_S;
    } else if (pattern == SIMPLE_GAIT_BRIDGE_B) {
        swing_ratio = SIMPLE_BRIDGE_B_SWING_RATIO;
        period_min = SIMPLE_BRIDGE_B_PERIOD_MIN_S;
        period_max = SIMPLE_BRIDGE_B_PERIOD_MAX_S;
    } else if (pattern == SIMPLE_GAIT_FAST_TRANSIT) {
        /* L1转场使用独立周期，不连带加速木桥B和限高模式。 */
        swing_ratio = SIMPLE_TRANSIT_SWING_RATIO;
        period_min = SIMPLE_TRANSIT_PERIOD_MIN_S;
        period_max = SIMPLE_TRANSIT_PERIOD_MAX_S;
    } else if (pattern == SIMPLE_GAIT_FAST_CRAWL) {
        swing_ratio = SIMPLE_FAST_CRAWL_SWING_RATIO;
        period_min = SIMPLE_FAST_CRAWL_PERIOD_MIN_S;
        period_max = SIMPLE_FAST_CRAWL_PERIOD_MAX_S;
    } else {
        swing_ratio = SIMPLE_WALK_SWING_RATIO;
        period_min = SIMPLE_WALK_PERIOD_MIN_S;
        period_max = SIMPLE_WALK_PERIOD_MAX_S;
    }
    fp32 period = period_max - (period_max - period_min) * magnitude;

    state->ramp = clamp(state->ramp + dt / SIMPLE_GAIT_RAMP_S, 0.0f, 1.0f);
    state->phase += dt / period;
    while (state->phase >= 1.0f) state->phase -= 1.0f;

    for (uint8_t leg = 0U; leg < SIMPLE_LEG_COUNT; ++leg) {
        /* 前进时四腿同方向；转向时左右两侧步长反号。 */
        fp32 side = (leg == SIMPLE_RF || leg == SIMPLE_RH) ? 1.0f : -1.0f;
        /* 先由摇杆得到目标速度，再用 step = speed * period 得到步幅。
         * 满摇杆、最快周期时恰好等于配置的最大步幅。 */
        fp32 period_scale = period / period_min;
        fp32 step = forward_command * walk_step_max * period_scale;
        step += turn_command * side * turn_step_max * period_scale;
        step *= state->ramp;

        static const fp32 trot_offset[SIMPLE_LEG_COUNT] = {
            0.00f,  /* RF */
            0.50f,  /* LF */
            0.50f,  /* RH */
            0.00f   /* LH */
        };
        static const fp32 crawl_offset[SIMPLE_LEG_COUNT] = {
            0.00f,  /* RF */
            0.50f,  /* LF */
            0.25f,  /* RH */
            0.75f   /* LH */
        };
        static const fp32 lift_extra[SIMPLE_LEG_COUNT] = {
            SIMPLE_RF_LIFT_EXTRA_MM, SIMPLE_LF_LIFT_EXTRA_MM,
            SIMPLE_RH_LIFT_EXTRA_MM, SIMPLE_LH_LIFT_EXTRA_MM,
        };
        uint8_t four_beat = pattern == SIMPLE_GAIT_CRAWL ||
                            pattern == SIMPLE_GAIT_BRIDGE_A ||
                            pattern == SIMPLE_GAIT_BRIDGE_B;
        fp32 phase_offset = four_beat ? crawl_offset[leg] : trot_offset[leg];
        fp32 leg_step_height = step_height;
        if (pattern == SIMPLE_GAIT_TROT) {
            leg_step_height += lift_extra[leg];
        }
        /* 水平步幅需要渐入；抬脚曲线自身从零速度起步，不再乘慢速 ramp，
         * 避免每次进入步态时第一组对角腿因高度不足而擦地。 */
        foot[leg] = foot_curve(state->phase + phase_offset, swing_ratio, step,
                               leg_step_height, base_z_down);
    }
}
