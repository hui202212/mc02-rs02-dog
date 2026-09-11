#include "Curve.h"
#include "math_support.h"
#include <math.h>

static fp32 clamp01(fp32 value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

/* 旋轮线进度在 0 和 1 处速度均为 0，减小离地和落地冲击。 */
static fp32 cycloid_progress(fp32 u)
{
    u = clamp01(u);
    return u - sinf(2.0f * PI * u) / (2.0f * PI);
}

void connection_dog_leg_curve::curve_init(const fp32 &swing_ratio,
                                           const fp32 &unused_mid_point,
                                           const fp32 &unused_mid_point_pwm)
{
    (void)unused_mid_point;
    (void)unused_mid_point_pwm;
    /* 摆动相占比限制在 20%~50%，其余周期自动作为支撑相。 */
    PWM_gait = swing_ratio;
    if (PWM_gait < 0.20f) PWM_gait = 0.20f;
    if (PWM_gait > 0.50f) PWM_gait = 0.50f;
}

void connection_dog_leg_curve::curve(const fp32 &phase,
                                      const fp32 start[3],
                                      const fp32 end[3],
                                      const fp32 &height)
{
    fp32 p = phase;
    while (p >= 1.0f) p -= 1.0f;
    while (p < 0.0f) p += 1.0f;

    /* 摆动相：足端由 start 移动到 end，并用余弦抬脚曲线离地后落回。 */
    if (p < PWM_gait) {
        fp32 u = p / PWM_gait;
        fp32 s = cycloid_progress(u);
        fp32 lift = 0.5f * (1.0f - cosf(2.0f * PI * u));

        leg_out.x = start[0] + (end[0] - start[0]) * s;
        leg_out.y = start[1] + (end[1] - start[1]) * s;
        leg_out.z = start[2] + (end[2] - start[2]) * s + height * lift;
    } else {
        /* 支撑相：足端沿地面由 end 回到 start，形成相对机身的推进。 */
        fp32 u = (p - PWM_gait) / (1.0f - PWM_gait);
        fp32 s = cycloid_progress(u);

        leg_out.x = end[0] + (start[0] - end[0]) * s;
        leg_out.y = end[1] + (start[1] - end[1]) * s;
        leg_out.z = end[2] + (start[2] - end[2]) * s;
    }
}
