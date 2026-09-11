#ifndef CURVE_H
#define CURVE_H

#include "struct_typedef.h"

struct connection_leg_curve
{
    fp32 x;
    fp32 y;
    fp32 z;
};

class connection_dog_leg_curve
{
public:
    /*
     * phase 归一化到 [0,1)，start/end/height 单位均为 mm。
     * swing_ratio 是整周期中的摆动相占比；后两个参数仅为旧接口兼容占位。
     */
    void curve_init(const fp32 &swing_ratio,
                    const fp32 &unused_mid_point,
                    const fp32 &unused_mid_point_pwm);
    void curve(const fp32 &phase,
               const fp32 start[3],
               const fp32 end[3],
               const fp32 &height);

    inline fp32 Get_X() const { return leg_out.x; }
    inline fp32 Get_Y() const { return leg_out.y; }
    inline fp32 Get_Z() const { return leg_out.z; }
    inline fp32 Get_PWM_gait() const { return PWM_gait; }

private:
    fp32 PWM_gait = 0.4f;
    connection_leg_curve leg_out = {0.0f, 0.0f, 0.0f};
};

#endif
