#include "Dynamic.h"
#include "math_support.h"

const fp32 leg_plus_minus_D[4][3]={
     -1,     -1,      -1.0f/2.0f,
     -1,      1,     1.0f/2.0f,
    1,     -1,      -1.0f/2.0f,
     -1,      1,     -1.0f/2.0f,
};


const fp32 leg_Deviation[4][3]={
    PI/2,   0,  (90.0f/180.0f)*PI,
    PI/2,   0,  (90.0f/180.0f)*PI,
    PI/2,   0,  (90.0f/180.0f)*PI,
    PI/2,   0,  (90.0f/180.0f)*PI,
};

const int8_t leg_plus_minus[4][3]={
     -1,     -1,      2,
     1,      1,     -2,
    1,     -1,      2,
     -1,      1,     -2,
};

void Dynamic_mechanical_12bot_leg_robot::
   Dynamic_T_FeedForward_count(const uint8_t &__leg, const fp32 *__pos, const fp32 *__W){
    /* 用局部变量，不修改调用方数组 */
    fp32 pos[3];
    pos[0] = (__pos[0]/leg_plus_minus[__leg-1][0] + leg_Deviation[__leg-1][0]);
    pos[1] = __pos[1]/leg_plus_minus[__leg-1][1] + leg_Deviation[__leg-1][1];
    pos[2] = __pos[2]/leg_plus_minus[__leg-1][2] + leg_Deviation[__leg-1][2];

   T_ff[0] =0.0014 * __W[0]
       - 0.5477 *  arm_cos_f32( pos[0])
       - 3.1356 * arm_sin_f32( pos[0])
       - 1.1135 * arm_cos_f32( pos[1]) * arm_sin_f32( pos[0])
       + 0.0344 * arm_sin_f32( pos[0]) * arm_sin_f32(pos[1]) * arm_sin_f32( pos[2])
       - 0.0344 * arm_cos_f32(pos[1]) * arm_cos_f32( pos[2]) * arm_sin_f32( pos[0])
       + 0.6501 * arm_cos_f32(pos[1]) * arm_sin_f32( pos[0]) * arm_sin_f32( pos[2])
       + 0.6501 * arm_cos_f32( pos[2]) * arm_sin_f32( pos[0]) * arm_sin_f32(pos[1]);

   T_ff[1] = 0.0484 *  __W[1]
       - 1.1135 * arm_cos_f32(pos[0]) * arm_sin_f32(pos[1])
       - 0.6501 * arm_cos_f32(pos[0]) * arm_cos_f32(pos[1]) * arm_cos_f32(pos[2])
       - 0.0344 * arm_cos_f32(pos[0]) * arm_cos_f32(pos[1]) * arm_sin_f32(pos[2])
       - 0.0344 * arm_cos_f32(pos[0]) * arm_cos_f32(pos[2]) * arm_sin_f32(pos[1])
       + 0.6501 * arm_cos_f32(pos[0]) * arm_sin_f32(pos[1]) * arm_sin_f32(pos[2]);

   T_ff[2] = 0.0602 * __W[2]
       - 0.6501 * arm_cos_f32(pos[0]) * arm_cos_f32(pos[1]) * arm_cos_f32(pos[2])
       - 0.0344 * arm_cos_f32(pos[0]) * arm_cos_f32(pos[1]) * arm_sin_f32(pos[2])
       - 0.0344 * arm_cos_f32(pos[0]) * arm_cos_f32(pos[2]) * arm_sin_f32(pos[1])
       + 0.6501 * arm_cos_f32(pos[0]) * arm_sin_f32(pos[1]) * arm_sin_f32(pos[2]);


   T_ff[0]*=1.2f*T_ff_count_ration*leg_plus_minus_D[__leg-1][0];
   T_ff[1]*=1.2f*T_ff_count_ration*leg_plus_minus_D[__leg-1][1];
   T_ff[2]*=1.2f*T_ff_count_ration*leg_plus_minus_D[__leg-1][2];

}
