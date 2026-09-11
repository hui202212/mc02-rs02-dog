#ifndef DYNAMIC_H
#define DYNAMIC_H

#include "struct_typedef.h"


class Dynamic_mechanical_12bot_leg_robot{

    public:

    void Dynamic_T_FeedForward_count(const uint8_t &__leg, const fp32 *__pos, const fp32 *__W);

    inline fp32 Get_T_ff(const uint8_t &__bot)const;

    protected:

    
    fp32 T_ff[3];
    const fp32 T_ff_count_ration =1.0f;

};

inline fp32 Dynamic_mechanical_12bot_leg_robot::
    Get_T_ff(const uint8_t &__bot)const{

    return T_ff[__bot-1];
}

#endif
