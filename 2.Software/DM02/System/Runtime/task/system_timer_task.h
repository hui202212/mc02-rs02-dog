#ifndef SYSTEM_TIMER_TASK_H
#define SYSTEM_TIMER_TASK_H

#include "struct_typedef.h"

#include "tim.h"
#include "stm32h7xx_hal.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Specialized, 系统时间戳
 *
 */
class Class_Timestamp
{
public:
    void Init(TIM_HandleTypeDef *htim);

    inline uint64_t Get_Current_Timestamp() const;

    inline float Get_Now_Second() const;

    inline float Get_Now_Millisecond() const;

    inline uint64_t Get_Now_Microsecond() const;

    void TIM_3600s_PeriodElapsedCallback();

protected:
    // 初始化相关常量

    TIM_HandleTypeDef *TIM_Handler;

    // 常量

    // 内部变量

    // 定时器溢出计数, 一小时溢出一次
    uint32_t TIM_Overflow_Count = 0;

    // 写变量

    // 读写变量

    // 内部函数

    uint64_t Calculate_Timestamp() const;
};

extern Class_Timestamp SYS_Timestamp;
extern bool control_task_init_bl;

namespace Namespace_SYS_Timestamp
{
    void Delay_Second(const uint32_t &Second);

    void Delay_Millisecond(const uint32_t &Millisecond);

    void Delay_Microsecond(const uint32_t &Microsecond);
};


/**
 * @brief 获取当前时间
 *
 * @return uint64_t 当前时间
 */
inline uint64_t Class_Timestamp::Get_Current_Timestamp() const
{
    return (Calculate_Timestamp());
}

/**
 * @brief 获取当前时间, 单位秒
 *
 * @return float 当前时间, 单位秒
 */
inline float Class_Timestamp::Get_Now_Second() const
{
    return ((float) (Calculate_Timestamp()) / 1000000.0f);
}

/**
 * @brief 获取当前时间, 单位毫秒
 *
 * @return float 当前时间, 单位毫秒
 */
inline float Class_Timestamp::Get_Now_Millisecond() const
{
    return ((float) (Calculate_Timestamp()) / 1000.0f);
}

/**
 * @brief 获取当前时间, 单位微秒
 *
 * @return uint64_t 当前时间, 单位微秒
 */
inline uint64_t Class_Timestamp::Get_Now_Microsecond() const
{
    return (Calculate_Timestamp());
}


#endif