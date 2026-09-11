#include "control_task.h"

#include "motor_task.h"
#include "robot_app.h"
#include "system_timer_task.h"
#include "tim.h"
#if SIMPLE_IMU_BALANCE_ENABLE
#include "imu_task.h"
#endif

extern bool control_task_init_bl;

#if SIMPLE_IMU_BALANCE_ENABLE
static uint8_t imu_init_attempted = 0U;
static uint32_t imu_poll_last_tick = 0U;
volatile uint32_t imu_poll_count_watch = 0U;
#endif

static void Task3600s_Callback(void)
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

void control_task_init(void)
{
    /* 顺序不能颠倒：先启动电机底层，再初始化简单版上层状态机。 */
    SYS_Timestamp.Init(&htim5);
    motor_task_init();
    robot_app_init();

    /* TIM8 负责电机 CAN 发送；TIM7 每 1 ms 跑一次上层控制。 */
    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_Base_Start_IT(&htim7);
    control_task_init_bl = true;
}

void control_task(void)
{
#if SIMPLE_IMU_BALANCE_ENABLE
    /* 电机定时器已经在control_task_init()启动；IMU慢速SPI只能在main轮询。 */
    if (!imu_init_attempted) {
        imu_init_attempted = 1U;
        (void)IMU_Init();
        imu_poll_last_tick = HAL_GetTick();
    } else {
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - imu_poll_last_tick) >= 1U) {
            imu_poll_last_tick = now;
            IMU_Task_1ms();
            imu_poll_count_watch++;
        }
    }
#endif
#if ROBOT_CAN_DIAGNOSTIC_ENABLE
    /* TX 开启时走 main 主循环；纯 RX 模式下此函数为空。 */
    motor_can_diagnostic_poll();
#else
    /* 电机实时任务由定时器调度，main只轮询IMU。 */
#endif
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM7) {
        /* 手柄 -> 状态机 -> 足端 -> 逆解 -> 电机目标。 */
        robot_app_step_1ms();
    } else if (htim->Instance == TIM8) {
        /* 把上层已经写好的 8 个目标通过 CAN 发给电机。 */
        motor_task();
    } else if (htim->Instance == TIM5) {
        Task3600s_Callback();
    }
}
