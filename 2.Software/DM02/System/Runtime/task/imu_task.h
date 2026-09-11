/**
 * @file imu_task.h
 * @brief BMI088姿态数据和主循环轮询接口
 */
#ifndef __IMU_TASK_H
#define __IMU_TASK_H

#include "bmi088_driver.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float q[4];
    float Gyro[3];
    float Accel[3];
    float Roll;
    float Pitch;
    float Yaw;
    uint8_t imu_flag;
} IMU_t;

extern IMU_t IMU;

/* Keil Watch：init_status=0且ready=1后，姿态才参与上坡补偿。 */
extern volatile uint8_t imu_init_status_watch;
extern volatile uint8_t imu_ready_watch;
extern volatile uint32_t imu_update_count_watch;
extern volatile float imu_roll_watch;
extern volatile float imu_pitch_watch;
extern volatile float imu_yaw_watch;

uint8_t IMU_Init(void);
void IMU_Task_1ms(void);

#ifdef __cplusplus
}
#endif

#endif
