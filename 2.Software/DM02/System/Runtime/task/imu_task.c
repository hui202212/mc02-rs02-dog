/**
 * @file imu_task.c
 * @brief BMI088 + Mahony姿态解算；只能由main主循环调用
 */

#include "imu_task.h"

#include "bmi088_platform.h"
#include "mahony_filter.h"
#include "spi.h"

#include <math.h>
#include <string.h>

IMU_t IMU;

volatile uint8_t imu_init_status_watch = 0xFFU;
volatile uint8_t imu_ready_watch = 0U;
volatile uint32_t imu_update_count_watch = 0U;
volatile float imu_roll_watch = 0.0f;
volatile float imu_pitch_watch = 0.0f;
volatile float imu_yaw_watch = 0.0f;

static struct MAHONY_FILTER_t mahony;
static uint8_t imu_hardware_ready = 0U;
static uint32_t imu_converge_ms = 0U;

uint8_t IMU_Init(void)
{
    memset(&IMU, 0, sizeof(IMU));
    bmi088_set_spi_handle(&hspi2);

    /* 单次初始化，失败后直接返回；禁止阻塞重试影响电机任务。 */
    imu_init_status_watch = BMI088_init(0U);
    mahony_init(&mahony, 1.0f, 0.0f, 0.001f);
    imu_hardware_ready = imu_init_status_watch == BMI088_NO_ERROR ? 1U : 0U;
    imu_ready_watch = 0U;
    imu_converge_ms = 0U;
    return imu_init_status_watch;
}

void IMU_Task_1ms(void)
{
    if (!imu_hardware_ready) return;

    BMI088_Read(&BMI088);
    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        IMU.Accel[axis] = BMI088.Accel[axis];
        IMU.Gyro[axis] = BMI088.Gyro[axis];
    }

    float accel_norm = sqrtf(IMU.Accel[0] * IMU.Accel[0]
                           + IMU.Accel[1] * IMU.Accel[1]
                           + IMU.Accel[2] * IMU.Accel[2]);
    if (!isfinite(accel_norm) || accel_norm < 1.0f) return;

    Axis3f gyro = {IMU.Gyro[0], IMU.Gyro[1], IMU.Gyro[2]};
    Axis3f accel = {IMU.Accel[0], IMU.Accel[1], IMU.Accel[2]};
    mahony_input(&mahony, gyro, accel);
    mahony_update(&mahony);
    mahony_output(&mahony);

    IMU.q[0] = mahony.q0;
    IMU.q[1] = mahony.q1;
    IMU.q[2] = mahony.q2;
    IMU.q[3] = mahony.q3;
    IMU.Roll = mahony.roll;
    IMU.Pitch = mahony.pitch;
    IMU.Yaw = mahony.yaw;

    imu_roll_watch = IMU.Roll;
    imu_pitch_watch = IMU.Pitch;
    imu_yaw_watch = IMU.Yaw;
    imu_update_count_watch++;

    /* 上电静置约3秒后才允许姿态参与坡度补偿。 */
    if (imu_converge_ms < 3000U) {
        imu_converge_ms++;
    } else {
        IMU.imu_flag = 1U;
        imu_ready_watch = 1U;
    }
}
