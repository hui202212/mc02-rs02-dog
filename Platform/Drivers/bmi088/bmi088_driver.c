/**
 * @file    bmi088_driver.c
 * @brief   BMI088 传感器驱动（平台无关版本）
 *
 * 与原版的区别：
 * - 所有 HAL_SPI_TransmitReceive → bmi088_spi_read_write_byte
 * - 所有 HAL_GPIO_WritePin → bmi088_*_cs_low/high
 * - 所有 HAL_Delay → bmi088_delay_ms
 * - 所有 DWT_Delay → bmi088_delay_us
 * - 所有 DWT_GetTimeline_s → bmi088_get_timeline_s
 * - BMI088_Init/BMI088_init 不再接收 SPI_HandleTypeDef* 参数
 */
#include "bmi088_driver.h"
#include "bmi088_reg.h"
#include "bmi088_platform.h"
#include <math.h>
#include <string.h>

float BMI088_ACCEL_SEN = BMI088_ACCEL_6G_SEN;
float BMI088_GYRO_SEN  = BMI088_GYRO_2000_SEN;

static uint8_t  res           = 0;
static uint8_t  write_reg_num = 0;
static uint8_t  error         = BMI088_NO_ERROR;
float  gyroDiff[3], gNormDiff;

uint8_t caliOffset = 1;
int16_t caliCount  = 0;

IMU_Data_t BMI088;

/* ============ SPI 读写宏（自动管理 CS） ============ */

#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        bmi088_accel_cs_low();                   \
        BMI088_write_single_reg((reg), (data));  \
        bmi088_accel_cs_high();                  \
    }

#define BMI088_accel_read_single_reg(reg, data)  \
    {                                            \
        bmi088_accel_cs_low();                   \
        bmi088_spi_read_write_byte((reg) | 0x80);\
        bmi088_spi_read_write_byte(0x55);        \
        (data) = bmi088_spi_read_write_byte(0x55);\
        bmi088_accel_cs_high();                  \
    }

#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        bmi088_accel_cs_low();                     \
        bmi088_spi_read_write_byte((reg) | 0x80);  \
        BMI088_read_muli_reg(reg, data, len);      \
        bmi088_accel_cs_high();                    \
    }

#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        bmi088_gyro_cs_low();                   \
        BMI088_write_single_reg((reg), (data)); \
        bmi088_gyro_cs_high();                  \
    }

#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        bmi088_gyro_cs_low();                   \
        BMI088_read_single_reg((reg), &(data)); \
        bmi088_gyro_cs_high();                  \
    }

#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        bmi088_gyro_cs_low();                       \
        BMI088_read_muli_reg((reg), (data), (len)); \
        bmi088_gyro_cs_high();                      \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

/* ============ 寄存器配置表 ============ */

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

static void Calibrate_MPU_Offset(IMU_Data_t *bmi088);

/* ============ 校准参数默认值（从原项目标定结果得到） ============ */
#ifndef GxOFFSET
#define GxOFFSET -0.000681414269f
#endif
#ifndef GyOFFSET
#define GyOFFSET -0.00134240754f
#endif
#ifndef GzOFFSET
#define GzOFFSET -0.00143384014f
#endif
#ifndef AxOFFSET
#define AxOFFSET -0.127637371f
#endif
#ifndef AyOFFSET
#define AyOFFSET -0.0857178643f
#endif
#ifndef AzOFFSET
#define AzOFFSET 9.59094429f
#endif
#ifndef gNORM
#define gNORM 9.84484291f
#endif
/* ============ 公开 API ============ */

void BMI088_Init(uint8_t calibrate)
{
    while (BMI088_init(calibrate))
    {
        ;
    }
}

uint8_t BMI088_init(uint8_t calibrate)
{
    error = BMI088_NO_ERROR;

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    if (calibrate)
    {
        Calibrate_MPU_Offset(&BMI088);
    }
    else
    {
        BMI088.GyroOffset[0] = GxOFFSET;
        BMI088.GyroOffset[1] = GyOFFSET;
        BMI088.GyroOffset[2] = GzOFFSET;

        BMI088.AccelOffset[0] = AxOFFSET;
        BMI088.AccelOffset[1] = AyOFFSET;
        BMI088.AccelOffset[2] = AzOFFSET;

        BMI088.gNorm       = gNORM;
        BMI088.AccelScale  = 9.81f / BMI088.gNorm;
        BMI088.TempWhenCali = 40;
    }

    return error;
}

/* ============ 校准 ============ */

static void Calibrate_MPU_Offset(IMU_Data_t *bmi088)
{
    static float startTime;
    static uint16_t CaliTimes     = 12000;
    static uint16_t acc_CaliTimes = 12000;
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;
    float gyroMax[3], gyroMin[3];
    float gNormTemp, gNormMax, gNormMin;

    startTime = bmi088_get_timeline_s();
    do
    {
        if (bmi088_get_timeline_s() - startTime > 10)
        {
            bmi088->GyroOffset[0] = GxOFFSET;
            bmi088->GyroOffset[1] = GyOFFSET;
            bmi088->GyroOffset[2] = GzOFFSET;
            bmi088->gNorm         = gNORM;
            bmi088->TempWhenCali  = 40;
            break;
        }

        bmi088_delay_us(5000);  /* 5ms */
        bmi088->gNorm = 0;
        bmi088->GyroOffset[0] = 0;
        bmi088->GyroOffset[1] = 0;
        bmi088->GyroOffset[2] = 0;

        for (uint16_t i = 0; i < CaliTimes; i++)
        {
            BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);
            bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
            bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
            gNormTemp = sqrtf(bmi088->Accel[0] * bmi088->Accel[0] +
                              bmi088->Accel[1] * bmi088->Accel[1] +
                              bmi088->Accel[2] * bmi088->Accel[2]);
            bmi088->gNorm += gNormTemp;

            BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
            {
                bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
                bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[0] += bmi088->Gyro[0];
                bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
                bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[1] += bmi088->Gyro[1];
                bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
                bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
                bmi088->GyroOffset[2] += bmi088->Gyro[2];
            }

            if (i == 0)
            {
                gNormMax = gNormTemp;
                gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    gyroMax[j] = bmi088->Gyro[j];
                    gyroMin[j] = bmi088->Gyro[j];
                }
            }
            else
            {
                if (gNormTemp > gNormMax) gNormMax = gNormTemp;
                if (gNormTemp < gNormMin) gNormMin = gNormTemp;
                for (uint8_t j = 0; j < 3; j++)
                {
                    if (bmi088->Gyro[j] > gyroMax[j]) gyroMax[j] = bmi088->Gyro[j];
                    if (bmi088->Gyro[j] < gyroMin[j]) gyroMin[j] = bmi088->Gyro[j];
                }
            }

            gNormDiff = gNormMax - gNormMin;
            for (uint8_t j = 0; j < 3; j++)
                gyroDiff[j] = gyroMax[j] - gyroMin[j];

            if (gNormDiff > 0.5f ||
                gyroDiff[0] > 0.15f || gyroDiff[1] > 0.15f || gyroDiff[2] > 0.15f)
                break;

            bmi088_delay_us(500);  /* 0.5ms */
        }

        bmi088->gNorm /= (float)CaliTimes;
        for (uint8_t i = 0; i < 3; i++)
            bmi088->GyroOffset[i] /= (float)CaliTimes;

        BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
        bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
        if (bmi088_raw_temp > 1023)
            bmi088_raw_temp -= 2048;
        bmi088->TempWhenCali = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

        caliCount++;
    } while (gNormDiff > 0.5f ||
             fabsf(bmi088->gNorm - 9.8f) > 0.5f ||
             gyroDiff[0] > 0.15f || gyroDiff[1] > 0.15f || gyroDiff[2] > 0.15f ||
             fabsf(bmi088->GyroOffset[0]) > 0.01f ||
             fabsf(bmi088->GyroOffset[1]) > 0.01f ||
             fabsf(bmi088->GyroOffset[2]) > 0.01f);

    bmi088->AccelScale = 9.81f / bmi088->gNorm;

    for (uint16_t i = 0; i < acc_CaliTimes; i++)
    {
        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

        bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
        bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
        bmi088->AccelOffset[0] = bmi088->AccelOffset[0] * 0.8f + bmi088->Accel[0] * 0.2f;

        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
        bmi088->AccelOffset[1] = bmi088->AccelOffset[1] * 0.8f + bmi088->Accel[1] * 0.2f;

        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
        bmi088->AccelOffset[2] = bmi088->AccelOffset[2] * 0.8f + bmi088->Accel[2] * 0.2f;

        bmi088_delay_us(500);
    }
}

/* ============ 传感器初始化 ============ */

uint8_t bmi088_accel_init(void)
{
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bmi088_delay_ms(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bmi088_delay_ms(1);

    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    bmi088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bmi088_delay_ms(1);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bmi088_delay_ms(1);

    if (res != BMI088_ACC_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;

    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {
        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0],
                                       write_BMI088_accel_reg_data_error[write_reg_num][1]);
        bmi088_delay_ms(1);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        bmi088_delay_ms(1);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            error |= write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t bmi088_gyro_init(void)
{
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bmi088_delay_ms(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bmi088_delay_ms(1);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    bmi088_delay_ms(BMI088_LONG_DELAY_TIME);

    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bmi088_delay_ms(1);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bmi088_delay_ms(1);

    if (res != BMI088_GYRO_CHIP_ID_VALUE)
        return BMI088_NO_SENSOR;

    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {
        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0],
                                      write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        bmi088_delay_ms(1);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        bmi088_delay_ms(1);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            /* 禁止下标自减：第0项失败时会从0下溢到255并卡死初始化。 */
            error |= write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

/* ============ 读取传感器数据 ============ */

void BMI088_Read(IMU_Data_t *bmi088)
{
    static uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    static int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    if (caliOffset)
    {
        bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
        bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale - bmi088->AccelOffset[0];
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale - bmi088->AccelOffset[1];
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
    }
    else
    {
        bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
        bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN * bmi088->AccelScale;
    }

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[0];
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[1];
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
        }
    }

    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);
    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));
    if (bmi088_raw_temp > 1023)
        bmi088_raw_temp -= 2048;
    bmi088->Temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/* ============ SPI 底层操作（已通过平台抽象层实现） ============ */

static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    bmi088_spi_read_write_byte(reg);
    bmi088_spi_read_write_byte(data);
}

static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    bmi088_spi_read_write_byte(reg | 0x80);
    *return_data = bmi088_spi_read_write_byte(0x55);
}

static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    bmi088_spi_read_write_byte(reg | 0x80);
    while (len != 0)
    {
        *buf = bmi088_spi_read_write_byte(0x55);
        buf++;
        len--;
    }
}
