/**
 * @file    bmi088_reg.h
 * @brief   BMI088 寄存器定义
 */
#ifndef BMI088REG_H
#define BMI088REG_H

/***************** ACCEL part *******************/
#define BMI088_ACC_CHIP_ID                0x00
#define BMI088_ACC_CHIP_ID_VALUE          0x1E
#define BMI088_ACC_RESERVED               0x01
#define BMI088_ACC_DIAM                  0x02
#define BMI088_ACC_ERR_REG               0x02
#define BMI088_ACC_STATUS                0x03
#define BMI088_ACCEL_XOUT_L              0x12
#define BMI088_ACCEL_XOUT_H              0x13
#define BMI088_ACCEL_YOUT_L              0x14
#define BMI088_ACCEL_YOUT_H              0x15
#define BMI088_ACCEL_ZOUT_L              0x16
#define BMI088_ACCEL_ZOUT_H              0x17
#define BMI088_SENSORTIME_0              0x18
#define BMI088_SENSORTIME_1              0x19
#define BMI088_SENSORTIME_2              0x1A
#define BMI088_ACC_INT_STAT_1            0x1D
#define BMI088_TEMP_M                    0x22
#define BMI088_TEMP_L                    0x23
#define BMI088_ACC_CONF                  0x40
#define BMI088_ACC_RANGE                 0x41
#define BMI088_INT1_IO_CTRL              0x53
#define BMI088_INT2_IO_CTRL              0x54
#define BMI088_INT_MAP_DATA              0x58
#define BMI088_ACC_SELF_TEST             0x6D
#define BMI088_ACC_PWR_CONF              0x7C
#define BMI088_ACC_PWR_CTRL              0x7D
#define BMI088_ACC_SOFTRESET             0x7E
#define BMI088_ACC_SOFTRESET_VALUE       0xB6

/* ACCEL CONFIG */
#define BMI088_ACC_NORMAL                0x00
#define BMI088_ACC_800_HZ                0x0B
#define BMI088_ACC_CONF_MUST_Set         0x80
#define BMI088_ACC_RANGE_3G              0x00
#define BMI088_ACC_RANGE_6G              0x01
#define BMI088_ACC_RANGE_12G             0x02
#define BMI088_ACC_RANGE_24G             0x03
#define BMI088_ACC_ENABLE_ACC_ON         0x04
#define BMI088_ACC_PWR_ACTIVE_MODE       0x00
#define BMI088_ACC_INT1_IO_ENABLE        0x01
#define BMI088_ACC_INT1_GPIO_PP          0x00
#define BMI088_ACC_INT1_GPIO_LOW         0x00
#define BMI088_ACC_INT1_DRDY_INTERRUPT   0x02

/***************** GYRO part *******************/
#define BMI088_GYRO_CHIP_ID              0x00
#define BMI088_GYRO_CHIP_ID_VALUE        0x0F
#define BMI088_GYRO_X_LSB                0x02
#define BMI088_GYRO_X_MSB                0x03
#define BMI088_GYRO_Y_LSB                0x04
#define BMI088_GYRO_Y_MSB                0x05
#define BMI088_GYRO_Z_LSB                0x06
#define BMI088_GYRO_Z_MSB                0x07
#define BMI088_GYRO_INT_STAT_1           0x0A
#define BMI088_GYRO_FIFO_STATUS          0x0E
#define BMI088_GYRO_RANGE                0x0F
#define BMI088_GYRO_BANDWIDTH            0x10
#define BMI088_GYRO_LPM1                 0x11
#define BMI088_GYRO_SOFTRESET            0x14
#define BMI088_GYRO_SOFTRESET_VALUE      0xB6
#define BMI088_GYRO_CTRL                 0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF    0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP     0x18
#define BMI088_GYRO_SELF_TEST            0x3C

/* GYRO CONFIG */
#define BMI088_GYRO_2000                 0x00
#define BMI088_GYRO_1000                 0x01
#define BMI088_GYRO_500                  0x02
#define BMI088_GYRO_250                  0x03
#define BMI088_GYRO_125                  0x04
#define BMI088_GYRO_2000_230_HZ          0x00
#define BMI088_GYRO_BANDWIDTH_MUST_Set   0x80
#define BMI088_GYRO_NORMAL_MODE          0x00
#define BMI088_DRDY_ON                   0x80
#define BMI088_GYRO_INT3_GPIO_PP         0x00
#define BMI088_GYRO_INT3_GPIO_LOW        0x00
#define BMI088_GYRO_DRDY_IO_INT3         0x01

#endif /* BMI088REG_H */
