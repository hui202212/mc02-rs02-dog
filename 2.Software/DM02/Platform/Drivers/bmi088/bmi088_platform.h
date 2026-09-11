/**
 * @file    bmi088_platform.h
 * @brief   BMI088 平台抽象接口
 *
 * 用户需要在新平台上实现以下所有函数，然后将此头文件包含到驱动中。
 * 参见 example/stm32/bmi088_platform_stm32.c 作为 STM32 HAL 参考实现。
 */
#ifndef BMI088_PLATFORM_H
#define BMI088_PLATFORM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 平台需实现的接口 ======================== */

/**
 * @brief   SPI 单字节全双工收发
 * @param  txdata  发送的数据
 * @return 接收到的数据
 */
uint8_t bmi088_spi_read_write_byte(uint8_t txdata);

/* --- 加速度计片选 --- */
void bmi088_accel_cs_low(void);
void bmi088_accel_cs_high(void);

/* --- 陀螺仪片选 --- */
void bmi088_gyro_cs_low(void);
void bmi088_gyro_cs_high(void);

/**
 * @brief   微秒级延时
 * @param   us  延时微秒数
 */
void bmi088_delay_us(uint32_t us);

/**
 * @brief   毫秒级延时
 * @param   ms  延时毫秒数
 */
void bmi088_delay_ms(uint32_t ms);

/**
 * @brief   获取从系统启动到当前的时间（秒），用于校准超时判断
 * @return  时间（秒）
 */
float bmi088_get_timeline_s(void);

/* ======================== 平台初始化辅助（可选） ======================== */

/**
 * @brief   如果平台需要传递 SPI 句柄，使用此函数
 * @param   handle  平台 SPI 句柄指针（void* 以避免类型依赖）
 */
void bmi088_set_spi_handle(void *handle);

#ifdef __cplusplus
}
#endif

#endif /* BMI088_PLATFORM_H */
