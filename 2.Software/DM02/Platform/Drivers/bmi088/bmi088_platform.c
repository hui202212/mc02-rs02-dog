/**
 * @file    bmi088_platform.c
 * @brief   BMI088 平台接口实现 — STM32H723 + SPI2
 */
#include "bmi088_platform.h"
#include "spi.h"
#include "gpio.h"
#include "main.h"

static SPI_HandleTypeDef *imu_spi = NULL;

void bmi088_set_spi_handle(void *handle)
{
    imu_spi = (SPI_HandleTypeDef *)handle;
}

/* ======================== CS 控制 ======================== */

void bmi088_accel_cs_low(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
}

void bmi088_accel_cs_high(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
}

void bmi088_gyro_cs_low(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
}

void bmi088_gyro_cs_high(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
}

/* ======================== SPI 收发 ======================== */

uint8_t bmi088_spi_read_write_byte(uint8_t txdata)
{
    uint8_t rxdata = 0;
    if (imu_spi == NULL) return 0U;
    /* 传感器或接线异常时最多等待1ms，不能永久卡住主循环。 */
    (void)HAL_SPI_TransmitReceive(imu_spi, &txdata, &rxdata, 1, 1U);
    return rxdata;
}

/* ======================== 延时 ======================== */

void bmi088_delay_us(uint32_t us)
{
    volatile uint32_t cnt = us * 27U;
    while (cnt--) { __NOP(); }
}

void bmi088_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

/* ======================== 时间戳 ======================== */

float bmi088_get_timeline_s(void)
{
    static uint32_t last_cnt = 0;
    static uint64_t overflow_cnt = 0;
    uint32_t now = DWT->CYCCNT;

    if (now < last_cnt) overflow_cnt++;
    last_cnt = now;

    uint64_t cycles = (overflow_cnt << 32) + now;
    return (float)cycles / 400000000.0f;  /* 400 MHz HCLK */
}
