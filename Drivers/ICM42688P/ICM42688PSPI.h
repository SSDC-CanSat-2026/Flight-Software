#ifndef _ICM42688PSPI_H_
#define _ICM42688PSPI_H_

#include <stm32g491xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include "stm32g4xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct ICM42688P_AccelData
{
    // These are the traditional "linear" accelerations
    float accel_x;
    float accel_yaw;
    float accel_z;

    // Normal gyroscope values
    float gyro_p;
    float gyro_y;
    float gyro_r;

    // Old gyroscope values for accel
    float gyro_old_r;
    float gyro_old_y;
    float gyro_old_p;
    TickType_t old_time_tick;

    // Because CanSat is weird, they want accel in the R/P/Y directions
    // This just means we have to take a couple gyro measurements and 
    //   run a simple calculation on the rate change. The old 2025 code
    //   should still have the necessary logic
    float accel_r;
    float accel_p;
    float accel_y;
} ICM42688P_AccelData;

int16_t ICM42688P_read_reg(uint8_t reg);

void ICM42688P_read_data();

uint8_t ICM42688P_init(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef* chip_select_port, uint16_t chip_select_gpio_pin);

#endif
