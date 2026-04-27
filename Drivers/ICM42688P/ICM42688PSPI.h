#ifndef _ICM42688PSPI_H_
#define _ICM42688PSPI_H_

#include <stm32g491xx.h>
#include "stm32g4xx_hal.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif


typedef struct ICM42688P_AccelData
{
    float accel_x;
    float accel_y;
    float accel_z;

    float gyro_p;
    float gyro_y;
    float gyro_r;
} ICM42688P_AccelData;

int16_t ICM42688P_read_reg(uint8_t reg);

ICM42688P_AccelData ICM42688P_read_data();

uint8_t ICM42688P_init(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef* chip_select_port, uint16_t chip_select_gpio_pin);

#endif
