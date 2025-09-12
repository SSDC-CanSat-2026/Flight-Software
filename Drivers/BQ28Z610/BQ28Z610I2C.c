/*
 * BQ28Z610I2C.c
 *
 *  Created on: Apr 2, 2025
 *      Author: Joel
 */

#include "BQ28Z610I2C.h"

#include "stm32g4xx_hal.h"

#define CMD_VOLTAGE 0x08
#define BQ28Z610_I2C_ADDR (0x55 << 1)

HAL_StatusTypeDef BQ28Z610_ReadVoltage(I2C_HandleTypeDef *hi2c, uint16_t *voltage)
{
	uint8_t rx[2];
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, BQ28Z610_I2C_ADDR, CMD_VOLTAGE, I2C_MEMADD_SIZE_8BIT, rx, 2, HAL_MAX_DELAY);

	if (status == HAL_OK)
	{
		uint16_t shifted = rx[1] << 8;
		uint16_t lower = rx[0];
		*voltage = shifted | lower;
	}
	return status;
};
