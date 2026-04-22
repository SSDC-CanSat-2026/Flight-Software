/*
 * BQ28Z610I2C.c
 *
 *  Created on: Apr 2, 2025
 *      Author: Joel
 */

#include "BQ28Z610I2C.h"

#include "stm32g4xx_hal.h"

#define BQ28Z610_I2C_ADDR 	(0x55 << 1)
#define WRITE				0x00
#define READ				0x01
#define CMD_VOLTAGE  		0x08

/*HAL_StatusTypeDef BQ28Z610_ReadVoltage(I2C_HandleTypeDef *hi2c, uint16_t *voltage)
{
	uint8_t tx[2] = {0x08, 0x09}; // Register pair for Voltage
	uint8_t rx[2];

	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, (BQ28Z610_I2C_ADDR | READ), CMD_VOLTAGE, I2C_MEMADD_SIZE_8BIT, rx, 2, HAL_MAX_DELAY);

	if (status == HAL_OK)
	{
		uint16_t shifted = rx[1] << 8;
		uint16_t lower = rx[0];
		*voltage = shifted | lower;
	}
	return status;
};*/

HAL_StatusTypeDef BQ28Z610_ReadVoltage(I2C_HandleTypeDef *hi2c, uint16_t *voltage)
{

//    if (HAL_I2C_IsDeviceReady(hi2c, 0xAA, 1, 10) == HAL_OK)
//    {
//    	return HAL_OK;
//    }
//    else {
//    	return HAL_ERROR;
//    }


    uint8_t cmd = CMD_VOLTAGE;
    uint8_t rx[2];

    // Write command (SMBus style)
    if (HAL_I2C_Master_Transmit(hi2c, BQ28Z610_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    // Read 2 bytes
    if (HAL_I2C_Master_Receive(hi2c, BQ28Z610_I2C_ADDR, rx, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    *voltage = (rx[1] << 8) | rx[0];

    return HAL_OK;
}
