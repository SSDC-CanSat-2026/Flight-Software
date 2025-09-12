/*
 * BQ28Z610I2C.h
 *
 *  Created on: Apr 2, 2025
 *      Author: Joel
 */

#ifndef BQ28Z610_BQ28Z610I2C_H_
#define BQ28Z610_BQ28Z610I2C_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

/*
typedef struct BQ28Z610_Voltage
{
	int16_t voltage;
	int8_t error;
} BQ28Z610_Voltage;
*/

HAL_StatusTypeDef BQ28Z610_ReadVoltage(I2C_HandleTypeDef *hi2c, uint16_t *voltage);

#endif /* BQ28Z610_BQ28Z610I2C_H_ */
