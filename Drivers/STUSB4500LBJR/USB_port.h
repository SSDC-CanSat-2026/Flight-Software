/*
 * USB_port.h
 *
 *  Created on: Nov 10, 2025
 *      Author: Joel
 */

#ifndef STUSB4500LBJR_USB_PORT_H_
#define STUSB4500LBJR_USB_PORT_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef USB_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef Soft_Reset(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef Set_PDOs(I2C_HandleTypeDef *hi2c);

#endif /* STUSB4500LBJR_USB_PORT_H_ */
