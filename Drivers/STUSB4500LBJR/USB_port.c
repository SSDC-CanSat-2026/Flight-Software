/*
 * USB_port.c
 *
 *  Created on: Nov 10, 2025
 *      Author: Joel
 */

/*
 * The STUSB4500 is used to negotiate the USB PD system.
 * This allows us to charge the batteries used to power the rest
 *    of the board without having to remove them and shutdown.
 *
 * The STUSB also helps with ESD protection for the data lines
 *    so that we might actually have usable DFU for the FSW.
 */

#include "USB_port.h"

#include "stm32g4xx_hal.h"

#define STUSB4500_addr_r 0x50 // Bit 0 is R/nW
#define STUSB4500_addr_w 0x51

#define NUM_PDOS 2
#define SOFT_RESET 0x0D
#define SEND_COMMAND 0x26

HAL_StatusTypeDef USB_init(I2C_HandleTypeDef *hi2c) {
	uint8_t rx[3];
	// bool type does not exist w/o header. There is also no True or False keywords, use 1 and 0.
	// _Bool ret = 1;  // Note that this variable is not used.

	// Read from all the status registers to clear them at the start
	uint8_t alert_addr_start = 0x0B;
	for (int i=0;i<=12;i++) /* clear ALERT Status */
	{
		HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c, STUSB4500_addr_r, alert_addr_start+i, I2C_MEMADD_SIZE_8BIT, rx, 1, HAL_MAX_DELAY);  // clear ALERT Status
		if(status != HAL_OK) {
			return status;
		}
	}

	// Set the number of PDOs to 2 (20V charging)
	HAL_StatusTypeDef status = Set_PDOs(hi2c);
	if (status != HAL_OK) {
		return status;
	}
	// Set a soft reset to re-negotiate any potential connection that may exist already
	status = Soft_Reset(hi2c);
	if (status != HAL_OK) {
		return status;
	}


	return status;
}

HAL_StatusTypeDef Soft_Reset(I2C_HandleTypeDef *hi2c) {
	uint8_t rx;

	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, STUSB4500_addr_w, 0x51, I2C_MEMADD_SIZE_8BIT, SOFT_RESET, 1, HAL_MAX_DELAY);
	status = HAL_I2C_Mem_Write(hi2c, STUSB4500_addr_w, 0x1A, I2C_MEMADD_SIZE_8BIT, SEND_COMMAND, 1, HAL_MAX_DELAY);

	return status;
}

HAL_StatusTypeDef Set_PDOs(I2C_HandleTypeDef *hi2c) {\
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, STUSB4500_addr_w, 0x70, I2C_MEMADD_SIZE_8BIT, NUM_PDOS, 1, HAL_MAX_DELAY);
}
