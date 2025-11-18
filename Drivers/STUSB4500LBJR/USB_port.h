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
#include "USBPD_spec_defines.h"

void USB_init();
void USB_SW_reset();


#endif /* STUSB4500LBJR_USB_PORT_H_ */
