/*
 * DRV8838.h
 *
 *  Created on: Apr 24, 2025
 *      Author: Joel Kubinsky
 */

#ifndef DRV8838_DRV8838_H_
#define DRV8838_DRV8838_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>

typedef enum {
    MOTOR_FORWARD,
    MOTOR_REVERSE
} motor_direction_t;

void drv8838_init(TIM_HandleTypeDef *htim, GPIO_TypeDef* ph_port, uint8_t ph_pin);
void drv8838_set_speed(uint8_t duty_cycle, motor_direction_t dir);
void drv8838_brake(void);

#endif /* DRV8838_DRV8838_H_ */
