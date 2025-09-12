/*
 * DRV8838.c
 *
 *  Created on: Apr 24, 2025
 *      Author: Joel Kubinsky
 */

#include "DRV8838.h"

// If the pin/bank need to be changed this is where the change should be made
//#define PH_GPIO_Port GPIOG 
//#define PH_Pin GPIO_PIN_10 

static TIM_HandleTypeDef *motor_pwm_htim;
static GPIO_TypeDef *PH_GPIO_Port;
static uint16_t PH_Pin;

void drv8838_init(TIM_HandleTypeDef *htim, GPIO_TypeDef* ph_port, uint8_t ph_pin) {
    motor_pwm_htim = htim;
    PH_GPIO_Port = ph_port; // This is the bank of the GPIO pins for the Phase pin
    PH_Pin = ph_pin; // This is the specific GPIO pin I want to use

    // Start PWM on TIM3_CH1 (PA6)
    HAL_TIM_PWM_Start(motor_pwm_htim, TIM_CHANNEL_2);

    // Set PG10 as output (already done in MX_GPIO_Init, ideally)
    HAL_GPIO_WritePin(PH_GPIO_Port, PH_Pin, GPIO_PIN_RESET); // Default to REVERSE
}

// duty_cycle is input as a percentage value, done hoping to make it easier to program for
void drv8838_set_speed(uint8_t duty_cycle, motor_direction_t dir) {
    // Clamp to 100%
    if (duty_cycle > 100) duty_cycle = 100;

    // Set direction
    if (dir == MOTOR_FORWARD) {
    	HAL_GPIO_WritePin(PH_GPIO_Port, PH_Pin, GPIO_PIN_SET);
    } else {
    	HAL_GPIO_WritePin(PH_GPIO_Port, PH_Pin, GPIO_PIN_RESET);
    }

    // Map 0–100 to CCR register (assumes 100% = ARR value)
    uint32_t pulse = (__HAL_TIM_GET_AUTORELOAD(motor_pwm_htim) * duty_cycle) / 100;
    __HAL_TIM_SET_COMPARE(motor_pwm_htim, TIM_CHANNEL_1, pulse);
}

void drv8838_brake(void) {
    __HAL_TIM_SET_COMPARE(motor_pwm_htim, TIM_CHANNEL_1, 0);
}
