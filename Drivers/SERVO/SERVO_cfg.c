/*
 * File: SERVO_cfg.c
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "SERVO.h"

// timer_clk, min_pulse, and max_pulse are all in SERVO.h

SERVO_CfgType SERVO_CfgParam[SERVO_NUM] =
{
	// Servo Motor 1 Configurations
    {
	    GPIOA,
		GPIO_PIN_6,
		TIM3,
		&TIM3->CCR1,
		TIM_CHANNEL_1,
		APB1_clk,
		min_pulse,
		max_pulse
	},
	// Servo Motor 2 Configurations
	{
		GPIOA,
		GPIO_PIN_7,
		TIM3,
		&TIM3->CCR2,
		TIM_CHANNEL_2,
		APB1_clk,
		min_pulse,
		max_pulse
	},
	// Servo Motor 3 Configurations
	{
		GPIOB,
		GPIO_PIN_0,
		TIM3,
		&TIM3->CCR3,
		TIM_CHANNEL_3,
		APB1_clk,
		min_pulse,
		max_pulse
	},
	// Servo Motor 4 Configurations
	{
		GPIOB,
		GPIO_PIN_1,
		TIM3,
		&TIM3->CCR4,
		TIM_CHANNEL_4,
		APB1_clk,
		min_pulse,
		max_pulse
	},
	// Servo Motor 5 Configurations (DC Motor joint line)
	{
		GPIOA,
		GPIO_PIN_3,
		TIM15,
		&TIM15->CCR2,
		TIM_CHANNEL_2,
		APB2_clk,
		min_pulse,
		max_pulse
	},
};
