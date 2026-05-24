/*
 * File: SERVO.h
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Ver: 1.0
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#ifndef SERVO_H_
#define SERVO_H_

#define HAL_TIM_MODULE_ENABLED

#define APB1_clk 64000000
#define APB2_clk 64000000
#define min_pulse 0.65
#define max_pulse 2.5

#include "stm32g4xx_hal.h"


// The Number OF Servo Motors To Be Used In The Project
#define SERVO_NUM  5

typedef struct
{
	GPIO_TypeDef * SERVO_GPIO;
	uint16_t       SERVO_PIN;
	TIM_TypeDef*   TIM_Instance;
	volatile uint32_t*      TIM_CCRx;
	uint32_t       PWM_TIM_CH;
	uint32_t       TIM_CLK;
	float          MinPulse;
	float          MaxPulse;
	TIM_HandleTypeDef* Handle;
}SERVO_CfgType;


/*-----[ Prototypes For All Functions ]-----*/

void SERVO_Init(uint16_t au16_SERVO_Instance, TIM_HandleTypeDef *htim);
void SERVO_MoveTo(uint16_t au16_SERVO_Instance, float af_Angle);
void SERVO_RawMove(uint16_t au16_SERVO_Instance, uint16_t au16_Pulse);
uint16_t SERVO_Get_MaxPulse(uint16_t au16_SERVO_Instance);
uint16_t SERVO_Get_MinPulse(uint16_t au16_SERVO_Instance);
void SERVO_Sweep(uint16_t au16_SERVO_Instance);


#endif /* SERVO_H_ */
