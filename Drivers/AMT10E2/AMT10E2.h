#ifndef _AMT10E2_H_
#define _AMT10E2_H_

#include "../STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal.h"
#include <stdint.h>

// TIM handles, should be defined/initialized elsewhere (e.g., in main or HAL config)
extern TIM_HandleTypeDef htim1; // Encoder 0
extern TIM_HandleTypeDef htim8; // Encoder 1

// Initialize encoder interface on TIM1
void QENC_Init_Encoder0(void);

void QENC_Init_Encoder1(void);

int16_t QENC_Get_Encoder0_Count(void);

int16_t QENC_Get_Encoder1_Count(void);

#endif /* _AMT10E2_H_ */
