#include "AMT10E2.h"

// Initialize encoder interface on TIM1
void QENC_Init_Encoder0(void) {
    TIM_Encoder_InitTypeDef encoderConfig = {0};

    encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    encoderConfig.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
    encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC1Filter = 0;

    encoderConfig.IC2Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
    encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC2Filter = 0;

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0xFFFF;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    HAL_TIM_Encoder_Init(&htim1, &encoderConfig);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}

// Initialize encoder interface on TIM8
void QENC_Init_Encoder1(void) {
    // same setup as above, just with htim8 and TIM8
    TIM_Encoder_InitTypeDef encoderConfig = {0};
    encoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    encoderConfig.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
    encoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC1Filter = 0;
    encoderConfig.IC2Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
    encoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    encoderConfig.IC2Filter = 0;

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 0xFFFF;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

    HAL_TIM_Encoder_Init(&htim8, &encoderConfig);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
}

// Get signed count from Encoder 0
int16_t QENC_Get_Encoder0_Count(void) {
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

// Get signed count from Encoder 1
int16_t QENC_Get_Encoder1_Count(void) {
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
}
