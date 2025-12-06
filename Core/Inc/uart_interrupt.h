/*
 * uart_interrupt.h
 *
 *  Created on: Feb 25, 2025
 *      Author: bbevel0133
 */

#ifndef INC_UART_INTERRUPT_H_
#define INC_UART_INTERRUPT_H_

#include "stm32g4xx_hal.h"

#define UART_BUFFER_SIZE 32

extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

extern uint8_t uart_rx_buffer3[];
extern uint8_t uart_rx_buffer5[];

extern volatile uint8_t uart3_data_ready;
extern volatile uint8_t uart5_data_ready;

void clear_uart_buffer(UART_HandleTypeDef *huart);

//overwrite HAL method, this will be called on uart interrupt (receive)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);



#endif /* INC_UART_INTERRUPT_H_ */
