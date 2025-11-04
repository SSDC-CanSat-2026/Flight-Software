#include "LC76G.h"
#include <string.h>
#include <math.h>

// Initialize global fields
uint8_t gps_buffer[GPS_DMA_BUFFER_SIZE] = {0}; // ASCII character uses 8 bits.

LC76G_gps_data gps_data;

void LC76G_init()
{
    // Disable all other types of NEMA sentences
    HAL_UART_Transmit(&huart5, LC76_DISABLE_GGL, strlen(LC76_DISABLE_GGL), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    HAL_UART_Transmit(&huart5, LC76_DISABLE_GSA, strlen(LC76_DISABLE_GSA), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    HAL_UART_Transmit(&huart5, LC76_DISABLE_GSV, strlen(LC76_DISABLE_GSV), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    HAL_UART_Transmit(&huart5, LC76_DISABLE_RMC, strlen(LC76_DISABLE_RMC), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    HAL_UART_Transmit(&huart5, LC76_DISABLE_VTG8, strlen(LC76_DISABLE_VTG8), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    // Enable GGA messages.
    HAL_UART_Transmit(&huart5, LC76_ENABLE_GGA, strlen(LC76_ENABLE_GGA), TIMEOUT);
    HAL_UART_Receive(&huart5, NULL, 32, TIMEOUT);

    // Clear UART idle flag if needed
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart5);
    }
}

void LC76G_read_data(){
	HAL_UART_TRANSMIT(&huart5, gps_buffer, 512, TIMEOUT);

	// Please see 2.2.2 GGA in "GNSS Protocol Guide LC67G"
	// Ex: $GNGGA,040143.000,3149.334166,N,11706.941670,E,2,36,0.48,61.496,M,-0.335,M,,*58

}
