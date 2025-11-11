#include "LC76G.h"
#include <string.h>
#include <stdlib.h>
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

	// Splitting a string which will include empty strings for error checking.
	// https://stackoverflow.com/questions/31377038/split-a-char-array-into-separate-strings

	char* str_copy = strdup(gps_buffer);
    char* running_copy = str_copy; // A pointer we can move without losing the start address for freeing
    char* token;
    const char delimiters[] = ",";


    uint8_t index = 0;
    // Loop through the string, extracting tokens including empty ones
    while ((token = strsep(&running_copy, delimiters)) != NULL) {
       // Token contains string in between commas (include empty strings)
    	if(index == 1){ 		// UTC in hhmmss:sss
    		gps_data.time_H = (token[0] - '0') * 10 + (token[1] - '0');
    		gps_data.time_M = (token[2] - '0') * 10 + (token[3] - '0');
    		gps_data.time_S = (token[4] - '0') * 10 + (token[5] - '0');

    		gps_data.time_S_fraction = (token[7] - '0') / 10.0 + (token[8] - '0') / 100.0 + (token[9] - '0') / 1000.0;
    	}
    	else if(index == 2){	// Latitude with ddmm.mmmmmm
    		if(sizeof(token) > 0){
    			uint8_t degrees = (token[0] - '0') * 10 + (token[1] - '0');
    			double minutes = (token[2] - '0') * 10 + (token[3] - '0');

    			gps_data.lat = degrees + (minutes / 60.0);

    			gps_data.decimal_minute_lat = (token[5] - '0') / 10.0 + (token[6] - '0') / 100.0 + (token[7] - '0') / 1000.0 + (token[8] - '0') / 10000.0 + (token[9] - '0') / 100000.0 + (token[10] - '0') / 1000000.0;
    		}
    	}
    	else if(index == 3){ 	// N/S
    		if(sizeof(token) > 0 && token[0] == 'S'){
    			gps_data.lat = -1.0 * gps_data.lat;
    		}
    	}
    	else if(index == 4){	// Longitude with dddmm.mmmmmm
    		if(sizeof(token) > 0){
				uint8_t degrees = (token[0] - '0') * 100 + (token[1] - '0') * 10 + (token[2] - '0');
				double minutes = (token[3] - '0') * 10 + (token[4] - '0');

				gps_data.lon = degrees + (minutes / 60.0);

				gps_data.decimal_minute_long = (token[5] - '0') / 10.0 + (token[6] - '0') / 100.0 + (token[7] - '0') / 1000.0 + (token[8] - '0') / 10000.0 + (token[9] - '0') / 100000.0 + (token[10] - '0') / 1000000.0;
    		}
    	}
    	else if(index == 5){ 	// E/W
    		if(sizeof(token) > 0 && token[0] == 'W'){
    			gps_data.lon = -1.0 * gps_data.lon;
    		}
    	}
    	else if(index == 6){ 	// Quality
    		gps_data.quality = (token[0] - '0');
    	}
    	else if(index == 7){ 	// NumSatUsed
    		gps_data.num_sat_used = (token[0] - '0') * 10 + (token[1] - '0');
    	}
    	else if(index == 8){	// HDOP (Horizontal Dilution of Precision
    		if(sizeof(token) > 0){
    			if(token[1] == '.'){ // HDOP is between 0 and 9
    				gps_data.horizontal_dilution_of_precision = (token[0] - '0') + (token[2] - '0') / 10.0 + (token[3] - '0') / 100.0;
    			}
    			else{
    				gps_data.horizontal_dilution_of_precision = (token[0] - '0') * 10 + (token[1] - '0') + (token[3] - '0') / 10.0 + (token[4] - '0') / 100.0;
    			}
    		}
    	}
    	else if(index == 9){	// Altitude
    		// Because the token is a char *, you can use strtod() to turn it into a double value.
    		char *endptr_for_error_checking;

    		double result = strtod(token, &endptr_for_error_checking);

    		gps_data.altitude = result;
    	}
    	else if(index == 11){ // Geoid Seperation (Optional)
    		char *endptr_for_error_checking;

    		double result = strtod(token, &endptr_for_error_checking);

    		gps_data.geoid_seperation = result;
    	}


    	index++;
    }

    // Free the allocated memory for the copy
    free(str_copy);

    return 0;

}
