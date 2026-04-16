/*
 * microSD.c
 *
 *  Created on: Apr 9, 2026
 *      Author: Joel
 */

#include "../Inc/microSD.h"

void init_SD(void){

//	if(f_mount(&global_micro_sd_data.FatFs, "", 1) != FR_OK){
	if (f_mount(&USERFatFs, USERPath, 1) != FR_OK) {
		global_micro_sd_data.successfullyMounted = 0;

		char header_string[203];
		uint16_t str_len = sprintf(header_string, "TEAM_ID,MISSION_TIME,PACKET_COUNT,MODE,STATE,ALTITUDE,TEMPERATURE,PRESSURE,VOLTAGE,CURRENT,GYRO_R,GYRO_P,GYRO_Y,ACCEL_R,ACCEL_P,ACCEL_Y,GPS_TIME,GPS_ALTITUDE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,CMD_ECHO");
		write_SD(header_string, str_len, "CanSat_Data_2026.csv");
		
		// TODO: ADD LED DEBUGGING LIGHTS HERE FOR LED
		return;
	}

	global_micro_sd_data.successfullyMounted = 1;;
}

void read_SD(char* buf, char filename[]) {

	UINT bytesRead;
	FRESULT result;

	if (global_micro_sd_data.successfullyMounted == 1) {

		char filepath[32];
		snprintf(filepath, sizeof(filepath), "%s%s", USERPath, filename);
		result = f_open(&global_micro_sd_data.Fil, filepath, FA_READ);
		if (result != FR_OK) {
			return;
		}

		char output[100];
		result = f_read(&global_micro_sd_data.Fil, &output, f_size(&global_micro_sd_data.Fil), &bytesRead);
		if (result != FR_OK || bytesRead == 0) {
			return;
		}

		// Format of Config file (.bin file)

	}
}

uint32_t write_SD(char* telemetry_string, uint16_t str_len, char filename[]) {
    UINT bytesWritten;
    FRESULT result;

    if(global_micro_sd_data.successfullyMounted == 1){
    	char filepath[32];
    	snprintf(filepath, sizeof(filepath), "%s%s", USERPath, filename);
		result = f_open(&global_micro_sd_data.Fil, filepath, FA_WRITE | FA_OPEN_ALWAYS);
		if (result != FR_OK) {
			return 1;
		}

		result = f_lseek(&global_micro_sd_data.Fil, f_size(&global_micro_sd_data.Fil)); // move to end of file
		if (result != FR_OK) {
			f_close(&global_micro_sd_data.Fil);
			return 2;
		}

		result = f_write(&global_micro_sd_data.Fil, telemetry_string, str_len, &bytesWritten);
		if (result != FR_OK || bytesWritten != str_len) {
			// bytesWritten != str_len means partial write — disk full?
			f_close(&global_micro_sd_data.Fil);
			return 3;
		}
		// Need to add a new line in order to indicate the next packet to the user.
		f_write(&global_micro_sd_data.Fil, "\n", 1, &bytesWritten);

		result = f_sync(&global_micro_sd_data.Fil);
		if (result != FR_OK) {
			return 4;
		}

		result = f_close(&global_micro_sd_data.Fil);
		if (result != FR_OK) {
			// f_close flushes the final sector — if this fails, data is lost
			return 5;
		}
    }
    else {
    	HAL_GPIO_TogglePin(DEBUG_2_GPIO_Port, DEBUG_2_Pin);
    }
}
