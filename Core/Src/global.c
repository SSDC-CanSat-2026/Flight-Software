/*
 * global.c
 *
 *  Created on: Sep 15, 2025
 *      Author: Sarah Tran
 */

#include "global.h"

// TODO: make this false before any demonstrations
volatile uint8_t telemetry_enable = 0;
volatile uint8_t gps_time_enable = 0;
volatile uint8_t is_calibrated = 0;
volatile uint8_t mec_wire_enable = 0;
volatile uint8_t simulation_enable = 0;
volatile uint8_t simulation_pre = 0;
volatile double simulated_pressure = 0.0;

Mission_Data 	global_mission_data = {0};
Micro_SD_Data	global_micro_sd_data;

void init_mission_data(void)
{
	memset(&global_mission_data, 0, sizeof(global_mission_data));

	global_mission_data.TEAM_ID = 3701;
	strcpy(global_mission_data.MISSION_TIME, "XX:XX:XX");
	global_mission_data.PACKET_COUNT = 0;

	global_mission_data.MODE = 'F';
	strcpy(global_mission_data.STATE, "LAUNCH_PAD");

	global_mission_data.ALTITUDE = 0.0;
	global_mission_data.TEMPERATURE = 0.0;
	global_mission_data.PRESSURE = 0.0;

	global_mission_data.VOLTAGE = 0.0;
	global_mission_data.CURRENT = 0.0;

	global_mission_data.GYRO_R = 0;
	global_mission_data.GYRO_P = 0;
	global_mission_data.GYRO_Y = 0;

	global_mission_data.ACCEL_R = 0;
	global_mission_data.ACCEL_P = 0;
	global_mission_data.ACCEL_Y = 0;

	strcpy(global_mission_data.GPS_TIME, "XX:XX:XX");
	global_mission_data.GPS_ALTITUDE = 0.0;
	global_mission_data.GPS_LATITUDE = 0.0;
	global_mission_data.GPS_LONGITUDE = 0.0;
	global_mission_data.GPS_SATS = 0;

	strcpy(global_mission_data.CMD_ECHO, "CMD");

	global_mission_data.ALTITUDE_OFFSET = 0.0;
}

void init_SD(void){

//	if(f_mount(&global_micro_sd_data.FatFs, "", 1) != FR_OK){
	if (f_mount(&USERFatFs, USERPath, 1) != FR_OK) {
		global_micro_sd_data.successfullyMounted = 0;

		// TODO: ADD LED DEBUGGING LIGHTS HERE FOR LED
		return;
	}

	global_micro_sd_data.successfullyMounted = 1;;
}

void write_SD(char* telemetry_string, uint16_t str_len) {
    UINT bytesWritten;
    FRESULT result;

    if(global_micro_sd_data.successfullyMounted == 1){

    	char filepath[32];
    	snprintf(filepath, sizeof(filepath), "%sCanSat_Data_2026.csv", USERPath);
		result = f_open(&global_micro_sd_data.Fil, filepath, FA_WRITE | FA_OPEN_ALWAYS);
		if (result != FR_OK) {
			return;
		}

		result = f_lseek(&global_micro_sd_data.Fil, f_size(&global_micro_sd_data.Fil)); // move to end of file
		if (result != FR_OK) {
			f_close(&global_micro_sd_data.Fil);
			HAL_GPIO_TogglePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin);
			return;
		}

		result = f_write(&global_micro_sd_data.Fil, telemetry_string, str_len, &bytesWritten);
		if (result != FR_OK || bytesWritten != str_len) {
			HAL_GPIO_TogglePin(DEBUG_1_GPIO_Port, DEBUG_1_Pin);
			// bytesWritten != str_len means partial write — disk full?
			f_close(&global_micro_sd_data.Fil);
			return;
		}
		// Need to add a new line in order to indicate the next packet to the user.
		f_write(&global_micro_sd_data.Fil, "\n", 1, &bytesWritten);

		result = f_sync(&global_micro_sd_data.Fil);
		if (result != FR_OK) {
		    HAL_GPIO_TogglePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin);
		}

		result = f_close(&global_micro_sd_data.Fil);
		if (result != FR_OK) {
			// f_close flushes the final sector — if this fails, data is lost
			HAL_GPIO_TogglePin(DEBUG_2_GPIO_Port, DEBUG_2_Pin);
		}

		FILINFO fno;
		result = f_stat("CanSat_Data_2026.csv", &fno);
		// Check in debugger:
		// result == FR_OK means file exists
		// fno.fsize tells you how many bytes FatFS thinks are written
		// fno.fname confirms the filename on disk

		HAL_GPIO_TogglePin(DEBUG_2_GPIO_Port, DEBUG_2_Pin);
		HAL_GPIO_TogglePin(DEBUG_1_GPIO_Port, DEBUG_1_Pin);
		HAL_GPIO_TogglePin(DEBUG_0_GPIO_Port, DEBUG_0_Pin);
    }
    else {
    	HAL_GPIO_TogglePin(DEBUG_2_GPIO_Port, DEBUG_2_Pin);
    	HAL_GPIO_TogglePin(DEBUG_1_GPIO_Port, DEBUG_0_Pin);
    }
}
