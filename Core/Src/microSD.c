/*
 * microSD.c
 *
 *  Created on: Apr 9, 2026
 *      Author: Joel
 */

#include "../Inc/microSD.h"
#include <stdio.h>
#include "stm32g4xx_it.h"

void check_fatfs_guards(void)
{
//    for (uint8_t i = 0; i < 4; i++)
//    {
//        if (guard_before[i] != 0xDEADBEEF)
//        {
//            // Something before USERFatFs was overwritten
//            __BKPT(0);
//        }
//        if (guard_after[i] != 0xDEADBEEF)
//        {
//            // Something after USERFatFs was overwritten
//            __BKPT(0);
//        }
//    }
}

void init_SD(void){

	if (f_mount(&USERFatFs, USERPath, 1) != FR_OK) {
		global_micro_sd_data.successfullyMounted = 0;
		

//		HardFault_Handler(); // FIXME : Obviously we don't want to trigger a hard fault here during competition.
		// TODO: ADD LED DEBUGGING LIGHTS HERE FOR LED
		return;
	}

	WORD sector_size;
	DRESULT res = disk_ioctl(0, GET_SECTOR_SIZE, &sector_size);

	global_micro_sd_data.successfullyMounted = 1;

  
  // FIXME : This header writing needs to be verified - Joel
	const char header_string[] = "TEAM_ID,MISSION_TIME,PACKET_COUNT,MODE,STATE,ALTITUDE,TEMPERATURE,PRESSURE,VOLTAGE,CURRENT,GYRO_R,GYRO_P,GYRO_Y,ACCEL_R,ACCEL_P,ACCEL_Y,GPS_TIME,GPS_ALTITUDE,GPS_LATITUDE,GPS_LONGITUDE,GPS_SATS,CMD_ECHO\n";

	// Check if file exists first
	FIL file;
	FRESULT fr = f_open(&file, "log26.csv", FA_READ);
	if (fr == FR_NO_FILE) {
	    // File doesn't exist, create and write header
	    f_close(&file);
	    write_SD(header_string, strlen(header_string), "log26.csv", FA_WRITE | FA_CREATE_ALWAYS);
	} else {
	    // File exists, just close it
	    f_close(&file);
	}

	check_fatfs_guards();
}

static FIL* get_fil_for_file(const char* filename)
{
    if (strstr(filename, "FSW") != NULL) return &fil_telemetry;
    if (strstr(filename, "debug")       != NULL) return &fil_debug;
    if (strstr(filename, "config")      != NULL) return &fil_config;
    return NULL;  // unknown file
}

void read_SD(char* buf, char filename[]) {

	UINT bytesRead;
	FRESULT result;

	if (global_micro_sd_data.successfullyMounted == 1) {

		char filepath[32];
		snprintf(filepath, sizeof(filepath), "%s%s", USERPath, filename);

	    FIL* fil = get_fil_for_file(filename);
	    if (fil == NULL) return;  // unknown filename

		result = f_open(fil, filepath, FA_READ);
		if (result != FR_OK) {
			return;
		}

		char output[100];
		result = f_read(fil, &output, f_size(fil), &bytesRead);
		if (result != FR_OK || bytesRead == 0) {
			return;
		}

		// Format of Config file (.bin file)

	}
}

uint32_t write_SD(char* telemetry_string, uint16_t str_len, char filename[], uint8_t FLAGS)
{
    uint32_t ret = FR_OK;
    UINT bytesWritten;
    FRESULT result;
    uint8_t fileIsOpen = 0;

    if (global_micro_sd_data.successfullyMounted != 1)
        return 6;

    FIL* fil = get_fil_for_file(filename);
    if (fil == NULL) return 7;  // unknown filename

//    memset(fil, 0, sizeof(FIL));  // ← add this line

    char filepath[64] = {0};
    snprintf(filepath, sizeof(filepath), "0:/%s", filename);

    check_fatfs_guards();
    result = f_open(fil, filepath, FA_WRITE | FA_OPEN_ALWAYS);
//    result = f_open(fil, "0:/test.txt", FA_WRITE | FA_OPEN_ALWAYS);
//    result = f_open(fil, filepath, FLAGS);
    if (result != FR_OK) return 1;
    fileIsOpen = 1;

    if (f_size(fil) == 0)
    {
        result = f_write(fil, header_string, strlen(header_string), &bytesWritten);
        if (result != FR_OK || bytesWritten != strlen(header_string))
        {
            ret = 3;
            goto cleanup;
        }
    }

    result = f_lseek(fil, f_size(fil));
    if (result != FR_OK) { ret = 2; goto cleanup; }

    // Place this just before your f_write call
    volatile void* dbg_fil_addr     = (void*)fil;  // or whichever FIL you're using
    volatile void* dbg_fatfs_addr   = (void*)&USERFatFs;
//    volatile void* dbg_guard_before = (void*)&guard_before[0];
//    volatile void* dbg_guard_after  = (void*)&guard_after[0];

    // Optionally compute the distances so you can read them directly in the watch window
//    volatile int32_t dbg_dist_fil_to_guard = (int32_t)((uint8_t*)&fil_telemetry - (uint8_t*)&guard_before[0]);
//    volatile int32_t dbg_dist_fatfs_to_guard = (int32_t)((uint8_t*)&USERFatFs - (uint8_t*)&guard_before[0]);

    result = f_write(fil, telemetry_string, str_len, &bytesWritten);

    check_fatfs_guards();

    if (result != FR_OK || bytesWritten != str_len) { ret = 3; goto cleanup; }

    check_fatfs_guards();

    result = f_write(fil, "\n", 1, &bytesWritten);
    if (result != FR_OK || bytesWritten != 1) { ret = 3; goto cleanup; }

    check_fatfs_guards();

    result = f_sync(fil);
    if (result != FR_OK) { ret = 4; goto cleanup; }

    // HAL_GPIO_TogglePin(DEBUG_1_GPIO_Port, DEBUG_1_Pin);

    result = f_lseek(fil, f_size(fil));
    if (result != FR_OK)
    {
        ret = 2;
        goto cleanup;
    }

cleanup:
    if (fileIsOpen)
    {
    	check_fatfs_guards();
        result = f_close(fil);
        check_fatfs_guards();
        if (result != FR_OK) ret = 5;
    }
    return ret;
}
