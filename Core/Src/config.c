/*
 * config.c
 *
 *  Created on: Apr 9, 2026
 *      Author: Joel
 */

#include "../Inc/config.h"
#include "../Inc/microSD.h"

SystemConfig_t global_config = {
	.MISSION_TIME 			 = "00:00:00",
	.PACKET_COUNT 			 = 0,
	.MODE					 = 'F',
	.STATE					 = "LAUNCH_PAD",
	.ALTITUDE_OFFSET		 = 0,
    .baro_ground_pressure    = 101325.0f,  // standard sea level Pa
    .baro_ground_altitude    = 0.0f,
    .accel_offset_x          = 0.0f,
    .accel_offset_y          = 0.0f,
    .accel_offset_z          = 0.0f,
    .checksum                = 0
};

static uint32_t compute_checksum(SystemConfig_t *cfg)
{
    uint32_t sum = 0;
    uint8_t *p = (uint8_t *)cfg;
    // Checksum everything except the checksum field itself
    for (size_t i = 0; i < offsetof(SystemConfig_t, checksum); i++)
        sum += p[i];
    return sum;
}

int32_t load_config_from_sd(void) {
    static FIL fil;
    static UINT bytesRead;
    FRESULT res;
    SystemConfig_t temp_cfg;

    char filepath[16];
    snprintf(filepath, sizeof(filepath), "%sconfig.bin", USERPath);

    if (f_open(&fil, filepath, FA_READ) != FR_OK)
        return APP_ERROR;  // file doesn't exist yet

    res = f_read(&fil, &temp_cfg, sizeof(SystemConfig_t), &bytesRead);
    f_close(&fil);

    if (res != FR_OK || bytesRead != sizeof(SystemConfig_t))
        return APP_ERROR;

    // Validate checksum before trusting the data
    if (compute_checksum(&temp_cfg) != temp_cfg.checksum)
        return APP_ERROR;  // data corrupt

    // Safe to apply
    memcpy(&global_config, &temp_cfg, sizeof(SystemConfig_t));
    return APP_OK;
}

int32_t save_config_to_sd(void) {
    static FIL fil;
    static UINT bytesWritten;

    global_config.checksum = compute_checksum(&global_config);

    char filepath[16];
    snprintf(filepath, sizeof(filepath), "%sconfig.bin", USERPath);

    if (f_open(&fil, filepath, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return APP_ERROR;

    FRESULT res = f_write(&fil, &global_config, sizeof(SystemConfig_t), &bytesWritten);
    f_sync(&fil);
    f_close(&fil);

    if (res != FR_OK || bytesWritten != sizeof(SystemConfig_t))
        return APP_ERROR;

    return APP_OK;
}

int32_t set_default_config(void) {
	global_config.ALTITUDE_OFFSET 		= 0.0;
	global_config.MODE					= 'F';
	global_config.PACKET_COUNT			= 0;
	global_config.accel_offset_x		= 0.0;
	global_config.accel_offset_y		= 0.0;
	global_config.accel_offset_z		= 0.0;
	global_config.baro_ground_altitude	= 0.0;
	global_config.baro_ground_pressure	= 101325.0f;
	global_config.checksum				= compute_checksum(&global_config);

	strcpy(global_config.MISSION_TIME, "00:00:00");
	strcpy(global_config.STATE, "LAUNCH_PAD");
}
