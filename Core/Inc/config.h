/*
 * config.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Joel
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "stm32g4xx_hal.h"
#include "string.h"
#include "ff.h"
#include "app_fatfs.h"
#include "global.h"

typedef struct {
	char 	 MISSION_TIME[9];
	uint32_t PACKET_COUNT;
	char 	 MODE;
	char 	 STATE[STATE_TEXT_LEN];
	float 	 ALTITUDE_OFFSET;
    float    baro_ground_pressure;   // Pa — calibration baseline
    float    baro_ground_altitude;   // m
    float    accel_offset_x;
    float    accel_offset_y;
    float    accel_offset_z;
    uint32_t checksum;               // simple validation
} SystemConfig_t;

extern SystemConfig_t global_config;

int32_t load_config_from_sd(void);
int32_t save_config_to_sd(void);
int32_t set_default_config(void);

#endif /* INC_CONFIG_H_ */
