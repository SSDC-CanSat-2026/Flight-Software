/*
 * global.h
 *
 *  Created on: Sep 15, 2025
 *      Author: Sarah Tran
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

#include "stm32g4xx_hal.h"
#include "string.h"
#include "ff.h"
#include "app_fatfs.h"

#define STATE_TEXT_LEN 14 // 13 max, plus 1 for null char
#define CMD_ECHO_LEN 10
#define CMD_BUFFER_LEN 22 //21 max, plus 1 for null

// flags
extern volatile uint8_t telemetry_enable;
extern volatile uint8_t simulation_enable;
extern volatile uint8_t gps_time_enable;
extern volatile uint8_t is_calibrated;
extern volatile uint8_t mec_wire_enable;
extern volatile uint8_t simulation_pre;
extern volatile double simulated_pressure;

extern volatile uint8_t calibrating;
extern volatile uint8_t cal_count;
extern volatile float 	cal_sum;

// struct
/* WATCH FOR RACE CONDITIONS */
typedef struct
{
	int16_t TEAM_ID;
	char MISSION_TIME[9]; // "hh:mm:ss"
	uint32_t PACKET_COUNT;

	char MODE;
	char STATE[STATE_TEXT_LEN];

	float ALTITUDE;
	float TEMPERATURE;
	float PRESSURE;

	float VOLTAGE;
	float CURRENT;

	float GYRO_R;
	float GYRO_P;
	float GYRO_Y;

	float ACCEL_X;
	float ACCEL_Y;
	float ACCEL_Z;

	char GPS_TIME[9];
	float GPS_ALTITUDE;
	float GPS_LATITUDE;
	float GPS_LONGITUDE;
	uint8_t GPS_SATS;

	char CMD_ECHO[CMD_ECHO_LEN];

	float ALTITUDE_OFFSET;
} Mission_Data;

typedef struct
{
	int successfullyMounted;
	FATFS FatFs;
	FIL Fil;
} Micro_SD_Data;

extern Mission_Data 	global_mission_data;
extern Micro_SD_Data 	global_micro_sd_data;

void init_mission_data(void);

#endif /* INC_GLOBAL_H_ */
