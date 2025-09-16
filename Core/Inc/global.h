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

#define STATE_TEXT_LEN 14 // 13 max, plus 1 for null char
#define CMD_ECHO_LEN 10

// flags



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

	int16_t GYRO_R;
	int16_t GYRO_P;
	int16_t GYRO_Y;

	int16_t ACCEL_R;
	int16_t ACCEL_P;
	int16_t ACCEL_Y;

	char GPS_TIME[9];
	float GPS_ALTITUDE;
	float GPS_LATITUDE;
	float GPS_LONGITUDE;
	uint8_t GPS_SATS;

	char CMD_ECHO[CMD_ECHO_LEN];

	float ALTIDUDE_OFFSET;
} Mission_Data;

extern Mission_Data global_mission_data;

void init_mission_data(void);

#endif /* INC_GLOBAL_H_ */
