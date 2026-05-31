/*
 * global.c
 *
 *  Created on: Sep 15, 2025
 *      Author: Sarah Tran
 */

#include "global.h"

// TODO: make this false before any demonstrations
volatile uint8_t telemetry_enable 		= 1;
volatile uint8_t gps_time_enable 		= 0;
volatile uint8_t is_calibrated 			= 0;
volatile uint8_t mec_wire_enable 		= 0;
volatile uint8_t simulation_enable 		= 0;
volatile uint8_t simulation_pre 		= 0;
volatile float simulated_pressure 		= 0.0;
volatile uint8_t calibrating 			= 0;
volatile uint8_t cal_count 				= 0;
volatile float cal_sum 					= 0;
Mission_Data 	global_mission_data 	= {0};
Flags global_flags = {0};
Micro_SD_Data	global_micro_sd_data;

void init_mission_data(void)
{
	memset(&global_mission_data, 0, sizeof(global_mission_data));

	global_mission_data.TEAM_ID = 1075;
	global_mission_data.MISSION_TIME_ms = 0;
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

	global_mission_data.ACCEL_X = 0;
	global_mission_data.ACCEL_YAW = 0;
	global_mission_data.ACCEL_Z = 0;

	strcpy(global_mission_data.GPS_TIME, "XX:XX:XX");
	global_mission_data.GPS_ALTITUDE = 0.0;
	global_mission_data.GPS_LATITUDE = 0.0;
	global_mission_data.GPS_LONGITUDE = 0.0;
	global_mission_data.GPS_SATS = 0;

	strcpy(global_mission_data.CMD_ECHO, "CMD");

	global_mission_data.ALTITUDE_OFFSET = 0.0;
}


