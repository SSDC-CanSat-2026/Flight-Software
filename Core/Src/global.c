/*
 * global.c
 *
 *  Created on: Sep 15, 2025
 *      Author: Sarah Tran
 */

#include "global.h"

Mission_Data global_mission_data = {0};

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
