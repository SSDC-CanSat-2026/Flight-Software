/*
 * LIV3F.h
 *
 *  Created on: Jan 12, 2026
 *      Author: Joel Kubinsky
 */

/*
 * Teseo-LIV3F UART communication parameters:
 * • 8 data bits
 * • No parity
 * • 1 stop bit
 * • 9600 baud
 *
 * In both directions, communication is based on the frames described in next sections.
 * From Teseo Module receiver to Host frames can be:
 * • Unsolicited: For instance, periodical frame reporting position
 * • Data Responses: Teseo Module Receiver returns data requested by Host
 * • ACK: in case no data need to be returned to Host (e.g. on a reset request), simple ACK is sent
 * • NACK: if request contains wrong parameters, NACK is returned to Host.
 * From Host to Teseo Module receiver frames can be:
 * • Read Requests;
 * • Write reset, initialization Requests
 *
 * NMEA messages (GGA, GLL, etc) managed using "NMEA message list bitmask" (64-bit register).
 * Programmed using $PSTMNMEAREQUEST (10.2.38 UM2229).
 * Full 64-bit map is defined in Table 208 in section 12.14 of the software manual (UM2229).
 * The only bits I can see being of use to use here are:
 * Low 32 bits:
 * • Bit 1	(0x2) 		$GPGGA 		Message
 * • Bit 27 (0x800000)	$PSTMKFCOV 	Message   (Possibly)
 * High 32 bits
 * • Bit 32 (0x1)		$PSTMPV		Message	  (Possibly)
 * • Bit 33 (0x2)		$PSTMPVQ	Message   (If bit 32 is used)
 *
 * BAUD rates are configured using $PSTMCFGPORT command.
 *  Default value of 9600
 *
 * The NMEA Checksum is the bitwise XOR of the ASCII codes of all characters between
 *  the '$' and '*', not inclusive.
 */

#include <stm32g491xx.h>
#include "stm32g4xx_hal.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_GGA_FIELDS 20
#define MAX_RMC_FIELDS 15

// We may or may not need another struct for GNC purposes.

typedef struct
{
	uint32_t time_ms;   // milliseconds since midnight
	char gps_time[9];
    uint8_t fix_quality;
    uint8_t num_satellites;
    float hdop;
    float latitude;     // decimal degrees
    float longitude;    // decimal degrees
    float altitude;     // meters
}GGA_Data_t;

typedef struct
{
    // Information pulled from LIV3F software manual section 11.4.5 "$--RMC"
    uint32_t time_ms; // milliseconds since midnight
    char gps_time[9];
    char status;
    // Direction for Lat and Long is assumed North and West as we
    //  never leave the US.
    float latitude;  
    float longitude;
    float speed; // Speed in knots
    float track_good;
    char date [6]; // ddmmyy
    float mag_var;
    char mag_var_dir;
    char mode;
    char nav_status;


    uint8_t year;
    uint8_t month;
    uint8_t day;
}RMC_Data_t;


//extern GGA_Data_t gps_data;
//extern RMC_Data_t rmc_data;

void teseo_INIT(UART_HandleTypeDef* huart);
void cold_start(UART_HandleTypeDef* huart);

int parse_gps_buffer(char *sentence, GGA_Data_t* gga_out, RMC_Data_t* rmc_out);
int parse_gga(char *sentence, GGA_Data_t* out);
int parse_rmc(char *sentence, RMC_Data_t* out);

// Just copy the below functions from the LC76(G) driver
// Specifically from the modified 2025 FSW code.

/* Helper functions */
//double convert_to_double(char string_double[]);
//uint8_t convert_to_integer(char string_int[]);
void time_to_string(uint32_t time_ms, char *out); // This is also used for MISSION_TIME

/* Private helpers */
static uint32_t parse_gps_str_time_ms(const char *s);
static float nmea_to_decimal(char *coord, char dir);
static float fast_atof(const char *s);
