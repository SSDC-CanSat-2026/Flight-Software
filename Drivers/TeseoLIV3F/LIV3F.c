/*
 * LIV3F.c
 *
 *  Created on: Jan 12, 2026
 *      Author: Joel Kubinsky
 */

#include "LIV3F.h"

void teseo_INIT(UART_HandleTypeDef* huart) {
	/*
	 * Low 32 bits:
	 * • Bit 1	(0x2) 		$GPGGA 		Message   (Always)
	 * • Bit 27 (0x800000)	$PSTMKFCOV 	Message   (Possibly)
	 * High 32 bits
	 * • Bit 32 (0x1)		$PSTMPV		Message	  (Possibly)
	 * • Bit 33 (0x2)		$PSTMPVQ	Message   (If bit 32 is used)
	 */

	const char* cmds[] = {
		"$PSTMCFGMSGL,0,0,00000000,00000000*4D\r\n",
		"$PSTMCFGMSGL,1,0,00000000,00000000*4C\r\n",
		"$PSTMCFGMSGL,2,0,00000000,00000000*4F\r\n",
		"$PSTMCFGMSGL,0,1,00000002,00000000*4E\r\n"
//	        "$PSTMSAVEPAR*5A\r\n"
	};

	for (int i = 0; i < 4; i++) {
		HAL_UART_Transmit(huart, (uint8_t*)cmds[i], strlen(cmds[i]), HAL_MAX_DELAY);
		HAL_Delay(100);
	}
    // Just Bit 1
//	char bitmask_clear[] = "$PSTMCFGMSGL,0,0,FFFFFFFD,FFFFFFFF*4F\r\n";
//	char bitmask_clear[] = "$PSTMCFGMSGL,0*A3\r\n";
//	char bitmask_clear[] = "$HELLO*00\r\n";
//	HAL_UART_Transmit(huart, bitmask_clear, strlen(bitmask_clear), HAL_MAX_DELAY);
//	HAL_Delay(1000);
//    char bitmask_set[] = "$PSTMCFGMSGL,0,1,00000002,00000000*4E\r\n";
//    char bitmask_set[] = "$PSTMCFGMSGL,0,0,FFFFFFFD,FFFFFFFF*4F\r\n";
//	char bitmask_set[] = "$PSTMCFGMSGL,0,10,00000000,00000000*7C\r\n";
//	HAL_UART_Transmit(huart, bitmask_set, strlen(bitmask_set), HAL_MAX_DELAY);
//	char receive[100];
//	HAL_UART_Receive(huart, &receive, 100, 2000000);

	// This is for the use of bit 27.
	/*
    char bitmask[] = "$PSTMCFGMSGL,0,1,08000002,00000000*46\r\n";
	HAL_UART_Transmit(huart, bitmask, sizeof(bitmask), HAL_MAX_DELAY);
    */

	// This is for the use of bits 27, 32, and 33.
	/*
    char bitmask[] = "$PSTMCFGMSGL,0,1,08000002,00000003*45\r\n"
	HAL_UART_Transmit(huart, bitmask, sizeof(bitmask), HAL_MAX_DELAY);
    */


//	char get_bitmask[] = "PSTMNMEAREQUEST,<msglist_l>,<msglist_h>*<checksum><cr><lf>"

}

void cold_start(UART_HandleTypeDef* huart) {
	// $PSTMCOLD to trigger a cold start

	char cold[] = "$PSTMCOLD,,*1E\r\n";
	HAL_UART_Transmit(huart, cold, sizeof(cold), HAL_MAX_DELAY);
}

int parse_gga(char *sentence, GGA_Data_t *out)
{
	/*
	The init() sets the GGA message set to be the only one used.
	The protocol specification can be found at this link
	https://quectel.com/content/uploads/2024/02/Quectel_LC26GABLC76GLC86G_Series_GNSS_Protocol_Specification_V1.1.pdf

	$<TalkerID>GGA,<UTC>,<Lat>,<N/S>,<Lon>,<E/W>,<Quality>,<NumSatUsed>,<HDOP>,<Alt>,M,<Sep>,M,<DiffAge>,<DiffStation>*<Checksum><CR><LF>
	Example: $GNGGA,040143.000,3149.334166,N,11706.941670,E,2,36,0.48,61.496,M,-0.335,M,,*58 (DiffAge and DiffStation not supported)

	TalkerID 			- 2 Characters
	GGA 					- 3 Characters
	<UTC> (hhmmss.sss) 	- 10 Characters (start at 7 character offset)
	<Lat> (ddmm.mmmmmm) 	- 11 Characters (start at 18 character offset)
	<N/S> 				- 1 Character
	<Long> (ddmm.mmmmmm) - 11 Characters (start at 32 character offset)
	<E/W> 				- 1 Character
	<Quality> 			- Numeric, 1 Digit  (start at 47 character offset)
	<NumSatUsed> 		- Numeric, 2 Digits (start at 49 character offset)
	<HDOP> 				- Numeric
	<Alt> 				- Numeric
	'M' 					- <Alt> unit
	<Sep> 				- Numeric
	'M' 					- <Sep> unit
	<DiffAge> and <DiffStation> are not supported
	<Checksum> 			- Hexadecimal, starts with '*'
	<CR><LF>				- 2 Characters
	*/

    if (strncmp(sentence, "$GPGGA", 6) != 0)
        return 0;

    char *fields[MAX_GGA_FIELDS] = {0};
    int field_count = 0;

    char *p = sentence;
    fields[field_count++] = p;

    while (*p && field_count < MAX_GGA_FIELDS)
    {
        if (*p == ',' || *p == '*')
        {
            *p = '\0';
            fields[field_count++] = p + 1;
        }
        p++;
    }

    if (field_count < 10)
        return 0;

    out->time_ms = parse_gps_str_time_ms(fields[1]);
    out->fix_quality = (fields[6][0]) ? atoi(fields[6]) : 0;
    out->num_satellites  = (fields[7][0]) ? atoi(fields[7]) : 0;
    out->hdop     = (fields[8][0]) ? fast_atof(fields[8]) : 0.0f;
    out->altitude = (fields[9][0]) ? fast_atof(fields[9]) : 0.0f;
    out->latitude = nmea_to_decimal(fields[2], fields[3][0]);
    out->longitude = nmea_to_decimal(fields[4], fields[5][0]);

    return 1;
}

/* --------------------------------------------------------- HELPER FUNCTIONS --------------------------------------------------------- */

// Used in the ReadSensors thread to convert from the millisecond form
// that the GPS time is stored in and converts it to the HH:MM:SS format
// needed for transmission to the GCS.
void time_to_string(uint32_t time_ms, char *out) {
	// For reference "UL" just stands for Unsigned Long.
	// This forces the code to use a 32-bit unsigned int
	//    for converting the time.
    uint32_t total_seconds = time_ms / 1000UL;

    uint32_t hours   = total_seconds / 3600UL;
    uint32_t minutes = (total_seconds % 3600UL) / 60UL;
    uint32_t seconds = total_seconds % 60UL;

    out[0] = '0' + (hours   / 10);
    out[1] = '0' + (hours   % 10);
    out[2] = ':';
    out[3] = '0' + (minutes / 10);
    out[4] = '0' + (minutes % 10);
    out[5] = ':';
    out[6] = '0' + (seconds / 10);
    out[7] = '0' + (seconds % 10);
}


/* Private helpers */

static uint32_t parse_gps_str_time_ms(const char *s) {
    if (!s || s[0] == '\0')
        return 0;

    // hh
    uint32_t hours =
        (s[0] - '0') * 10 +
        (s[1] - '0');

    // mm
    uint32_t minutes =
        (s[2] - '0') * 10 +
        (s[3] - '0');

    // ss
    uint32_t seconds =
        (s[4] - '0') * 10 +
        (s[5] - '0');

    uint32_t ms = 0;

    if (s[6] == '.')
    {
        // sss (can be fewer digits — handle safely)
        if (s[7] >= '0' && s[7] <= '9') ms += (s[7] - '0') * 100;
        if (s[8] >= '0' && s[8] <= '9') ms += (s[8] - '0') * 10;
        if (s[9] >= '0' && s[9] <= '9') ms += (s[9] - '0');
    }

    return
        hours   * 3600000UL +
        minutes * 60000UL +
        seconds * 1000UL +
        ms;
}

static float nmea_to_decimal(char *coord, char dir) {
    if (coord == NULL || coord[0] == '\0')
        return 0.0f;

    float raw = atof(coord);

    int degrees = (int)(raw / 100);
    float minutes = raw - (degrees * 100);

    float decimal = degrees + (minutes / 60.0f);

    if (dir == 'S' || dir == 'W')
        decimal *= -1.0f;

    return decimal;
}

static float fast_atof(const char *s) {
    if (!s || *s == '\0')
        return 0.0f;

    float result = 0.0f;
    float fraction = 0.1f;
    int negative = 0;

    if (*s == '-')
    {
        negative = 1;
        s++;
    }

    // Integer part
    while (*s >= '0' && *s <= '9')
    {
        result = result * 10.0f + (*s - '0');
        s++;
    }

    // Fractional part
    if (*s == '.')
    {
        s++;
        while (*s >= '0' && *s <= '9')
        {
            result += (*s - '0') * fraction;
            fraction *= 0.1f;
            s++;
        }
    }

    return negative ? -result : result;
}



void send_getrtc(UART_HandleTypeDef* huart) {
	char cmd[] = "PSTMDATUMSELECT,0*DB\r\n";
	HAL_UART_Transmit(huart, cmd, sizeof(cmd), HAL_MAX_DELAY);
}
