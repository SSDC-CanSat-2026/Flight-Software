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
	};

	for (int i = 0; i < 4; i++) {
		HAL_UART_Transmit(huart, (uint8_t*)cmds[i], strlen(cmds[i]), HAL_MAX_DELAY);
		HAL_Delay(100);
	}

}

void cold_start(UART_HandleTypeDef* huart) {
	// $PSTMCOLD to trigger a cold start

	char cold[] = "$PSTMCOLD,,*1E\r\n";
	HAL_UART_Transmit(huart, cold, sizeof(cold), HAL_MAX_DELAY);
}


// Because there is now more than one message, this function acts as a wrapper
//  that will determine which parser needs to be called first.
// Return 1 for GGA and return 2 for RMC
int parse_gps_buffer(char *sentence, GGA_Data_t* gga_out, RMC_Data_t* rmc_out) {
	for (uint8_t i = 0; i < 50; i++) {
		if (sentence[i] == '$') {
			sentence += i;
			break;
		}
	}

    if (strncmp(sentence, "$GPGGA", 6) == 0) {
        return parse_gga(sentence, gga_out);
    } else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        return parse_rmc(sentence, rmc_out);
    } else {
        return -1;
    }
}

int parse_gga(char *sentence, GGA_Data_t *out)
{
	/*
	The init() sets the GGA message to be enabled.1
	The protocol specification can be found at this link
	https://receiverhelp.trimble.com/alloy-gnss/en-us/NMEA-0183messages_GGA.html
    Or in the software manual under section 11.4.1

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

    out->time_ms        = parse_gps_str_time_ms(fields[1]);
    out->fix_quality    = (fields[6][0]) ? atoi(fields[6]) : 0;
    out->num_satellites = (fields[7][0]) ? atoi(fields[7]) : 0;
    out->hdop           = (fields[8][0]) ? fast_atof(fields[8]) : 0.0f;
    out->altitude       = (fields[9][0]) ? fast_atof(fields[9]) : 0.0f;
    out->latitude       = nmea_to_decimal(fields[2], fields[3][0]);
    out->longitude      = nmea_to_decimal(fields[4], fields[5][0]);

    return 1;
}

int parse_rmc(char *sentence, RMC_Data_t* out) {
    /*
	The init() sets the GGA message to be enabled.1
	The protocol specification can be found at this link
	https://receiverhelp.trimble.com/alloy-gnss/en-us/nmea0183-messages-rmc.html
    Or in the software manual under section 11.4.5

	$<TalkerID>RMC,<UTC>,<Status>,<Lat>,<N/S>,<Long>,<E/W>,<Speed>,<Trackgood>,<Date>,<MagVar>,<MagVarDir>,<Mode>,<Nav_status>,<Checksum><CR><LF>
	Example: $GPRMC,183417.000,V,4814.040,N,01128.522,E,0.0,0.0,170907,0.0,W*6C

	TalkerID 			- 2 Characters
	RMC					- 3 Characters
	<UTC> (hhmmss.sss) 	- 10 Characters
    <Satus>             - 1 Character
	<Lat> (ddmm.mmmmmm) - 11 Characters     
	<N/S> 				- 1 Character       
	<Long> (ddmm.mmmmmm)- 11 Characters     
	<E/W> 				- 1 Character       
    <Speed> (ddd.d)     - Numeric, 4 digits 
	<Trackgood> (ddd.d) - Decimal, 4 digits
    <Date> (ddmmyy)     - Deciaml, 6 digits
    <MagVar> (ddd.d)    - Deciaml, 4 digits
    <MagVarDir>         - 1 Character
    <Mode>              - 1 Character
    <Nav_status>        - 1 Character
	<Checksum> 			- Hexadecimal, starts with '*'
	<CR><LF>		    - 2 Characters
	*/


    if (strncmp(sentence, "$GPRMC", 6) != 0 && strncmp(sentence, "$GNRMV", 6) != 0)
        return 0;

    char *fields[MAX_RMC_FIELDS] = {0};
    int field_count = 0;

    char *p = sentence;
    fields[field_count++] = p;

    while (*p && field_count < MAX_RMC_FIELDS)
    {
        if (*p == ',' || *p == '*')
        {
            *p = '\0';
            fields[field_count++] = p + 1;
        }
        p++;
    }

    if (field_count < 14)
        return 0;

    out->time_ms        = parse_gps_str_time_ms(fields[1]);
//    out->status         = fields[2];
    out->latitude       = nmea_to_decimal(fields[3], fields[4][0]);
    out->longitude      = nmea_to_decimal(fields[5], fields[6][0]);
    out->speed          = (fields[7][0]) ? fast_atof(fields[7]) : 0.0f;
    out->track_good     = (fields[8][0]) ? fast_atof(fields[8]) : 0.0f;
    out->mag_var        = (fields[10][0]) ? fast_atof(fields[10]) : 0.0f;
//    out->mag_var_dir    = fields[11];
//    out->mode           = fields[12];
//    out->nav_status     = fields[13];

    strncpy(out->date, fields[9], 6);

    return 2;
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
