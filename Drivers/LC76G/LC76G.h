#ifndef _LC76G_H_
#define _LC76G_H_

#include "stm32g4xx_hal.h"
#include "uart_interrupt.h"

#define TIMEOUT 5
#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

// Assuming a very long GPS buffer
#define GPS_DMA_BUFFER_SIZE 512

/* Define GPS commands */
// Checksums calculated using: https://nmeachecksum.eqth.net/

// PAIR messages - 2.4.14 PAIR062: PAIR_COMMON_SET_NMEA_OUTPUT_RATE
//								"GNSS Protocol Guide LC76G"
static const char LC76_ENABLE_GGA[] = "$PAIR062,0,1*3F\r\n";
static const char LC76_DISABLE_GGL[] = "$PAIR062,1,0*3F\r\n";
static const char LC76_DISABLE_GSA[] = "$PAIR062,2,0*3C\r\n";
static const char LC76_DISABLE_GSV[] = "$PAIR062,3,0*3D\r\n";
static const char LC76_DISABLE_RMC[] = "$PAIR062,4,0*3A\r\n";
static const char LC76_DISABLE_VTG8[] = "$PAIR062,5,0*3B\r\n";

// Baud Rate - 2.4.69. PAIR864: PAIR_IO_SET_BAUDRATE (Use if 115200 bps is not used)
//					"GNSS Protocol Guide LC76G"
static const char LC76_BAUDRATE[] = "$PAIR864,0,0,115200*1B\r\n";


// We need: time, lat, lon, alt, numberOfSats
// Time format: HH:MM:SS
// Degrees in decimal degrees
// Altitude in meters above sea level
typedef struct {
    uint8_t time_H;         // UTC Time
    uint8_t time_M;
    uint8_t time_S;

    double lat;
    double lon;

    double altitude;

    uint8_t num_sat_used;
}LC76G_gps_data;
extern LC76G_gps_data gps_data;

/* Define functions */
void LC76G_init();

#endif /* _LC76G_H_ */
