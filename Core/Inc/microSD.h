/*
 * microSD.h
 *
 *  Created on: Apr 9, 2026
 *      Author: Joel
 */

#ifndef INC_MICROSD_H_
#define INC_MICROSD_H_

#include "global.h"

void init_SD(void);
void read_SD(char* buf, char filename[]);
uint32_t write_SD(char* telemetry_string, uint16_t str_len, char filename[]);

#endif /* INC_MICROSD_H_ */
