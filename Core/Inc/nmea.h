#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

uint8_t	NMEA_Parse(uint8_t *inputStr,uint8_t	size);

#endif
