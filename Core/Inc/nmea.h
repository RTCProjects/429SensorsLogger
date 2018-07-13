#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

#define NMEA_POS_SIZE	26

typedef struct
{
	float	fLatitude;
	float	fLongitude;
	char	strPosition[NMEA_POS_SIZE];
}tNMEAPosition;

uint8_t	NMEA_Parse(uint8_t *inputStr,uint8_t	size);
char	*NMEA_GetPositionString(void);

#endif
