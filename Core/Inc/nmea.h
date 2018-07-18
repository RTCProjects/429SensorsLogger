#ifndef INC_NMEA_H_
#define INC_NMEA_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

#define NMEA_POS_SIZE	27
#define NMEA_VEL_SIZE	26
#define NMEA_BUF_SIZE 	128			//размер GPS буфера

typedef struct
{
	float	fLatitude;
	float	fLongitude;
	float	fVelocity;
	char	strVelocity[NMEA_VEL_SIZE];
	char	strPosition[NMEA_POS_SIZE];

}tNMEAPosition;

void	NMEA_Parse();
char	*NMEA_GetPositionString(void);
char	*NMEA_GetVelocityString(void);
void	NMEA_RcvByteCallback(uint8_t inputByte);
uint8_t	*NMEA_GetGPSBuffer(void);
#endif
