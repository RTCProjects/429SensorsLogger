#ifndef _DEVICES_H
#define _DEVICES_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

typedef struct
{
	uint16_t	ulRadarDistance;
	uint16_t	ulCenterLidarDistance;
	uint16_t	ulLeftLidarDistance;
	uint16_t	ulRightLidarDistance;
	uint16_t	ulFrontLidarDistance;
	uint16_t	ulSonarDistance;
}tSensors;

void 	Devices_Init(void);
void	Devices_LedToggle(void);
void	Devices_LedOff(void);
void	Devices_LedOn(void);
void	Devices_IMUOn(void);
void	Devices_IMUOff(void);
tSensors	*Devices_GetDataPointer(void);

#endif
