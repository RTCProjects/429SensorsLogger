#ifndef _DEVICES_H
#define _DEVICES_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#define RX_SENSOR	0x100
#define RX_ARRAY	0x110

#define DATA_ARRAY_SIZE	64

#define TYPE_1	0x01
#define TYPE_2	0x02

typedef union
{
	struct
	{
		uint8_t	LCORR:1;
		uint8_t	LSTAT:1;
		uint8_t	SCORR:1;
		uint8_t	SSTAT:1;
		uint8_t	RCORR:1;
		uint8_t	RSTAT:1;
		uint8_t	Reserved:2;
	}bit;
	uint8_t	flags;
}uSensorFlag;

typedef struct
{
	uint8_t			devType;
	uint8_t			devID;
	uint16_t		sensorValue[3];
	uSensorFlag	uFlags;
}tSensorData;

typedef struct
{
	uint16_t	ulLidarDistance;
	uint16_t	ulRadarDistance;
	uint16_t	ulSonarDistance;
}tSensors;

void 	Devices_Init(void);
void	Devices_SensorsDataRequest(void);
void	Devices_LedToggle(void);
void	Devices_LedOff(void);
void	Devices_LedOn(void);
void	Devices_IMUOn(void);
void	Devices_IMUOff(void);


#endif
