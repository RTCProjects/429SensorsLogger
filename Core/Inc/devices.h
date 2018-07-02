#ifndef _DEVICES_H
#define _DEVICES_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "bsp_canbus.h"

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
	uint16_t	msgCounter;
	float		*pRxData;
}tDistArrayData;

typedef struct
{
	uint16_t	ulRangefinder[4];
	uint16_t	ulRadar;
}tSensorLowData;

void 	Devices_Init(void);
void	Devices_PackageAnalysis(TQueryCanRxData	*rxData);
void	Devices_SensorsDataRequest(void);
#endif
