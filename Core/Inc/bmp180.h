#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "math.h"
#include "RegisterMap.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bsp_iic.h"
#include "cmsis_os.h"

#define BMP180_I2CADDR	0x77

typedef enum
{
	BMP180_CHANNEL1	= 0x01,
	BMP180_CHANNEL2 = 0x02
}eBMP180Channel;

typedef struct
{
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t	MC;
	int16_t	MD;
}tBMP180Calibrate;

typedef struct
{
	tBMP180Calibrate	cData;		//calibration data
	uint32_t			ulPressure;	//pressure in PA
	float				fAltitude;	//altitude
	int16_t				UT;			//uncompensation temp
	int32_t				UP;			//uncompensation pressure
}tBMP180Sensor;

void		BMP180_Init(void);
void 		BMP180_PressureAltitude(uint32_t *Pressure,float *Altitude);
uint16_t	BMP180_ReadRawTemperature(void);
uint16_t	BMP180_ReadRawPresure(void);
int32_t		BMP180_ReadPresusure(void);
void		BMP180_StartMeasure(void);
uint32_t	BMP180_GetPressure(void);
float		BMP180_GetAltitude(void);

uint32_t	BMP180_GetPressure2(void);
float		BMP180_GetAltitude2(void);

#endif
