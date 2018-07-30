#ifndef INC_BMP180_H_
#define INC_BMP180_H_

#include "math.h"
#include "RegisterMap.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "bsp_iic.h"
#include "cmsis_os.h"

#define BMP180_I2CADDR			0x77
#define BMP180_CHANNELS			2
#define BMP180_ZEROALT_ITERs	4

typedef enum
{
	BMP180_CHANNEL1	= 0x00,
	BMP180_CHANNEL2 = 0x01
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
	uint32_t			ulZeroPressure;
	int16_t				UT;			//uncompensation temp
	int32_t				UP;			//uncompensation pressure
	uint8_t				oss;
	uint8_t				uZeroAltCounter;
}tBMP180Sensor;

void		BMP180_Init(void);
void 		BMP180_CalibrateData(eBMP180Channel ch);
void		BMP180_StartMeasure(void);
uint32_t	BMP180_GetPressure(eBMP180Channel ch);
float		BMP180_GetAltitude(eBMP180Channel ch);

#endif
