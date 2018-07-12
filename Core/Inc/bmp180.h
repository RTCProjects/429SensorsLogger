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


void		BMP180_Init(void);
void 		BMP180_PressureAltitude(uint32_t *Pressure,float *Altitude);
uint16_t	BMP180_ReadRawTemperature(void);
uint16_t	BMP180_ReadRawPresure(void);
int32_t		BMP180_ReadPresusure(void);
void		BMP180_StartMeasure(void);
uint32_t	BMP180_GetPressure(void);
float		BMP180_GetAltitude(void);

#endif
