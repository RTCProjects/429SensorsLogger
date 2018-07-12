#ifndef _BSP_DRIVER_SPI_H
#define _BSP_DRIVER_SPI_H

#define SPI_OK			0x00
#define SPI_ERROR	 	0x01

#define TIMEOUT					100
#define SDCARD_WRITE_QUEUE_SIZE	16
#define SDCARD_FILENAME_MAX_LEN	12

#define DIRNAME_SENSORS			"Sensors"
#define DIRNAME_IMU				"IMU"

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "devices.h"
#include "IMU.h"

typedef struct sd_info{
	uint8_t	ocr1[4];
	uint8_t	ocr2[4];
  uint8_t type;
}sd_info_ptr;

typedef enum
{
	E_RANGEFINDER = 0x01,
	E_GYRO		  = 0x02
}eSensorType;

typedef struct
{
	eSensorType	type;
	tSensors	sensorsData;
	tIMUData	imuData;
	float		fAltitude;
	float		fAltitude2;
	float		fLatitude;
	float		fLongitude;
}tSDCardWriteData;


typedef struct
{
	char	*sensorsFilename;
	char	*imusensFilename;
}tSDCardFileNames;

void		BSP_SDCard_Init(void);
void		BSP_SDCard_WriteSensorsData(tSDCardWriteData	*Data);
uint16_t	BSP_SDCard_GetFileNumbers(char*);
void 		BSP_SDCard_SPIInit(uint32_t baudratePrescaler);
void 		BSP_SDCardSPIDeInit(void);
void		BSP_SDCard_StartWrite(void);
tSDCardFileNames		*BSP_SDCard_GetNewFileName(void);
#endif
