#ifndef _BSP_DRIVER_SPI_H
#define _BSP_DRIVER_SPI_H

#define SPI_OK					0x00
#define SPI_ERROR	 			0x01

#define SDCARD_WRITE_QUEUE_SIZE	16
#define SDCARD_FILENAME_MAX_LEN	32
#define SDCARD_LOGDATA_SIZE		300

#define DIRNAME_SENSORS			""


#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "devices.h"
#include "skif.h"
#include "gps.h"

/**
 * Struct/enum section
 **/

typedef struct sd_info{
	uint8_t	ocr1[4];
	uint8_t	ocr2[4];
  uint8_t type;
}sd_info_ptr;	//информация о SD карте

typedef struct
{

	tSensors	sensorsData;
	tIMUData	imuData;
	uint8_t		uRadarVspeed;
	float		fAltitude;
	float		fAltitude2;
	float		fLatitude;
	float		fLongitude;
	char		strNMEAPosition[NMEA_POS_SIZE];
	char		strNMEAVelocity[NMEA_VEL_SIZE];
}tSDCardWriteData;	//формат записи лога на SD карту

/**
 * Func section
 **/

void		BSP_SDCard_Init(void);
void		BSP_SDCard_WriteSensorsData(tSDCardWriteData	*Data);
uint16_t	BSP_SDCard_GetFileNumbers(char*);
void 		BSP_SDCard_SPIInit(uint32_t baudratePrescaler);
void 		BSP_SDCardSPIDeInit(void);
void		BSP_SDCard_StartWrite(void);
char		*BSP_SDCard_GetNewFileName(void);

/**
 * Extern section
 **/

extern tSDCardWriteData	accumData[IMU_LOW_DATA_SIZE] CCM_SRAM;

#endif
