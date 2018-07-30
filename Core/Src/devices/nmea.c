/*----------------------------------------------------------------------------------------------------*/
/**
  * @file           nmea.c
  * @brief          ћодуль декодировани€ GNGLL данных с координатами GPS
**/
/*----------------------------------------------------------------------------------------------------*/
#include "nmea.h"
tNMEAPosition	nmeaPosition;

uint8_t		gpsBuffer[NMEA_BUF_SIZE];
uint8_t		writeBufInd = 0;
uint8_t		readBufInd = 0;
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ѕарсинг строки NMEA
  * @param	GPSBuf: указатель на строку данных
  * @param	size: длина строки данных
  * @retval None
  */
void	NMEA_RcvByteCallback(uint8_t inputByte)
{
	gpsBuffer[(writeBufInd++)%NMEA_BUF_SIZE] = inputByte;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			‘ункци€ парсинга GPS данных
  * @reval			None
  */
void NMEA_Parse()
{
	//немного гомнокода
	uint8_t	GPSBuf[NMEA_BUF_SIZE];

	memcpy((uint8_t*)GPSBuf,(uint8_t*)gpsBuffer,sizeof(uint8_t)*NMEA_BUF_SIZE);
	//теперь очень много гомнокода
	//TODO в идеале переписать прием данных от GPS на DMA, как сделано в радаре + сделать номральный парсер NMEA180 на основе sscanf
	for(uint8_t i = 0;i<NMEA_BUF_SIZE - 7;i++)
	{
		if(GPSBuf[i] == '$' && GPSBuf[i + 1] == 'G' && GPSBuf[i + 2] == 'N' && GPSBuf[i + 3] == 'G' && GPSBuf[i + 4] == 'L' && GPSBuf[i + 5] == 'L')
		{
			__IO uint8_t	gpsCoordInd = i + 7;
			if(gpsCoordInd > (NMEA_BUF_SIZE - NMEA_POS_SIZE))break;
			else{
				if(GPSBuf[gpsCoordInd + NMEA_POS_SIZE - 2] != 'E')
					break;
				else{
					memset((char*)&nmeaPosition.strPosition[0],0,sizeof(uint8_t) * NMEA_POS_SIZE);
					strncpy((char*)nmeaPosition.strPosition,(char*)&GPSBuf[gpsCoordInd],sizeof(uint8_t) * NMEA_POS_SIZE - 1);
				}
			}
		}
	}

	for(uint8_t i = 0;i<NMEA_BUF_SIZE - 7;i++)
	{
		if(GPSBuf[i] == '$' && GPSBuf[i + 1] == 'G' && GPSBuf[i + 2] == 'N' && GPSBuf[i + 3] == 'V' && GPSBuf[i + 4] == 'T' && GPSBuf[i + 5] == 'G')
				{
					uint8_t	gpsVelocityInd = i + 7;
					if(gpsVelocityInd > (NMEA_BUF_SIZE - NMEA_VEL_SIZE))break;
					else{
						__IO uint8_t	*pGPSVelocityData = (uint8_t*)&GPSBuf[gpsVelocityInd];

						__IO uint8_t	velStartInd = 0;
						__IO uint8_t velEndInd = 0;

						for(uint8_t j = 0;j<NMEA_VEL_SIZE;j++){
							if(pGPSVelocityData[j] == 'N')velStartInd = j + 2;
							if(pGPSVelocityData[j] == 'K')velEndInd = j - 1;
						}
						velStartInd = velStartInd + gpsVelocityInd;
						velEndInd = velEndInd + gpsVelocityInd;

						uint8_t	velStrSize = velEndInd - velStartInd;

						if(velStrSize > 0 && velStrSize < NMEA_VEL_SIZE){
							memset(nmeaPosition.strVelocity,0,sizeof(uint8_t) * NMEA_VEL_SIZE);
							strncpy((char*)nmeaPosition.strVelocity,(char*)&GPSBuf[velStartInd],sizeof(uint8_t) * velStrSize );
						}
					}
				}
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  «апрос строки с коориднатами
  * @retval укататель на строку с координатами
  */
char	*NMEA_GetPositionString()
{
	return nmeaPosition.strPosition;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  «апрос строки с путевой скоростью
  * @retval укататель на строку с координатами
  */
char	*NMEA_GetVelocityString()
{
	return nmeaPosition.strVelocity;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  «апрос строки с буфером от GPS приемника
  * @retval укататель на строку с координатами
  */
uint8_t	*NMEA_GetGPSBuffer()
{
	return gpsBuffer;
}
/*----------------------------------------------------------------------------------------------------*/
