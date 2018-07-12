#include "nmea.h"

tNMEAPosition	nmeaPosition;

uint8_t NMEA_Parse(uint8_t *GPSBuf,uint8_t	size)
{

	for(int i = 0;i<size;i++)
	{
		if(GPSBuf[i] == '$' && GPSBuf[i + 1] == 'G' && GPSBuf[i + 2] == 'N' && GPSBuf[i + 3] == 'G' && GPSBuf[i + 4] == 'L' && GPSBuf[i + 5] == 'L'){
			if(GPSBuf[i + 6] == ',' && GPSBuf[i + 7] != ','){
				if(size - i > NMEA_POS_SIZE){

					//memset(tempStr,0,sizeof(char) * 26);
					strncpy((char*)nmeaPosition.strPosition,(char*)&GPSBuf[i+7],NMEA_POS_SIZE);
					/*for(int j = 0;j<26;j++){

						tempStr[j] = GPSBuf[j+i+7];
					}*/
					break;
				}
			}
		}
	}
	return 1;
}

char	*NMEA_GetPositionString()
{
	return nmeaPosition.strPosition;
}
