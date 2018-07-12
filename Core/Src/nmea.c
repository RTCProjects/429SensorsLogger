#include "nmea.h"
#include "minmea.h"

uint8_t NMEA_Parse(uint8_t *GPSBuf,uint8_t	size)
{

	__IO uint8_t	foobar1 = 0;

	foobar1 = minmea_sentence_id(GPSBuf,false);
}
