#include "bmp180.h"

__IO int16_t AC1 = 0, AC2 = 0, AC3 = 0;
__IO uint16_t AC4 = 0, AC5 = 0, AC6 = 0;
__IO int16_t B1 = 0, B2 = 0, MB = 0, MC = 0, MD = 0;

void	BMP180_Init()
{
	uint8_t	devID = 0;
	uint8_t	regByte[2];


	devID = BSP_I2C_Read_Byte(BMP180_I2CADDR,WHO_AM_I_BMP280);
	if(devID == 0x55){
		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC1_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC1_L);
		AC1 = regByte[1] << 8 | regByte[0];

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC2_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC2_L);
		AC2 = regByte[1] << 8 | regByte[0];

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC3_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC3_L);
		AC3 = regByte[1] << 8 | regByte[0];

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC4_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC4_L);
		AC4 = regByte[1] << 8 | regByte[0];

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC5_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_L);
		AC5 = regByte[1] << 8 | regByte[0];

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_L);
		AC6 = regByte[1] << 8 | regByte[0];

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B1_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B1_L);
		B1 = regByte[1] << 8 | regByte[0];

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B2_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B2_L);
		B2 = regByte[1] << 8 | regByte[0];

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MC_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MC_L);
		MC = regByte[1] << 8 | regByte[0];

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MD_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MD_L);
		MD = regByte[1] << 8 | regByte[0];
	}
	else
	{
		Error_Handler();
	}
}

int16_t	BMP180_MeasureT()
{
	uint16_t	regByte[2];
	int16_t	UT = 0;

	BSP_I2C_Write_Byte(BMP180_I2CADDR,BMP085_RA_CONTROL,BMP085_MODE_TEMPERATURE);
	vTaskDelay(5);

	regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MSB);
	regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_LSB);

	UT = regByte[1] << 8 | regByte[0];

	return UT;
}

int32_t BMP180_MeasureP(uint8_t oss)
{
	uint16_t	regByte[3];
	int32_t		PT = 0;

	BSP_I2C_Write_Byte(BMP180_I2CADDR,BMP085_RA_CONTROL,BMP085_MODE_PRESSURE_0 + (oss << 6));
	vTaskDelay(5);

	regByte[2] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MSB);
	regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_LSB);
	regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_XLSB);

	PT = ((regByte[2] << 16) | (regByte[1] << 8) | regByte[0]) >> (8 - oss);

	return PT;
}

float BMP180_Altitude()
{
	uint8_t	oss = 0;
	float	fAltitude = 0;

	__IO int16_t UT =	BMP180_MeasureT();
	__IO int32_t UP =	BMP180_MeasureP(oss);

	__IO long X1 = 0,X2 = 0,X3 = 0,B3 = 0,B5 = 0,B6 = 0,p = 0;
	unsigned long B4 = 0,B7 = 0;
	//Temp compensate
	X1 = (UT - AC6) * AC5 / 32768;
	X2 = MC * 2048 / (X1 + MD);
	B5 = X1 + X2;
	//Press compensate
	X1 = 0;
	X2 = 0;

	B6 = B5 - 4000;
	X1 = (B2 * (B6 * B6 / 4096)) / 2048;
	X2 = AC2 * B6 / 2048;
	X3 = X1 + X2;
	B3 = (((AC1*4+X3)<<oss)+2)/4;
	X1 = AC3 * B6 / 8192;
	X2 = (B1 * (B6 * B6 / 4096)) / 65536;
	X3 = ((X1 + X2) + 2) / 4;
	B4 = AC4 * (unsigned long)(X3 + 32768) / 32768;
	B7 = ((unsigned long)UP - B3) * (50000 >> oss);
	if(B7 < 0x80000000){p = (B7 * 2)/B4;}
	else {p = (B7 / B4) * 2;}
	X1 = (p / 256) * (p / 256);
	X1 = (X1 * 3038 / 65536);
	X2 = (-7357 * p) / 65536;
	p = p + (X1 + X2 + 3791) / 16;



	//fAltitude = 44330.0f*(1-pow(p/101858.0f,1/5.255));

	return p;
}


