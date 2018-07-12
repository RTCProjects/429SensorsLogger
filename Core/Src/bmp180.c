#include "bmp180.h"

__IO int16_t AC1 = 0, AC2 = 0, AC3 = 0;
__IO uint16_t AC4 = 0, AC5 = 0, AC6 = 0;
__IO int16_t B1 = 0, B2 = 0, MB = 0, MC = 0, MD = 0;

__IO int16_t AC1_2 = 0, AC2_2 = 0, AC3_2 = 0;
__IO uint16_t AC4_2 = 0, AC5_2 = 0, AC6_2 = 0;
__IO int16_t B1_2 = 0, B2_2 = 0, MB_2 = 0, MC_2 = 0, MD_2 = 0;



osThreadId 			altitudeTaskHandle;
xSemaphoreHandle 	xAltitudeSemaphore;
xQueueHandle 		xAltitudeDataQueue;

void BMP180Task(void const * argument);

uint32_t ulPressure;
float	 fAltitude;

uint32_t	uZeroPressure;
uint8_t		uZeroCounter;

uint32_t ulPressure2;
float	 fAltitude2;

uint32_t	uZeroPressure2;
uint8_t		uZeroCounter2;

tBMP180Sensor	bmpSensor;

void	BMP180_Init()
{
	ulPressure = 0;
	fAltitude = 0;
	uZeroCounter = 0;
	uZeroPressure = 0;

	ulPressure2 = 0;
	fAltitude2 = 0;
	uZeroCounter2 = 0;
	uZeroPressure2 = 0;

	osThreadDef(altitudeTask, BMP180Task, osPriorityRealtime, 0, configMINIMAL_STACK_SIZE + 0x100);
	altitudeTaskHandle = osThreadCreate(osThread(altitudeTask), NULL);

	xAltitudeDataQueue = xQueueCreate(1, sizeof(uint32_t));

	vSemaphoreCreateBinary(xAltitudeSemaphore);
}

void BMP180Task(void const * argument)
{
	uint8_t	devID = 0;
	uint8_t	regByte[2];

	/*devID = BSP_I2C_Read_Byte(BMP180_I2CADDR,WHO_AM_I_BMP280);
	if(devID == 0x55)
	{
		BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_AC1_H,2,(uint8_t*)&bmpSensor.cData.AC1);
		BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_AC2_H,2,(uint8_t*)&bmpSensor.cData.AC2);
		BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_AC3_H,2,(uint8_t*)&bmpSensor.cData.AC3);
		BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_AC4_H,2,(uint8_t*)&bmpSensor.cData.AC4);
		BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_AC5_H,2,(uint8_t*)&bmpSensor.cData.AC5);
		BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_AC6_H,2,(uint8_t*)&bmpSensor.cData.AC6);
		BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_B1_H,2,(uint8_t*)&bmpSensor.cData.B1);
		BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_B2_H,2,(uint8_t*)&bmpSensor.cData.B2);
		BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_MC_H,2,(uint8_t*)&bmpSensor.cData.MC);
		BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_MD_H,2,(uint8_t*)&bmpSensor.cData.MD);

	}*/

	//BMP180 Channel 1
	devID = BSP_I2C_Read_Byte(BMP180_I2CADDR,WHO_AM_I_BMP280);
	if(devID == 0x55)
	{
		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC1_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC1_L);
		AC1 = regByte[1] << 8 | regByte[0];
		osDelay(10);

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC2_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC2_L);
		AC2 = regByte[1] << 8 | regByte[0];
		osDelay(10);

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC3_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC3_L);
		AC3 = regByte[1] << 8 | regByte[0];
		osDelay(10);

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC4_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC4_L);
		AC4 = regByte[1] << 8 | regByte[0];
		osDelay(10);

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC5_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_L);
		AC5 = regByte[1] << 8 | regByte[0];
		osDelay(10);

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_L);
		AC6 = regByte[1] << 8 | regByte[0];
		osDelay(10);

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B1_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B1_L);
		B1 = regByte[1] << 8 | regByte[0];
		osDelay(10);

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B2_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B2_L);
		B2 = regByte[1] << 8 | regByte[0];
		osDelay(10);

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MC_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MC_L);
		MC = regByte[1] << 8 | regByte[0];
		osDelay(10);

		regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MD_H);
		regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MD_L);
		MD = regByte[1] << 8 | regByte[0];
		osDelay(10);

	}
	else
	{
		Error_Handler();
	}
	//BMP180 Channel 2
		devID = BSP_I2C2_Read_Byte(BMP180_I2CADDR,WHO_AM_I_BMP280);
		if(devID == 0x55)
		{
			regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC1_H);
			regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC1_L);
			AC1_2 = regByte[1] << 8 | regByte[0];
			osDelay(10);

			regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC2_H);
			regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC2_L);
			AC2_2 = regByte[1] << 8 | regByte[0];
			osDelay(10);

			regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC3_H);
			regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC3_L);
			AC3_2 = regByte[1] << 8 | regByte[0];
			osDelay(10);

			regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC4_H);
			regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC4_L);
			AC4_2 = regByte[1] << 8 | regByte[0];
			osDelay(10);

			regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC5_H);
			regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_L);
			AC5_2 = regByte[1] << 8 | regByte[0];
			osDelay(10);

			regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_H);
			regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_L);
			AC6_2 = regByte[1] << 8 | regByte[0];
			osDelay(10);

			regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_B1_H);
			regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_B1_L);
			B1_2 = regByte[1] << 8 | regByte[0];
			osDelay(10);

			regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_B2_H);
			regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_B2_L);
			B2_2 = regByte[1] << 8 | regByte[0];
			osDelay(10);

			regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_MC_H);
			regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_MC_L);
			MC_2 = regByte[1] << 8 | regByte[0];
			osDelay(10);

			regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_MD_H);
			regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_MD_L);
			MD_2 = regByte[1] << 8 | regByte[0];
			osDelay(10);

		}
		else
		{
			Error_Handler();
		}

	xSemaphoreTake( xAltitudeSemaphore, portMAX_DELAY );
	while(1)
	{
		xSemaphoreTake( xAltitudeSemaphore, portMAX_DELAY );
		BMP180_PressureAltitude(&ulPressure,&fAltitude);
		BMP180_PressureAltitude2(&ulPressure2,&fAltitude2);


		/*if(xAltitudeDataQueue!=0)
				xQueueSendToBack(xAltitudeDataQueue,&ulPressure,0);*/
	}
}

void	BMP180_StartMeasure()
{
	xSemaphoreGive(xAltitudeSemaphore);
}

int16_t	BMP180_MeasureT()
{
	uint16_t	regByte[2];
	int16_t		UT = 0;

	BSP_I2C_Write_Byte(BMP180_I2CADDR,BMP085_RA_CONTROL,BMP085_MODE_TEMPERATURE);
	vTaskDelay(5);

	regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MSB);
	regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_LSB);

	UT = regByte[1] << 8 | regByte[0];

	return UT;
}

int32_t BMP180_MeasureP(uint8_t oss)
{

	__IO uint8_t	regByte[3];
	__IO int32_t		PT = 0;

	BSP_I2C_Write_Byte(BMP180_I2CADDR,BMP085_RA_CONTROL,BMP085_MODE_PRESSURE_0 + (oss << 6));
	vTaskDelay(8);
	BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_MSB,3,(uint8_t*)regByte);

	PT = (int32_t)((regByte[0] << 16) | (regByte[1] << 8) | regByte[2]) >> (8 - oss);

	return PT;
}

uint32_t BMP180_GetPressure()
{
	return ulPressure;
}

float	BMP180_GetAltitude()
{
	return fAltitude;
}

void BMP180_PressureAltitude(uint32_t *Pressure,float *Altitude)
{
	uint8_t	oss = 0;

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

	*Pressure = p;

	*Altitude = 44330.0f*(1-pow(p/(float)uZeroPressure,1/5.255));

	if(uZeroCounter < 2){
		uZeroPressure+=p;
		uZeroCounter ++;

		if(uZeroCounter == 2){
			uZeroPressure = uZeroPressure / 2;
		}
	*Altitude = 0;
	}
}

int16_t	BMP180_Measure2T()
{
	uint16_t	regByte[2];
	int16_t		UT = 0;

	BSP_I2C2_Write_Byte(BMP180_I2CADDR,BMP085_RA_CONTROL,BMP085_MODE_TEMPERATURE);
	vTaskDelay(5);

	regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_MSB);
	regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_LSB);

	UT = regByte[1] << 8 | regByte[0];

	return UT;
}

int32_t BMP180_Measure2P(uint8_t oss)
{

	__IO uint8_t	regByte[3];
	__IO int32_t		PT = 0;

	BSP_I2C2_Write_Byte(BMP180_I2CADDR,BMP085_RA_CONTROL,BMP085_MODE_PRESSURE_0 + (oss << 6));
	vTaskDelay(8);
	BSP_I2C2_Read_Bytes(BMP180_I2CADDR,BMP085_RA_MSB,3,(uint8_t*)regByte);

	PT = (int32_t)((regByte[0] << 16) | (regByte[1] << 8) | regByte[2]) >> (8 - oss);

	return PT;
}

uint32_t BMP180_GetPressure2()
{
	return ulPressure2;
}

float	BMP180_GetAltitude2()
{
	return fAltitude2;
}

void BMP180_PressureAltitude2(uint32_t *Pressure,float *Altitude)
{
	uint8_t	oss = 0;

	__IO int16_t UT =	BMP180_Measure2T();
	__IO int32_t UP =	BMP180_Measure2P(oss);

	__IO long X1 = 0,X2 = 0,X3 = 0,B3 = 0,B5 = 0,B6 = 0,p = 0;
	unsigned long B4 = 0,B7 = 0;
	//Temp compensate
	X1 = (UT - AC6_2) * AC5_2 / 32768;
	X2 = MC_2 * 2048 / (X1 + MD_2);
	B5 = X1 + X2;
	//Press compensate
	X1 = 0;
	X2 = 0;

	B6 = B5 - 4000;
	X1 = (B2_2 * (B6 * B6 / 4096)) / 2048;
	X2 = AC2_2 * B6 / 2048;
	X3 = X1 + X2;
	B3 = (((AC1_2*4+X3)<<oss)+2)/4;
	X1 = AC3_2 * B6 / 8192;
	X2 = (B1_2 * (B6 * B6 / 4096)) / 65536;
	X3 = ((X1 + X2) + 2) / 4;
	B4 = AC4_2 * (unsigned long)(X3 + 32768) / 32768;
	B7 = ((unsigned long)UP - B3) * (50000 >> oss);
	if(B7 < 0x80000000){p = (B7 * 2)/B4;}
	else {p = (B7 / B4) * 2;}
	X1 = (p / 256) * (p / 256);
	X1 = (X1 * 3038 / 65536);
	X2 = (-7357 * p) / 65536;
	p = p + (X1 + X2 + 3791) / 16;

	*Pressure = p;

	*Altitude = 44330.0f*(1-pow(p/(float)uZeroPressure2,1/5.255));

	if(uZeroCounter2 < 2){
		uZeroPressure2+=p;
		uZeroCounter2 ++;

		if(uZeroCounter2 == 2){
			uZeroPressure2 = uZeroPressure2 / 2;
		}
	*Altitude = 0;
	}
}
