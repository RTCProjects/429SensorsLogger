/*----------------------------------------------------------------------------------------------------*/
/**
  * @file           bmp180.c
  * @brief          Модуль для работы с барометрическим датчиком
**/
/*----------------------------------------------------------------------------------------------------*/
#include "bmp180.h"

osThreadId 			altitudeTaskHandle;
xSemaphoreHandle 	xAltitudeSemaphore;

void BMP180Task(void const * argument);

tBMP180Sensor	bmpSensor[BMP180_CHANNELS];
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Инициализация процесса высотомеров
  * @reval			None
  */
void	BMP180_Init()
{
	memset(bmpSensor,0,sizeof(tBMP180Sensor) * BMP180_CHANNELS);

	vSemaphoreCreateBinary(xAltitudeSemaphore);

	osThreadDef(altitudeTask, BMP180Task, osPriorityRealtime, 0, configMINIMAL_STACK_SIZE + 0x100);
	altitudeTaskHandle = osThreadCreate(osThread(altitudeTask), NULL);
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Поток инициализации и обработки данных высотомеров
  * @reval			None
  */
void BMP180Task(void const * argument)
{
	uint8_t	devID = 0;

	//BMP180 Channel 1
	devID = BSP_I2C_Read_Byte(BMP180_I2CADDR,WHO_AM_I_BMP280);
	if(devID == 0x55)
	{
		bmpSensor[0].cData.AC1 = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC1_H) << 8 | BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC1_L);
		osDelay(10);
		bmpSensor[0].cData.AC2 = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC2_H) << 8 | BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC2_L);
		osDelay(10);
		bmpSensor[0].cData.AC3 = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC3_H) << 8 | BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC3_L);
		osDelay(10);
		bmpSensor[0].cData.AC4 = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC4_H) << 8 | BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC4_L);
		osDelay(10);
		bmpSensor[0].cData.AC5 = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC5_H) << 8 | BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_L);
		osDelay(10);
		bmpSensor[0].cData.AC6 = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_H) << 8 | BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_L);
		osDelay(10);
		bmpSensor[0].cData.B1 = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B1_H) << 8 | BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B1_L);
		osDelay(10);
		bmpSensor[0].cData.B2 = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B2_H) << 8 | BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_B2_L);
		osDelay(10);
		bmpSensor[0].cData.MC = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MC_H) << 8 | BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MC_L);
		osDelay(10);
		bmpSensor[0].cData.MD = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MD_H) << 8 | BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MD_L);
		osDelay(10);
	}
	else{
			Error_Handler();
	}
	//BMP180 Channel 2
	devID = BSP_I2C2_Read_Byte(BMP180_I2CADDR,WHO_AM_I_BMP280);
	if(devID == 0x55)
	{
		bmpSensor[1].cData.AC1 = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC1_H) << 8 | BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC1_L);
		osDelay(10);
		bmpSensor[1].cData.AC2 = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC2_H) << 8 | BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC2_L);
		osDelay(10);
		bmpSensor[1].cData.AC3 = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC3_H) << 8 | BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC3_L);
		osDelay(10);
		bmpSensor[1].cData.AC4 = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC4_H) << 8 | BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC4_L);
		osDelay(10);
		bmpSensor[1].cData.AC5 = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC5_H) << 8 | BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_L);
		osDelay(10);
		bmpSensor[1].cData.AC6 = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_H) << 8 | BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_AC6_L);
		osDelay(10);
		bmpSensor[1].cData.B1 = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_B1_H) << 8 | BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_B1_L);
		osDelay(10);
		bmpSensor[1].cData.B2 = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_B2_H) << 8 | BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_B2_L);
		osDelay(10);
		bmpSensor[1].cData.MC = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_MC_H) << 8 | BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_MC_L);
		osDelay(10);
		bmpSensor[1].cData.MD = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_MD_H) << 8 | BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_MD_L);
		osDelay(10);
	}
	else{
		Error_Handler();
	}

	xSemaphoreTake( xAltitudeSemaphore, portMAX_DELAY );
	while(1)
	{
		xSemaphoreTake( xAltitudeSemaphore, portMAX_DELAY );
		BMP180_CalibrateData(BMP180_CHANNEL1);
		BMP180_CalibrateData(BMP180_CHANNEL2);
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Запуск измерений
  * @reval			None
  */
void	BMP180_StartMeasure()
{
	xSemaphoreGive(xAltitudeSemaphore);
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Получение некалиброванных данных температуры
  * @param			ch:Номер канала высотомера
  * @reval			None
  */
void	BMP180_MeasureT(eBMP180Channel	ch)
{
	uint16_t	regByte[2];
	switch(ch)
	{
		case BMP180_CHANNEL1:
		{
			BSP_I2C_Write_Byte(BMP180_I2CADDR,BMP085_RA_CONTROL,BMP085_MODE_TEMPERATURE);
			vTaskDelay(5);

			regByte[1] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_MSB);
			regByte[0] = BSP_I2C_Read_Byte(BMP180_I2CADDR,BMP085_RA_LSB);

			bmpSensor[ch].UT = regByte[1] << 8 | regByte[0];
		}break;

		case BMP180_CHANNEL2:
		{
			BSP_I2C2_Write_Byte(BMP180_I2CADDR,BMP085_RA_CONTROL,BMP085_MODE_TEMPERATURE);
			vTaskDelay(5);

			regByte[1] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_MSB);
			regByte[0] = BSP_I2C2_Read_Byte(BMP180_I2CADDR,BMP085_RA_LSB);

			bmpSensor[ch].UT = regByte[1] << 8 | regByte[0];
		}break;
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Получение некалиброванных данных давления
  * @param			ch:Номер канала высотомера
  * @reval			None
  */
void BMP180_MeasureP(eBMP180Channel	ch)
{
	uint8_t	regByte[3];
	switch(ch)
	{
		case BMP180_CHANNEL1:
		{
			BSP_I2C_Write_Byte(BMP180_I2CADDR,BMP085_RA_CONTROL,BMP085_MODE_PRESSURE_0 + (bmpSensor[ch].oss << 6));
			vTaskDelay(8);
			BSP_I2C_Read_Bytes(BMP180_I2CADDR,BMP085_RA_MSB,3,(uint8_t*)regByte);

			bmpSensor[ch].UP = (int32_t)((regByte[0] << 16) | (regByte[1] << 8) | regByte[2]) >> (8 - bmpSensor[ch].oss);
		}break;

		case BMP180_CHANNEL2:
		{
			BSP_I2C2_Write_Byte(BMP180_I2CADDR,BMP085_RA_CONTROL,BMP085_MODE_PRESSURE_0 + (bmpSensor[ch].oss << 6));
			vTaskDelay(8);
			BSP_I2C2_Read_Bytes(BMP180_I2CADDR,BMP085_RA_MSB,3,(uint8_t*)regByte);

			bmpSensor[ch].UP = (int32_t)((regByte[0] << 16) | (regByte[1] << 8) | regByte[2]) >> (8 - bmpSensor[ch].oss);
		}break;
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Получение данных давления
  * @param			ch:Номер канала высотомера
  * @reval			давление в Па
  */
uint32_t BMP180_GetPressure(eBMP180Channel ch)
{
	return bmpSensor[ch].ulPressure;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Получение данных высоты
  * @param			ch:Номер канала высотомера
  * @reval			высота в
  */
float	BMP180_GetAltitude(eBMP180Channel ch)
{
	return bmpSensor[ch].fAltitude;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Калибровка данных температуры и давления + рассчёт высоты
  * @param			ch:Номер канала высотомера
  * @reval			None
  */
void BMP180_CalibrateData(eBMP180Channel ch)
{

	BMP180_MeasureT(ch);
	BMP180_MeasureP(ch);

	__IO long X1 = 0,X2 = 0,X3 = 0,B3 = 0,B5 = 0,B6 = 0,p = 0;
	unsigned long B4 = 0,B7 = 0;
	//Temp compensate
	X1 = (bmpSensor[ch].UT - bmpSensor[ch].cData.AC6) * bmpSensor[ch].cData.AC5 / 32768;
	X2 = bmpSensor[ch].cData.MC * 2048 / (X1 + bmpSensor[ch].cData.MD);
	B5 = X1 + X2;
	//Press compensate
	X1 = 0;
	X2 = 0;

	B6 = B5 - 4000;
	X1 = (bmpSensor[ch].cData.B2 * (B6 * B6 / 4096)) / 2048;
	X2 = bmpSensor[ch].cData.AC2 * B6 / 2048;
	X3 = X1 + X2;
	B3 = (((bmpSensor[ch].cData.AC1*4+X3)<<bmpSensor[ch].oss)+2)/4;
	X1 = bmpSensor[ch].cData.AC3 * B6 / 8192;
	X2 = (bmpSensor[ch].cData.B1 * (B6 * B6 / 4096)) / 65536;
	X3 = ((X1 + X2) + 2) / 4;
	B4 = bmpSensor[ch].cData.AC4 * (unsigned long)(X3 + 32768) / 32768;
	B7 = ((unsigned long)bmpSensor[ch].UP - B3) * (50000 >> bmpSensor[ch].oss);
	if(B7 < 0x80000000){p = (B7 * 2)/B4;}
	else {p = (B7 / B4) * 2;}
	X1 = (p / 256) * (p / 256);
	X1 = (X1 * 3038 / 65536);
	X2 = (-7357 * p) / 65536;
	p = p + (X1 + X2 + 3791) / 16;

	//*Pressure = p;
	bmpSensor[ch].ulPressure = p;

	bmpSensor[ch].fAltitude = 44330.0f*(1-pow(p/(float)bmpSensor[ch].ulZeroPressure,1/5.255));

	if(bmpSensor[ch].uZeroAltCounter < BMP180_ZEROALT_ITERs){
		bmpSensor[ch].ulZeroPressure+=p;
		bmpSensor[ch].uZeroAltCounter ++;

		if(bmpSensor[ch].uZeroAltCounter == BMP180_ZEROALT_ITERs){
			bmpSensor[ch].ulZeroPressure = bmpSensor[ch].ulZeroPressure / BMP180_ZEROALT_ITERs;
		}
	bmpSensor[ch].fAltitude = 0;
	}
}
/*----------------------------------------------------------------------------------------------------*/
