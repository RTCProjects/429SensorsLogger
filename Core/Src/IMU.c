/*----------------------------------------------------------------------------------------------------*/
/**
  * @file           IMU.c
  * @brief          Модуль для работы с гиродатчиком и акселерометром
**/
/*----------------------------------------------------------------------------------------------------*/
#include <bmp180.h>
#include "IMU.h"
#include "bsp_exti.h"
#include "bsp_usb.h"
#include "bsp_sdcard.h"
#include "bsp_usart.h"
/*----------------------------------------------------------------------------------------------------*/
extern I2C_HandleTypeDef I2C1Handle;
osThreadId 				IMUDeviceHandle;
void	IMU_Task(void const * argument);

float Ax, Ay, Az, Gx, Gy, Gz;

tSDCardWriteData	accumData[IMU_LOW_DATA_SIZE] CCM_SRAM;	//размер массива равен частоте опроса датчика IMU
tSDCardWriteData	skifCurrentData;

uint16_t	uBMPCounter = 0;
uint16_t	uIMUCounter = 0;
uint16_t	uIMUSDCardCounter = 0;
static float	fAltitude = 0;
xSemaphoreHandle xIMURdySemaphore;
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 		IMU Инициализация
  * @reval		None
  */
void IMU_Init(void)
{
	osThreadDef(IMUTask, IMU_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE + 0x300 );
	IMUDeviceHandle = osThreadCreate(osThread(IMUTask), NULL);

	vSemaphoreCreateBinary(xIMURdySemaphore);
	uIMUCounter = 0;
	uBMPCounter = 0;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Поток обработки данных с датчика IMU.
  * @param		 	argument: параметры потока FreeRTOS
  * @reval			None
  */
void	IMU_Task(void const * argument)
{
	uint8_t	devID = 0;

	BSP_I2C_Init();
	Devices_IMUOff();
	osDelay(10);
	Devices_IMUOn();
	osDelay(10);
	//	Clear sleep mode bit (6), enable all sensors
	BSP_I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);
	devID = BSP_I2C_Read_Byte(MPU6050_ADDRESS, WHO_AM_I_MPU9255);

	if(devID == 0x73)
	{
		//	Set accelerometers low pass filter at 5Hz
		BSP_I2C_Write_Byte(MPU6050_ADDRESS, CONFIG, 0x06);
		osDelay(10);
		//	Configure gyroscope range
		BSP_I2C_Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG, GYRO_FULL_SCALE_250_DPS);
		osDelay(10);
		//	Configure accelerometers range
		BSP_I2C_Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, ACC_FULL_SCALE_2_G);
		osDelay(10);
		// Set 200Hz INT
		BSP_I2C_Write_Byte(MPU6050_ADDRESS, SMPLRT_DIV , 0x03);
		osDelay(10);
		//	Configuring hardware interrupts on INT pin
		BSP_I2C_Write_Byte(MPU6050_ADDRESS, INT_ENABLE, DATA_RDY_EN);
		osDelay(10);
	}

	else {
		Error_Handler();
	}

	BMP180_Init();

	while(1)
	{
		xSemaphoreTake( xIMURdySemaphore, portMAX_DELAY );
		IMU_Calcualte();
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Функция запроса измеренных данных с IMU датчика
  * @reval			None
  */
extern xQueueHandle 		xAltitudeDataQueue;

void IMU_Calcualte(void)
{

	//	Send raw data to the filter
	MadgwickAHRSupdateIMU(Gx, Gy, Gz, Ax, Ay, Az);

	//	Calculate angles from quaternions
	roll = atan2(2*(q0*q1+q2*q3), q3*q3-q2*q2-q1*q1+q0*q0);
	pitch = asin(2.0f*(q1*q3-q0*q2));
	yaw = atan2(2*(q0*q3+q1*q2), q1*q1+q0*q0-q3*q3-q2*q2);

	//	Translation of angles from radians to degrees
	yaw = RAD_TO_DEG * yaw;
	pitch = RAD_TO_DEG * pitch;
	roll = RAD_TO_DEG * roll;

	if(uIMUCounter < IMU_LOW_DATA_SIZE)
	{
		if(uBMPCounter < 20)
		{
			uBMPCounter++;
			if(uBMPCounter==20){
				BMP180_StartMeasure();
				uBMPCounter = 0;
			}
		}

		tSensors *curDistanceData = Devices_GetDataPointer();
		//

		memcpy(&accumData[uIMUCounter].sensorsData,curDistanceData,sizeof(tSensors));

		accumData[uIMUCounter].imuData.fAz = Az;
		accumData[uIMUCounter].imuData.fPitch = pitch;
		accumData[uIMUCounter].imuData.fRoll = roll;
		//accumData[uIMUCounter].fAltitude = 0;
		accumData[uIMUCounter].fLatitude = 0;
		accumData[uIMUCounter].fLongitude = 0;
		accumData[uIMUCounter].fAltitude = BMP180_GetAltitude();

		if(uIMUCounter == IMU_LOW_DATA_SIZE / 2){
			mainGiveSemaphore();
		}

		uIMUCounter++;

	}
	else
	{
		uIMUCounter = 0;
		memcpy(&skifCurrentData,&accumData[IMU_LOW_DATA_SIZE - 1],sizeof(tSDCardWriteData));
		//WriteSDCardThread
		mainGiveSemaphore();
		BSP_SDCard_StartWrite();

	}


	uIMUSDCardCounter++;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Прерываение 200Hz с MPU6050
  * @reval			None
  */
void BSP_EXTI5_Callback()
{
	portBASE_TYPE 	xTaskWoken;
	uint8_t 		Buf[14];
	HAL_StatusTypeDef hStatus = HAL_I2C_Mem_Read(&I2C1Handle, (uint16_t)MPU6050_ADDRESS << 1, ACCEL_XOUT_H, 1, Buf, 14, 0x100);
	if(hStatus!= HAL_OK){
		static I2C_Module	i2c1Module;
			i2c1Module.instance = I2C1Handle;
			i2c1Module.sclPin = GPIO_PIN_6;
			i2c1Module.sclPort = GPIOB;
			i2c1Module.sdaPin = GPIO_PIN_8;
			i2c1Module.sdaPort = GPIOB;

		I2C_ClearBusyFlagErratum(&i2c1Module);
	}
	ax = (int16_t)Buf[0]<<8 | Buf[1];
	ay = (int16_t)Buf[2]<<8 | Buf[3];
	az = (int16_t)Buf[4]<<8 | Buf[5];

	gx = (int16_t)Buf[8]<<8 | Buf[9];
	gy = (int16_t)Buf[10]<<8 | Buf[11];
	gz = (int16_t)Buf[12]<<8 | Buf[13];

	Ax = (float)(ax) * A_RES;
	Ay = (float)(ay) * A_RES;
	Az = (float)(az) * A_RES;

	Gx = (float)(gx) * G_RES * DEG_TO_RAD;
	Gy = (float)(gy) * G_RES * DEG_TO_RAD;
	Gz = (float)(gz) * G_RES * DEG_TO_RAD;




	xSemaphoreGiveFromISR( xIMURdySemaphore, &xTaskWoken );
	if( xTaskWoken == pdTRUE){
			taskYIELD();
	}

}
/*----------------------------------------------------------------------------------------------------*/
