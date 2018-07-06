/*----------------------------------------------------------------------------------------------------*/
/**
  * @file           IMU.c
  * @brief          Модуль для работы с гиродатчиком и акселерометром
**/
/*----------------------------------------------------------------------------------------------------*/
#include "IMU.h"
#include "bsp_exti.h"
#include "bsp_usb.h"
#include "bsp_sdcard.h"
#include "bsp_usart.h"

#define IMU_LOW_DATA_SIZE 10

extern I2C_HandleTypeDef I2C1Handle;
osThreadId 				IMUDeviceHandle;
void	IMU_Task(void const * argument);

static float	fTemp1X,fTemp1Y,fTempAz;
float		fCalibX,fCalibY,fCalibAz;
uint8_t		uCalibState;
uint16_t	uCalibCounter;

const uint16_t	CALIB_REQUEST_VAL = 1000;

float	SelfTest[6];
float 	gyroBias[3], accelBias[3];
float Ax, Ay, Az, Gx, Gy, Gz;

tIMUData	imuCurrentData;
tIMULowData	imuLowData[IMU_LOW_DATA_SIZE] CCM_SRAM;
uint8_t	uIMURdy = 0;
uint16_t	uIMUCounter = 0;



xSemaphoreHandle xIMURdySemaphore;
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 		IMU Инициализация
  * @reval		None
  */
void IMU_Init(void)
{
	osThreadDef(IMUTask, IMU_Task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE + 0x200 );
	IMUDeviceHandle = osThreadCreate(osThread(IMUTask), NULL);

	vSemaphoreCreateBinary(xIMURdySemaphore);

	fTemp1X= 0;
	fTemp1Y = 0;
	fTempAz = 0;

	uCalibState = 1;
	uCalibCounter = 0;
	fCalibX = 0;
	fCalibY = 0;
	fCalibAz = 0;

	uIMURdy = 0;
	uIMUCounter = 0;
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
		/*	osDelay(5000);
			//Self test
			IMU_SelfTest(SelfTest);
			sprintf(strFmtString,"aX%f aY%f aZ%f gX%f gY%f gZ%f\n\n",SelfTest[0],SelfTest[1],SelfTest[2],SelfTest[3],SelfTest[4],SelfTest[5]);
			BSP_Usb_SendString(strFmtString);
			if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f)
				BSP_Usb_SendString("Pass selftest!\r\n");

			//Calibrate
			IMU_Calibrate(gyroBias, accelBias);
			sprintf(strFmtString,"gbX%f gbY%f gbZ%f abX%f abY%f abZ%f\n\n",gyroBias[0],gyroBias[1],gyroBias[2],accelBias[0],accelBias[1],accelBias[2]);
			BSP_Usb_SendString(strFmtString);
		*/
		//	Set accelerometers low pass filter at 5Hz
		BSP_I2C_Write_Byte(MPU6050_ADDRESS, CONFIG, 0x06);
		osDelay(10);
		//	Configure gyroscope range
		BSP_I2C_Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG, GYRO_FULL_SCALE_250_DPS);
		osDelay(10);
		//	Configure accelerometers range
		BSP_I2C_Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, ACC_FULL_SCALE_2_G);
		osDelay(10);
		// Set 1000Hz INT
		BSP_I2C_Write_Byte(MPU6050_ADDRESS, SMPLRT_DIV , 0x00);
		osDelay(10);
		//	Configuring hardware interrupts on INT pin
		BSP_I2C_Write_Byte(MPU6050_ADDRESS, INT_ENABLE, DATA_RDY_EN);
		osDelay(10);
	}

	else {
		Error_Handler();
	}

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
extern tSensors	SensorsData;
extern float Ax, Ay, Az, Gx, Gy, Gz;
extern float roll,pitch,yaw;
char	strBufOutput[128];
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




	if(uIMUCounter >= 250){
		uIMUCounter = 0;

		Devices_SensorsDataRequest();
		IMU_SensorsDataRequest();
		mainGiveSemaphore();
		//sprintf(strBufOutput,"L%5d R%5d S%5d\nRoll:%f\t Pitch:%f\t Yaw:%f\n",SensorsData.ulLidarDistance,SensorsData.ulRadarDistance,SensorsData.ulSonarDistance,roll,pitch,yaw);
		//BSP_WIFI_UARTSend((uint8_t*)strBufOutput,strlen(strBufOutput));
	}
	uIMUCounter++;

	/*
	memset(&writeData.imuData,0,sizeof(tIMUData));
							writeData.type = E_GYRO;
							writeData.imuData.fAz = Az;
							writeData.imuData.fPitch = Gx;//fTemp1X / M_PI * 180.0f / 72.0f * 90.0f;
							writeData.imuData.fRoll = Gy;//fTemp1Y / M_PI * 180.0f / 72.0f * 90.0f;

	imuCurrentData.fAz = Az;
	imuCurrentData.fPitch = Gx;
	imuCurrentData.fRoll = Gy;

	BSP_SDCard_WriteSensorsData(&writeData);*/

}

void IMU_SensorsDataRequest()
{
	static tSDCardWriteData	writeData;

	memset(&writeData.imuData,0,sizeof(tIMUData));
								writeData.type = E_GYRO;
								writeData.imuData.fAz = Az;
								writeData.imuData.fPitch = pitch;//fTemp1X / M_PI * 180.0f / 72.0f * 90.0f;
								writeData.imuData.fRoll = roll;//fTemp1Y / M_PI * 180.0f / 72.0f * 90.0f;

	BSP_SDCard_WriteSensorsData(&writeData);
}
/*----------------------------------------------------------------------------------------------------*/
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

	Ax = (float)(ax/* - rKoef.fKoef[0]*/) * A_RES;
	Ay = (float)(ay/* - rKoef.fKoef[1]*/) * A_RES;
	Az = (float)(az/* - rKoef.fKoef[2]*/) * A_RES;

	Gx = (float)(gx/* - cKoef.fKoef[0]*/) * G_RES * DEG_TO_RAD;
	Gy = (float)(gy/* - cKoef.fKoef[1]*/) * G_RES * DEG_TO_RAD;
	Gz = (float)(gz/* - cKoef.fKoef[2]*/) * G_RES * DEG_TO_RAD;

	xSemaphoreGiveFromISR( xIMURdySemaphore, &xTaskWoken );
	if( xTaskWoken == pdTRUE){
			taskYIELD();
	}



	/*if(uIMUCounter < IMU_LOW_DATA_SIZE)
	{
		imuLowData[uIMUCounter].fAccel[0] = Ax;
		imuLowData[uIMUCounter].fAccel[1] = Ay;
		imuLowData[uIMUCounter].fAccel[2] = Az;
		imuLowData[uIMUCounter].fAccel[0] = Gx;
		imuLowData[uIMUCounter].fAccel[1] = Gy;
		imuLowData[uIMUCounter].fAccel[2] = Gz;

		uIMUCounter++;

		writeData.type = E_GYRO;
	}
	else
	{


		uIMUCounter = 0;
	}*/
}
/*----------------------------------------------------------------------------------------------------*/
