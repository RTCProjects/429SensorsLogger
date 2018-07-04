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

#define IMU_LOW_DATA_SIZE 100

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

tIMULowData	imuLowData[IMU_LOW_DATA_SIZE] CCM_SRAM;
uint8_t	uIMURdy = 0;
uint8_t	uIMUCounter = 0;

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
void	IMU_SelfTest(float *destination)
{
	   uint8_t rawData[4];
	   uint8_t selfTest[6];
	   float factoryTrim[6];

	   // Configure the accelerometer for self-test
	   BSP_I2C_Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
	   BSP_I2C_Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	   osDelay(250);  // Delay a while to let the device execute the self-test
	   rawData[0] = BSP_I2C_Read_Byte(MPU6050_ADDRESS, SELF_TEST_X_ACCEL); // X-axis self-test results
	   rawData[1] = BSP_I2C_Read_Byte(MPU6050_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis self-test results
	   rawData[2] = BSP_I2C_Read_Byte(MPU6050_ADDRESS, SELF_TEST_Z_GYRO); // Z-axis self-test results
	   rawData[3] = BSP_I2C_Read_Byte(MPU6050_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
	   // Extract the acceleration test results first
	   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
	   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2 ; // YA_TEST result is a five-bit unsigned integer
	   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 0 ; // ZA_TEST result is a five-bit unsigned integer
	   // Extract the gyration test results first
	   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
	   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
	   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
	   // Process results to allow final comparison with factory set values
	   factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
	   factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
	   factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
	   factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
	   factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
	   factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation

	 //  Output self-test results and factory trim calculation if desired
	 //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
	 //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
	 //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
	 //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

	 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	 // To get to percent, must multiply by 100 and subtract result from 100
	   for (int i = 0; i < 6; i++) {
	     destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
	   }
}

void	IMU_Calibrate(float * dest1, float * dest2)
{
	  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	  uint16_t ii, packet_count, fifo_count;
	  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

	// reset device, reset all registers, clear gyro and accelerometer bias registers
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	  osDelay(100);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
	  osDelay(200);

	// Configure device for bias calculation
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	  osDelay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
	  osDelay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	  BSP_I2C_Write_Byte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	  BSP_I2C_Read_Bytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	  fifo_count = ((uint16_t)data[0] << 8) | data[1];
	  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	  for (ii = 0; ii < packet_count; ii++) {
	    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
	    BSP_I2C_Read_Bytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
	    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
	    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
	    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
	    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
	    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
	    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

	    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	    accel_bias[1] += (int32_t) accel_temp[1];
	    accel_bias[2] += (int32_t) accel_temp[2];
	    gyro_bias[0]  += (int32_t) gyro_temp[0];
	    gyro_bias[1]  += (int32_t) gyro_temp[1];
	    gyro_bias[2]  += (int32_t) gyro_temp[2];

	}
	    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	    accel_bias[1] /= (int32_t) packet_count;
	    accel_bias[2] /= (int32_t) packet_count;
	    gyro_bias[0]  /= (int32_t) packet_count;
	    gyro_bias[1]  /= (int32_t) packet_count;
	    gyro_bias[2]  /= (int32_t) packet_count;

	  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	  else {accel_bias[2] += (int32_t) accelsensitivity;}

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	  data[3] = (-gyro_bias[1]/4)       & 0xFF;
	  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	  data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers; works well for gyro but not for accelerometer
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

	  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
	  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	  BSP_I2C_Read_Bytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	  accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	  BSP_I2C_Read_Bytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
	  accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	  BSP_I2C_Read_Bytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
	  accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

	  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

	  for(ii = 0; ii < 3; ii++) {
	    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	  }

	  // Construct total accelerometer bias, including calculated average accelerometer bias from above
	  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	  accel_bias_reg[1] -= (accel_bias[1]/8);
	  accel_bias_reg[2] -= (accel_bias[2]/8);

	  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	  data[1] = (accel_bias_reg[0])      & 0xFF;
	  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	  data[3] = (accel_bias_reg[1])      & 0xFF;
	  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	  data[5] = (accel_bias_reg[2])      & 0xFF;
	  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	  // Push accelerometer biases to hardware registers; doesn't work well for accelerometer
	  // Are we handling the temperature compensation bit correctly?
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
	//  BSP_I2C_Write_Byte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

	// Output scaled accelerometer biases for manual subtraction in the main program
	   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
	   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

static uint16_t	uSuperCounter = 0;

typedef struct
{
	float	fKoef[3];
}tCalibKoef;

tCalibKoef	cKoef,rKoef;

tCalibKoef	IMU_GyroCalib()
{
	tCalibKoef	koef;
	uint8_t Buf[14];

	memset(&koef,0,sizeof(tCalibKoef));
	memset(Buf,0,sizeof(uint8_t) * 14);

	for(int i = 0;i<100;i++)
	{
	HAL_I2C_Mem_Read(&I2C1Handle, (uint16_t)MPU6050_ADDRESS << 1, ACCEL_XOUT_H, 1, Buf, 14, 0xFFFF);

		gx = (int16_t)Buf[8]<<8 | Buf[9];
		gy = (int16_t)Buf[10]<<8 | Buf[11];
		gz = (int16_t)Buf[12]<<8 | Buf[13];

		koef.fKoef[0]+=gx;
		koef.fKoef[1]+=gy;
		koef.fKoef[2]+=gz;

		osDelay(1);
	}
	koef.fKoef[0]/=100.0f;
	koef.fKoef[1]/=100.0f;
	koef.fKoef[2]/=100.0f;

	return koef;
}

tCalibKoef	IMU_AccelCalib()
{
	tCalibKoef	rData;
	tCalibKoef	offsMin,offsMax;

	uint8_t Buf[14];
	memset(Buf,0,sizeof(uint8_t) * 14);

	uSuperCounter = 0;

	HAL_I2C_Mem_Read(&I2C1Handle, (uint16_t)MPU6050_ADDRESS << 1, ACCEL_XOUT_H, 1, Buf, 14, 0xFFFF);
	ax = (int16_t)Buf[0]<<8 | Buf[1];
	ay = (int16_t)Buf[2]<<8 | Buf[3];
	az = (int16_t)Buf[4]<<8 | Buf[5];

	offsMin.fKoef[0] = ax;
	offsMin.fKoef[1] = ay;
	offsMin.fKoef[2] = az;

	HAL_I2C_Mem_Read(&I2C1Handle, (uint16_t)MPU6050_ADDRESS << 1, ACCEL_XOUT_H, 1, Buf, 14, 0xFFFF);
	ax = (int16_t)Buf[0]<<8 | Buf[1];
	ay = (int16_t)Buf[2]<<8 | Buf[3];
	az = (int16_t)Buf[4]<<8 | Buf[5];

	offsMax.fKoef[0] = ax;
	offsMax.fKoef[1] = ay;
	offsMax.fKoef[2] = az;

	for(int i = 0;i<10000;i++)
	{
		HAL_I2C_Mem_Read(&I2C1Handle, (uint16_t)MPU6050_ADDRESS << 1, ACCEL_XOUT_H, 1, Buf, 14, 0xFFFF);

		ax = (int16_t)Buf[0]<<8 | Buf[1];
		ay = (int16_t)Buf[2]<<8 | Buf[3];
		az = (int16_t)Buf[4]<<8 | Buf[5];

		gx = (int16_t)Buf[8]<<8 | Buf[9];
		gy = (int16_t)Buf[10]<<8 | Buf[11];
		gz = (int16_t)Buf[12]<<8 | Buf[13];

		Gx = (float)(gx - cKoef.fKoef[0]) * G_RES;
		Gy = (float)(gy - cKoef.fKoef[1]) * G_RES;
		Gz = (float)(gz - cKoef.fKoef[2]) * G_RES;

		if(fabsf(Gx)<2.0f && fabsf(Gx)<2.0f && fabsf(Gz) < 2.0f)
		{
			if(ax > offsMax.fKoef[0])offsMax.fKoef[0] = ax;
			if(ay > offsMax.fKoef[1])offsMax.fKoef[1] = ay;
			if(az > offsMax.fKoef[2])offsMax.fKoef[2] = az;

			if(ax < offsMin.fKoef[0])offsMin.fKoef[0] = ax;
			if(ay < offsMin.fKoef[1])offsMin.fKoef[1] = ay;
			if(az < offsMin.fKoef[2])offsMin.fKoef[2] = az;

			uSuperCounter++;
		}
		osDelay(1);

	}
	rData.fKoef[0] = offsMin.fKoef[0] + (offsMax.fKoef[0] - offsMin.fKoef[0]) / 2.0f;
	rData.fKoef[1] = offsMin.fKoef[1] + (offsMax.fKoef[1] - offsMin.fKoef[1]) / 2.0f;
	rData.fKoef[2] = offsMin.fKoef[2] + (offsMax.fKoef[2] - offsMin.fKoef[2]) / 2.0f;


	return rData;
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
		/*
		cKoef = IMU_GyroCalib();
		sprintf(TransmitData, "Gyro done %0.8f %0.8f %0.8f\n", cKoef.fKoef[0], cKoef.fKoef[1],cKoef.fKoef[2]);
		BSP_Usb_SendString(TransmitData);

		osDelay(2000);

		rKoef = IMU_AccelCalib();
		sprintf(TransmitData, "Accel done %0.8f %0.8f %0.8f %6d\n", rKoef.fKoef[0], rKoef.fKoef[1],rKoef.fKoef[2],uSuperCounter);
		BSP_Usb_SendString(TransmitData);
		*/
	}

	else {
		Error_Handler();
	}

	while(1)
	{
		xSemaphoreTake( xIMURdySemaphore, portMAX_DELAY );
		IMU_GetData();
		Devices_SensorsDataRequest();
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Функция запроса измеренных данных с IMU датчика
  * @reval			None
  */
void IMU_GetData(void)
{
	/*uint8_t Buf[14];
	HAL_I2C_Mem_Read(&I2C1Handle, (uint16_t)MPU6050_ADDRESS << 1, ACCEL_XOUT_H, 1, Buf, 14, 0xFFFF);

	ax = (int16_t)Buf[0]<<8 | Buf[1];
	ay = (int16_t)Buf[2]<<8 | Buf[3];
	az = (int16_t)Buf[4]<<8 | Buf[5];

	gx = (int16_t)Buf[8]<<8 | Buf[9];
	gy = (int16_t)Buf[10]<<8 | Buf[11];
	gz = (int16_t)Buf[12]<<8 | Buf[13];
*/


	//Ax = (float)(ax/* - rKoef.fKoef[0]*/) * A_RES;
	//Ay = (float)(ay/* - rKoef.fKoef[1]*/) * A_RES;
	//Az = (float)(az/* - rKoef.fKoef[2]*/) * A_RES;

	//Gx = (float)(gx/* - cKoef.fKoef[0]*/) * G_RES;
	//Gy = (float)(gy/* - cKoef.fKoef[1]*/) * G_RES;
	//Gz = (float)(gz/* - cKoef.fKoef[2]*/) * G_RES;

	/*
	 * CALIBRATION
	 */
	/*if(!uCalibState){
		fCalibAz+=Az;
		uCalibCounter++;
		if(uCalibCounter >= CALIB_REQUEST_VAL){
			uCalibState = 1;
			fCalibAz = fCalibAz / CALIB_REQUEST_VAL;

			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
		}
	}
	else{
		Az = (Az - fCalibAz);
	}*/

	//Gx = Gx - 0.017692864938857;
	//Gy = Gy - 0.012610743859649;
	//Az = Az - 1.0395446271510515;

	//fTemp1X = (fTemp1X + (Gx) * 0.1f) ;
	//fTemp1Y = (fTemp1Y + (Gy) * 0.1f) ;



	//	Send raw data to the filter
/*
	MadgwickAHRSupdateIMU(Gx, Gy, Gz, Ax, Ay, Az);


	//	Calculate angles from quaternions
	roll = atan2(2*(q0*q1+q2*q3), q3*q3-q2*q2-q1*q1+q0*q0);
	pitch = asin(2.0f*(q1*q3-q0*q2));
	yaw = atan2(2*(q0*q3+q1*q2), q1*q1+q0*q0-q3*q3-q2*q2);

	//	Translation of angles from radians to degrees
	yaw = RAD_TO_DEG * yaw;
	pitch = RAD_TO_DEG * pitch;
	roll = RAD_TO_DEG * roll;
*/

	static tSDCardWriteData	writeData;
	memset(&writeData.imuData,0,sizeof(tIMUData));
							writeData.type = E_GYRO;
							writeData.imuData.fAz = Az;
							writeData.imuData.fPitch = Gx;//fTemp1X / M_PI * 180.0f / 72.0f * 90.0f;
							writeData.imuData.fRoll = Gy;//fTemp1Y / M_PI * 180.0f / 72.0f * 90.0f;


	BSP_SDCard_WriteSensorsData(&writeData);
/*
	sprintf(TransmitData, "aX: %0.8f aY: %0.8f aZ: %0.8f, Gx: %0.8f, Gy: %0.8f Gz: %0.8f \n",Ax,Ay, Az, Gx, Gy,Gz);
	//sprintf(TransmitData, "aX: %8d aY: %8d aZ: %8d, Gx: %8d, Gy: %8d Gz: %8d \n",ax,ay,az,gx,gy,gz);
	BSP_Usb_SendString(TransmitData);
*/

	//IMU_InsertDataInQuery(&currentIMUData);

	//sprintf(TransmitData, "%f\r\n", fCalibAz * 10.0f);
	//BSP_Usb_SendString(TransmitData);

	//sprintf(TransmitData, "q0: %0.1d, q1: %0.1d, q2: %0.1d, q3: %0.1d\r\n", (int8_t)(q0*10), (int8_t)(q1*10), (int8_t)(q2*10), (int8_t)(q3*10));
	//HAL_UART_Transmit(&huart1, (uint8_t*)TransmitData, strlen(TransmitData), 0xFFFF);
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

	Gx = (float)(gx/* - cKoef.fKoef[0]*/) * G_RES;
	Gy = (float)(gy/* - cKoef.fKoef[1]*/) * G_RES;
	Gz = (float)(gz/* - cKoef.fKoef[2]*/) * G_RES;

	/*
	 *
	 * Частота работы IMU - 1kHz
	 * Пишем каждый 100-й запрос, получаем запись 10 раз в секунду
	 */
	if(uIMUCounter<IMU_LOW_DATA_SIZE)
	{
		imuLowData[uIMUCounter].fAccel[0] = Ax;
		imuLowData[uIMUCounter].fAccel[1] = Ay;
		imuLowData[uIMUCounter].fAccel[2] = Az;
		imuLowData[uIMUCounter].fGyro[0] = Gx;
		imuLowData[uIMUCounter].fGyro[1] = Gy;
		imuLowData[uIMUCounter].fGyro[2] = Gz;

		uIMUCounter++;
	}
	else
	{
		xSemaphoreGiveFromISR( xIMURdySemaphore, &xTaskWoken );
		if( xTaskWoken == pdTRUE){
			taskYIELD();
		}
		uIMUCounter = 0;
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
