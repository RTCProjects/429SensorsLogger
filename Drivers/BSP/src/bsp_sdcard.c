/*----------------------------------------------------------------------------------------------------*/
/**
  * @file    bsp_driver_spi.c ������� ��� ������ � SD ������ �� SPI
  * @brief   This file includes a generic uSD card driver.
**/
/*----------------------------------------------------------------------------------------------------*/
#include "bsp_sdcard.h"
#include "bsp_usb.h"
#include "bsp_rtc.h"
#include "fatfs.h"
#include "IMU.h"

#include <string.h>

static	char usbOutputBuffer[128];

static uint8_t				startBtnState,startBtnPress;
static SD_CardInfo			sdInfo;
static FATFS				fileSystem;

SPI_HandleTypeDef 		spiSDHandle;
osThreadId 				usbTaskSdCardHandle;
xQueueHandle 			xSDCardDataWriteQueue;

uint32_t				uFilesCount;

void	BSP_SDCard_Task(void const * argument);
void 	BSP_SDCard_StartBtnState(void);
void 	BSP_SDCard_CreateDefaultFolders(void);

extern xQueueHandle 			xIMUDataQueue;
extern tIMULowData				imuLowData[100] CCM_SRAM;
extern tSensorLowData			sensorsLowData[100] CCM_SRAM;
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
void	BSP_SDCard_Init()
{	

	xSDCardDataWriteQueue = xQueueCreate(SDCARD_WRITE_QUEUE_SIZE, sizeof(tSDCardWriteData));
	
	osThreadDef(SDCardTask, BSP_SDCard_Task, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE + 0x400);
	usbTaskSdCardHandle = osThreadCreate(osThread(SDCardTask), NULL);
	
	uFilesCount = 0;
	startBtnState = 0;
	startBtnPress = 0;


}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ������� ������������� ����������� SPI
  * @param	baudratePrescaler: ������������ �������� SPI
  * @retval None
  */
void BSP_SDCard_SPIInit(uint32_t	baudratePrescaler)
{

	spiSDHandle.Instance               = SPI4;
	spiSDHandle.Init.BaudRatePrescaler = baudratePrescaler;
	spiSDHandle.Init.Direction         = SPI_DIRECTION_2LINES;
	spiSDHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	spiSDHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
	spiSDHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	spiSDHandle.Init.CRCPolynomial     = 7;
	spiSDHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
	spiSDHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	spiSDHandle.Init.NSS               = SPI_NSS_HARD_OUTPUT;
	spiSDHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
	spiSDHandle.Init.Mode = SPI_MODE_MASTER;

	if(HAL_SPI_Init(&spiSDHandle) != HAL_OK){
		/* Initialization Error */
		Error_Handler();
	  }
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ������� ��������������� ����������� SPI
  * @retval None
  */
void BSP_SDCardSPIDeInit()
{
	HAL_SPI_DeInit(&spiSDHandle);
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ������� �������� ������ � ������� ��� ������ �� SD �����
  * @param 	Data: ��������� �� ��������� ������ � ��������
  * @retval SD status
  */
void	BSP_SDCard_WriteSensorsData(tSDCardWriteData	*Data)
{
	if(xSDCardDataWriteQueue!=0)
		xQueueSendToBack(xSDCardDataWriteQueue,Data,0);
}
/*----------------------------------------------------------------------------------------------------*/
void 	BSP_SDCard_StartBtnState()
{

		if(HAL_GPIO_ReadPin(GPIOH,GPIO_PIN_12) == GPIO_PIN_SET){
			if(startBtnPress == 1)return;
			else startBtnPress = 1;

			if(startBtnState == 0)startBtnState = 1;
		}
		else
		{
			startBtnPress = 0;
		}

}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	������� ��������� ���� ������ � ������ �� SD �����
	* @reval	��������� �� ��������� ���������� ����� ������ �����
	* @note		Memory allocation
  */
tSDCardFileNames	*BSP_SDCard_GetNewFileName()
{
	static tSDCardFileNames	logFileNames;

	logFileNames.imusensFilename = (char*)pvPortMalloc(sizeof(char) * SDCARD_FILENAME_MAX_LEN);
	logFileNames.sensorsFilename = (char*)pvPortMalloc(sizeof(char) * SDCARD_FILENAME_MAX_LEN);

	if(!logFileNames.imusensFilename || !logFileNames.imusensFilename)
		return 0;

	else{
		uint16_t	uFilesInDir = 0;

		uFilesInDir = BSP_SDCard_GetFileNumbers(DIRNAME_SENSORS);
		sprintf(logFileNames.sensorsFilename,"senslog%d.txt",uFilesInDir + 1);

		uFilesInDir = BSP_SDCard_GetFileNumbers(DIRNAME_IMU);
		sprintf(logFileNames.imusensFilename,"imulog%d.txt", uFilesInDir + 1);

		return &logFileNames;
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	������� ������ �� ����� ������ �� ��������
	* @param 	fileName: ��� ����� ��� ������ �� ����
	* @param	pData: ��������� ��������� ������ ��������
	* @reval	None
  */
static uint16_t	uRangefinders[5];
static uint16_t	uRadar;

void	BSP_SDCard_RangefinderWrite(const char *fileName,tSDCardWriteData *pData)
{
	FRESULT 		fResult;
	FIL 			fFile;
	char			strFmtToFile[100];

	if(!pData || pData->type!=E_RANGEFINDER)
		return;

	if(!fileName)
		return;



	//TO-DO write to disk
	/*fResult = f_open(&fFile,fileName,FA_OPEN_APPEND|FA_WRITE);
	if(fResult == FR_OK)
	{
		for(int i = 0;i<100;i++)
		{
			sprintf(strFmtToFile,"%5d %5d %5d %5d %5d %5d\n",sensorsLowData[i].ulRangefinder[0],sensorsLowData[i].ulRangefinder[1],sensorsLowData[i].ulRangefinder[2],sensorsLowData[i].ulRangefinder[3],sensorsLowData[i].ulRangefinder[4],sensorsLowData[i].ulRadar);
			f_printf(&fFile,"%s",strFmtToFile);
		}
		f_close(&fFile);

		BSP_Usb_SendString(strFmtToFile);
	}
	else
	{
		BSP_Usb_SendString("Lidar/Radar data write error\n");
	}
	 */

	uint8_t	devID = pData->sensorsData.devID;

	if(devID>=0 && devID<=4){
		if(pData->sensorsData.devType == TYPE_1){
			uRangefinders[devID] = pData->sensorsData.sensorValue[2];
		}
		if(pData->sensorsData.devType == TYPE_2){
			uRangefinders[devID] = pData->sensorsData.sensorValue[1];
			uRadar = pData->sensorsData.sensorValue[0];
		}

		if(devID == 4)
		{

			sprintf(strFmtToFile,"%5d %5d %5d %5d %5d %5d\n",uRangefinders[0],uRangefinders[1],uRangefinders[2],uRangefinders[3],uRangefinders[4],uRadar);
				//TO-DO write to disk
			fResult = f_open(&fFile,fileName,FA_OPEN_APPEND|FA_WRITE);
			if(fResult == FR_OK)
			{
				f_printf(&fFile,"%s",strFmtToFile);
				f_close(&fFile);

				BSP_Usb_SendString(strFmtToFile);
			}
			else
			{
				BSP_Usb_SendString("Lidar/Radar data write error\n");
			}
		}
	}

}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	������� ������ �� ����� ������ �� ���������
	* @param 	fileName: ��� ����� ��� ������ �� ����
	* @param	pData: ��������� �� ��������� ������ ��������
	* @reval	None
  */
void	BSP_SDCard_IMUSensorWrite(const char	*fileName,tSDCardWriteData *pData)
{
	FRESULT 		fResult;
	FIL 			fFile;
	char			strFmtToFile[100];

	if(!pData || pData->type!=E_GYRO)
		return;
	if(!fileName)
		return;

	//sprintf(strFmtToFile,"IMU Az:%f Gx:%f Gy:%f\n",pData->imuData.fAccel[0],pData->imuData.fAccel[1],pData->imuData.fAccel[2]);

	fResult = f_open(&fFile,fileName,FA_OPEN_APPEND|FA_WRITE);
	if(fResult == FR_OK)
	{
		f_printf(&fFile,"%s",strFmtToFile);
		f_close(&fFile);

		BSP_Usb_SendString(strFmtToFile);
	}
	else
	{
		BSP_Usb_SendString("IMU data write error\n");
	}

}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	�������� ����� ��� ������ � SD ������
	* @param	argument: ��������� ������ FreeRTOS 
	* @reval	None
  */
static tSDCardWriteData	sdWriteData;

void	BSP_SDCard_Task(void const * argument)
{
	/*
	������������� ��������
	������������ �������� �������
	�������� ����������
	*/
	FRESULT 		fResult;
	FIL 			fFile;
	portBASE_TYPE 	xStatus;
	//tSDCardFileNames			*logFileNames = 0;

	
	FATFS_LinkDriver(&SD_Driver, SDPath);
	BSP_SD_Init();

	if(f_mount(&fileSystem, "", 0) != FR_OK){
			Error_Handler();
	}

	BSP_SD_GetCardInfo(&sdInfo);
	/*logFileNames = BSP_SDCard_GetNewFileName();

	if(!logFileNames){
		Error_Handler();
	}
	 */

	fResult = f_open(&fFile,"imulog.ini",FA_OPEN_APPEND|FA_WRITE);
	if(fResult == FR_OK){
		f_printf(&fFile,"\r\n--------------------START--------------------\r\n");
		f_close(&fFile);
	}

	fResult = f_open(&fFile,"senslog.ini",FA_OPEN_APPEND|FA_WRITE);
	if(fResult == FR_OK){
		f_printf(&fFile,"\r\n--------------------START--------------------\r\n");
		f_close(&fFile);
	}


	while(1)
	{
		xStatus=xQueueReceive(xSDCardDataWriteQueue, &sdWriteData, portMAX_DELAY);

		if (xStatus == pdPASS)
		{
			//RTC_TimeTypeDef	*rtcTime = BSP_RTC_GetTime();

			switch(sdWriteData.type)
			{
				case E_RANGEFINDER:
				{


					fResult = f_open(&fFile,"senslog.ini",FA_OPEN_APPEND|FA_WRITE);
					if(fResult == FR_OK){
						if(sdWriteData.sensorsData.devType == TYPE_1){
							f_printf(&fFile,"%d %5d %5d %5d\n",

															  sdWriteData.sensorsData.devID,
															  sdWriteData.sensorsData.sensorValue[0],
															  sdWriteData.sensorsData.sensorValue[1],
															  sdWriteData.sensorsData.sensorValue[2]);
						}
						if(sdWriteData.sensorsData.devType == TYPE_2){
							f_printf(&fFile,"%d %5d %5d %5d\n",

															  sdWriteData.sensorsData.devID,
															  sdWriteData.sensorsData.sensorValue[0],
															  sdWriteData.sensorsData.sensorValue[1],0);


						}

						f_close(&fFile);

						sprintf(usbOutputBuffer,"%d %5d %5d %5d\n",sdWriteData.sensorsData.devID,
																   sdWriteData.sensorsData.sensorValue[0],
																   sdWriteData.sensorsData.sensorValue[1],
																   sdWriteData.sensorsData.sensorValue[2]);

						BSP_Usb_SendString(usbOutputBuffer);

						if(sdWriteData.sensorsData.devID == 4)
							BSP_Usb_SendString("--------------------------------------\n");
					}
					else{
						BSP_Usb_SendString("write error sensors\n");
					}


				/*BSP_SDCard_RangefinderWrite("senslog.ini",&sdWriteData);
				 */
				}break;

				case E_GYRO:
				{
					fResult = f_open(&fFile,"imulog.ini",FA_OPEN_APPEND|FA_WRITE);
					if(fResult == FR_OK){

						char	fmtStr[100];
						sprintf(fmtStr,"IMU Az:%f Gx:%f Gy:%f\n",sdWriteData.imuData.fAz,sdWriteData.imuData.fPitch,sdWriteData.imuData.fRoll);
						f_printf(&fFile,"%s",fmtStr);

						/*for(int i = 0;i<10;i++)
						{
							//sprintf(fmtStr,"IMU Ax%f Gx:%f Gy:%f\n",sdWriteData.imuData.fAz,sdWriteData.imuData.fPitch,sdWriteData.imuData.fRoll);
							sprintf(fmtStr,"%f;%f;%f\n",imuLowData[i].fAccel[2],imuLowData[i].fGyro[0],imuLowData[i].fGyro[1]);
							f_printf(&fFile,"%s",fmtStr);
						}*/

						f_close(&fFile);

						//BSP_Usb_SendString(fmtStr);
					}
					else
						BSP_Usb_SendString("write imu error\n");
				}break;
			}
		}
	}
}
/*----------------------------------------------------------------------------------------------------*/
/*
 * @brief	������� ��������� ���������� ������ � �������� ��������
 * @param	strPath:�������
 * @reval	���������� ������
 */
uint16_t	BSP_SDCard_GetFileNumbers(char	*strPath)
{
	uint16_t	uResult = 0;

	FRESULT fResult;
	DIR		fDir;
	FILINFO	fInfo;

	//��������� ������
	fResult = f_opendir(&fDir,strPath);

	if(fResult == FR_OK){
		for (;;){
			fResult = f_readdir(&fDir, &fInfo);

		if(fResult != FR_OK || fInfo.fname[0] == 0)
		   break;
		else
			uResult++;
		}
		f_closedir(&fDir);
	}
	return uResult;
}
/*----------------------------------------------------------------------------------------------------*/
/*
 * @brief	������� �������� � �������� ��������� ���������� ��� ������ ��� ������
 * @reval	���������� ������
 */
void BSP_SDCard_CreateDefaultFolders()
{
	FRESULT fResult;
	FILINFO fInfo;

	fResult = f_stat(DIRNAME_SENSORS,&fInfo);
	if(fResult != FR_OK)
		f_mkdir(DIRNAME_SENSORS);

	fResult = f_stat(DIRNAME_IMU,&fInfo);
	if(fResult != FR_OK)
		f_mkdir(DIRNAME_IMU);

}

/*----------------------------------------------------------------------------------------------------*/
/*
 * @brief	LL ������� ��������� ������ SPI
 * @reval	None
 */
static void SPIx_Error()
{
	//HAL_SPI_DeInit(&spiSDHandle);
	//BSP_SDCard_SPIInit();
}
/*----------------------------------------------------------------------------------------------------*/
/*
 * @brief	LL ������� ������/�������� ������ SPI
 * @reval	None
 */
static void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth)
{
  HAL_StatusTypeDef status = HAL_OK;

  	  status = HAL_SPI_TransmitReceive(&spiSDHandle, (uint8_t*) DataIn, DataOut, DataLegnth, 5000);

  if(status != HAL_OK){
    SPIx_Error();
  }
}
/*----------------------------------------------------------------------------------------------------*/
/*
 * @brief	LL ������� ������������� SDCard �� SPI
 * @reval	None
 */
void    SD_IO_Init(void)
{
	BSP_SDCard_SPIInit(SPI_BAUDRATEPRESCALER_256);

	for (int counter = 0; counter <= 9; counter++){
	    SD_IO_WriteByte(0xFF);
	}
	BSP_SDCard_SPIInit(SPI_BAUDRATEPRESCALER_8);
}
/*----------------------------------------------------------------------------------------------------*/
/*
 * @brief	LL ������� ����������� ����������
 * @param	state: CS High/Low
 * @reval	None
 */
void    SD_IO_CSState(uint8_t state)
{
	if(state)
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
}
/*----------------------------------------------------------------------------------------------------*/
/*
 * @brief	LL ������� ������/�������� ������ SPI
 * @param	DataIn: ��������� �� ����� �������� �����
 * @param	DataOut: ��������� �� ����� ����� ��� ��������
 * @param	DataLength: ������ ������
 * @reval	None
 */
void   SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
	SPIx_WriteReadData(DataIn, DataOut, DataLength);
}
/*----------------------------------------------------------------------------------------------------*/
/*
 * @brief	LL ������� ������/������ ����� �� SPI
 * @param	Data: ��������� �� ����� �������� �����
 * @reval	����������� ����
 */
uint8_t   SD_IO_WriteByte(uint8_t Data)
{
	uint8_t tmp;
		SPIx_WriteReadData(&Data,&tmp,1);
	return tmp;
}
/*----------------------------------------------------------------------------------------------------*/

