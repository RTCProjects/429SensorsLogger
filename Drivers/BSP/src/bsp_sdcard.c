/*----------------------------------------------------------------------------------------------------*/
/**
  * @file    bsp_driver_spi.c ������� ��� ������ � SD ������ �� SPI
  * @brief   This file includes a generic uSD card driver.
**/
/*----------------------------------------------------------------------------------------------------*/
#include <skif.h>
#include "bsp_sdcard.h"
#include "bsp_usb.h"
#include "bsp_rtc.h"
#include "fatfs.h"
#include <string.h>
/*----------------------------------------------------------------------------------------------------*/
static char usbOutputBuffer[300];

static uint8_t				startBtnState,startBtnPress;
static SD_CardInfo			sdInfo;
static FATFS				fileSystem;

SPI_HandleTypeDef 		spiSDHandle;
osThreadId 				usbTaskSdCardHandle;
xQueueHandle 			xSDCardDataWriteQueue;

uint32_t				uFilesCount;

void	BSP_SDCard_Task(void const * argument);
void 	BSP_SDCard_CreateDefaultFolders(void);

xSemaphoreHandle xSDWriteProcessSemaphore;
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
void	BSP_SDCard_Init()
{	

	xSDCardDataWriteQueue = xQueueCreate(SDCARD_WRITE_QUEUE_SIZE, sizeof(tSDCardWriteData));
	vSemaphoreCreateBinary(xSDWriteProcessSemaphore);
	
	osThreadDef(SDCardTask, BSP_SDCard_Task, /*osPriorityAboveNormal*/ osPriorityRealtime, 0, configMINIMAL_STACK_SIZE + 0x400);
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
		_Error_Handler("bsp_sdcard.c",74);
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
/**
	* @brief	������� ��������� ���� ������ � ������ �� SD �����
	* @reval	��������� �� ��������� ���������� ����� ������ �����
	* @note		Memory allocation
  */
char	*BSP_SDCard_GetNewFileName()
{
	static char	strFileName[16];

	uint16_t	uFilesInDir = 0;

	uFilesInDir = BSP_SDCard_GetFileNumbers(DIRNAME_SENSORS);
	sprintf(strFileName,"senslog%d.ini",uFilesInDir + 1);

	return strFileName;
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	������ ������ �� ������ �� SD �����
	* @param	argument: ��������� ������ FreeRTOS 
	* @reval	None
  */
void	BSP_SDCard_StartWrite()
{
	xSemaphoreGive(xSDWriteProcessSemaphore);
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	�������� ����� ��� ������ � SD ������
	* @param	argument: ��������� ������ FreeRTOS
	* @reval	None
  */
extern tSDCardWriteData	accumData[IMU_LOW_DATA_SIZE] CCM_SRAM;
void	BSP_SDCard_Task(void const * argument)
{
	/*
	������������� ��������
	������������ �������� �������
	�������� ����������
	*/
	char			*pFileName;
	FRESULT 		fResult;
	FIL 			fFile;

	FATFS_LinkDriver(&SD_Driver, SDPath);
	BSP_SD_Init();

	if(f_mount(&fileSystem, "", 0) != FR_OK){
		_Error_Handler("bsp_sdcard.c",147);
	}

	BSP_SD_GetCardInfo(&sdInfo);

	pFileName = BSP_SDCard_GetNewFileName();
	if(!pFileName)
		return;

	fResult = f_open(&fFile,pFileName,FA_OPEN_APPEND|FA_WRITE);
	if(fResult == FR_OK){
		f_printf(&fFile,"\r\n--------------------START--------------------\r\n");
		f_close(&fFile);
	}

	xSemaphoreTake( xSDWriteProcessSemaphore, portMAX_DELAY );

	while(1)
	{
		xSemaphoreTake( xSDWriteProcessSemaphore, portMAX_DELAY );
		{
			fResult = f_open(&fFile,pFileName,FA_OPEN_APPEND|FA_WRITE);
			if(fResult == FR_OK){
				Devices_LedToggle();

				for(int i = 0;i<IMU_LOW_DATA_SIZE;i++)
				{
					sprintf(usbOutputBuffer,"%5d, %5d, %5d, %5d, %5d, %2d, %5d, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %s, %s\n",
							accumData[i].sensorsData.ulCenterLidarDistance,
							accumData[i].sensorsData.ulLeftLidarDistance,
							accumData[i].sensorsData.ulRightLidarDistance,
							accumData[i].sensorsData.ulFrontLidarDistance,
							accumData[i].sensorsData.ulRadarDistance,
							accumData[i].uRadarVspeed,
							accumData[i].sensorsData.ulSonarDistance,
							accumData[i].imuData.fPitch,
							accumData[i].imuData.fRoll,
							accumData[i].imuData.fAz,
							accumData[i].fAltitude,
							accumData[i].fAltitude2,
							accumData[i].strNMEAPosition,
							accumData[i].strNMEAVelocity
							);

					f_printf(&fFile,"%s",usbOutputBuffer);
				}
				f_close(&fFile);
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

