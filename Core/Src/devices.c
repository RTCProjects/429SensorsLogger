/*----------------------------------------------------------------------------------------------------*/
/**
  * @file    devices.c Модуль обработки данных от устройств на шине CAN
  * @brief   
**/
/*----------------------------------------------------------------------------------------------------*/
#include <string.h>
#include "devices.h"
#include "bsp_sdcard.h"
#include "bsp_canbus.h"

static const uint8_t	sensorRequestArray[] = {0x00};
static tDistArrayData	distArrayData;	//данные камеры

tSensorLowData	sensorsLowData[100] CCM_SRAM;
uint16_t		sensorsCounter = 0;
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  Инициализация модуля устройств
  * @retval None
  */
void Devices_Init()
{
	sensorsCounter = 0;

	distArrayData.pRxData = (float*)pvPortMalloc(sizeof(float) * DATA_ARRAY_SIZE);
	if(!distArrayData.pRxData)
		Error_Handler();
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  Функция анализа полученных из линии CAN данных
	*	Формат пакета данных	
	* | Byte0| Byte1|Byte2|Byte3|Byte4|Byte5|Byte6|Byte7|
	* | Type | Stat |Lidar		|Sonar		|Radar		|
	* 	Type - тип
	*		Stat - байт флагов статуса
	*		Lidar - лидар
	*		Sonar -	сонар
	*		Radar - радар
	* @param *rxData: указатель на входящее сообщение из CAN линии
  * @retval None
  */
void	Devices_PackageAnalysis(TQueryCanRxData	*rxData)
{
	uint16_t	canID = rxData->RxHeader.StdId;
	uint8_t		logicNumber = 0;
	uint8_t		typeNumber = 0;
	
	//проверка на корректность ID устройства. Диапазон устройств с сенсорами на CAN линии - 0x100 - 0x10F
	if(canID >= 0x100 && canID <= 0x10F)
	{
		logicNumber = canID & 0x0F;		//логический номер устройства на шине, закодирован в младших 4 битах CanID
		typeNumber = rxData->Data[0];	//логический ттип устройства - согласно протоколу обмена это 1 байт кадра данных
		
		static tSDCardWriteData		writeData;
		
		writeData.type = E_RANGEFINDER;

		memset(&writeData.sensorsData,0,sizeof(tSensorData));

		writeData.sensorsData.devID = logicNumber;
		writeData.sensorsData.devType = typeNumber;
		writeData.sensorsData.uFlags.flags = rxData->Data[1];

		switch(typeNumber)
		{
			case TYPE_1:
			{
				memset(&writeData.sensorsData,0,sizeof(tSensorData));

				writeData.sensorsData.devID = logicNumber;
				writeData.sensorsData.devType = typeNumber;
				writeData.sensorsData.uFlags.flags = rxData->Data[1];

				memcpy(writeData.sensorsData.sensorValue,rxData->Data + 2,sizeof(uint16_t) * 3);
				BSP_SDCard_WriteSensorsData(&writeData);
			}break;
			
			case TYPE_2:
			{
				memset(&writeData.sensorsData,0,sizeof(tSensorData));

				writeData.sensorsData.devID = logicNumber;
				writeData.sensorsData.devType = typeNumber;
				writeData.sensorsData.uFlags.flags = rxData->Data[1];
				
				memcpy(writeData.sensorsData.sensorValue,rxData->Data + 2,sizeof(uint16_t) * 2);
				BSP_SDCard_WriteSensorsData(&writeData);
			}break;
		}

	}
	//проверка на корректность ID устройства. Диапазон устройств с камерами на CAN линии - 0x110 - 0x11F
	if(canID >= 0x110 && canID <= 0x11F)
	{
		uint32_t	uHeaders[2];

		uHeaders[0] = (rxData->Data[3]<<24)|(rxData->Data[2]<<16)|(rxData->Data[1]<<8)|rxData->Data[0];
		uHeaders[1] = (rxData->Data[7]<<24)|(rxData->Data[6]<<16)|(rxData->Data[5]<<8)|rxData->Data[4];

		if(uHeaders[0] == 0x7FFFFFFF && uHeaders[1] == 0xFFFFFFFF)
		{
			//поймали заголовочное сообщение
			distArrayData.msgCounter = 0;
		}
		else if(distArrayData.msgCounter < DATA_ARRAY_SIZE)
		{
			memcpy(distArrayData.pRxData,rxData->Data,sizeof(float));
			memcpy(distArrayData.pRxData + distArrayData.msgCounter + 1,rxData->Data + 4,sizeof(float));

			distArrayData.msgCounter += 2;
		}
		else
		{
			//закончили получать массив
			distArrayData.msgCounter = 0;
		}
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  Функция отправки запроса измеренных данных
  * @retval None
  */
void	Devices_SensorsDataRequest()
{
	/*for(uint8_t i = 0;i<=4;i++){
		BSP_CanBus_SendData(RX_SENSOR + i,(uint8_t*)sensorRequestArray,0);
			osDelay(1);
	}*/
	BSP_CanBus_SendData(RX_SENSOR ,(uint8_t*)sensorRequestArray,0);
		osDelay(1);
	BSP_CanBus_SendData(RX_SENSOR + 1 ,(uint8_t*)sensorRequestArray,0);
		osDelay(1);
	BSP_CanBus_SendData(RX_SENSOR + 2,(uint8_t*)sensorRequestArray,0);
		osDelay(1);
	BSP_CanBus_SendData(RX_SENSOR + 3,(uint8_t*)sensorRequestArray,0);
		osDelay(1);
	BSP_CanBus_SendData(RX_SENSOR + 4,(uint8_t*)sensorRequestArray,0);
		osDelay(1);
}
/*----------------------------------------------------------------------------------------------------*/
