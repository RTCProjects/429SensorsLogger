/*----------------------------------------------------------------------------------------------------*/
/**
  * @file    bsp_canbus.c Модуль для работы с шиной CAN
  * @brief   
**/
/*----------------------------------------------------------------------------------------------------*/
#include <string.h>
#include "bsp_canbus.h"
#include "bsp_usb.h"
#include "devices.h"

CAN_HandleTypeDef  CanHandle;

xQueueHandle xCanbus2RxQueue;  // очередь входящих сообщений
xQueueHandle xCanbus2TxQueue;		//очередь исходящих сообщений

osThreadId canbusRxTaskHandle;
osThreadId canbusTxTaskHandle;

static void BSP_CanBus_ReceiveCallback(TQueryCanRxData*);
static void BSP_CanBus_RxTask(void const *);
static void BSP_CanBus_TxTask(void const *);
static void BSP_CanBus_SendMessage(TQueryCanTxData*);
	
TQueryCanTxData	canTxMessage;
TQueryCanRxData	canRxMessage;

/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Инициализация BSP CANBus
	* @param	None
	* @reval	None
	*/
void BSP_CanBus_Init()
{
	CAN_FilterTypeDef  sFilterConfig;

	CanHandle.Instance = CAN1;

  CanHandle.Init.TimeTriggeredMode 				= DISABLE;
  CanHandle.Init.AutoBusOff 					= DISABLE; //ENABLE;		//DISABLE;
  CanHandle.Init.AutoWakeUp 					= DISABLE; //DISABLE;
  CanHandle.Init.AutoRetransmission 			= DISABLE; //ENABLE;		//ENABLE;
  CanHandle.Init.ReceiveFifoLocked 				= DISABLE;
  CanHandle.Init.TransmitFifoPriority 			= DISABLE; //DISABLE;
  CanHandle.Init.Mode = CAN_MODE_NORMAL;
  CanHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
  /* CAN 125kBit
	CanHandle.Init.TimeSeg1 = CAN_BS1_13TQ;
  CanHandle.Init.TimeSeg2 = CAN_BS2_2TQ;
  CanHandle.Init.Prescaler = 21;*/
	
	/* CAN 1Mbit
			Warning Sample point at 85.7%!
	
	CanHandle.Init.TimeSeg1 = CAN_BS1_11TQ;
  CanHandle.Init.TimeSeg2 = CAN_BS2_2TQ;
  CanHandle.Init.Prescaler = 3;	
	*/
	/*
	 CAN 500kBit
			Warning Sample point at 85.7%!
	*/
	CanHandle.Init.TimeSeg1 = CAN_BS1_11TQ;
  CanHandle.Init.TimeSeg2 = CAN_BS2_2TQ;
  CanHandle.Init.Prescaler = 6;	
	
	
	
  if (HAL_CAN_Init(&CanHandle) != HAL_OK)
  {
    /* Initialization Error */
		Error_Handler();
  }

	sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK){
		Error_Handler();
  }
	
  if (HAL_CAN_Start(&CanHandle) != HAL_OK){
    Error_Handler();
  }
	
  if (HAL_CAN_ActivateNotification(&CanHandle, CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK){
    Error_Handler();
  }
	

	xCanbus2RxQueue=xQueueCreate(CANBUS2_RXD_QUEUE_LENGTH, sizeof(TQueryCanRxData));
	xCanbus2TxQueue=xQueueCreate(CANBUS2_TXD_QUEUE_LENGTH, sizeof(TQueryCanTxData));

	osThreadDef(canbusRxTask, BSP_CanBus_RxTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE + 0x200);
  canbusRxTaskHandle = osThreadCreate(osThread(canbusRxTask), NULL);
	
	osThreadDef(canbusTxTask, BSP_CanBus_TxTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE + 0x200);
  canbusTxTaskHandle = osThreadCreate(osThread(canbusTxTask), NULL);
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Функция отправки сообщения в линию CAN
	* @param	TxMessage: указатель на структуру данных для отправки
	* @reval	None
	*/
void BSP_CanBus_SendMessage(TQueryCanTxData *TxMessage)
{
	static uint32_t	uMailBox = 0;

	if(HAL_CAN_AddTxMessage(&CanHandle,&TxMessage->TxHeader,TxMessage->Data,&uMailBox)!= HAL_OK)
		BSP_Usb_SendString("429CAN Transmit error\r\n");
	else
		vTaskSuspend(canbusTxTaskHandle);
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Функция формирования пакета данных для отправки в линию CAN
	* @param	stdID: 11bit ID пакета данных
	* @param 	*pData: указатель на пакет данных для отправки
	* @param	Size: размер пакета для отправки
	* @reval	None
	*/
void BSP_CanBus_SendData(uint16_t	stdID,uint8_t 	*pData,uint16_t Size)
{
	uint16_t	dataSize = Size;
	
	static TQueryCanTxData TxMessage;
						   TxMessage.TxHeader.StdId = stdID;
	
	do{
		if(dataSize > 8)TxMessage.TxHeader.DLC = 8; else TxMessage.TxHeader.DLC = dataSize;
			memcpy(TxMessage.Data,pData,sizeof(uint8_t) * TxMessage.TxHeader.DLC);
			
			xQueueSendToBack(xCanbus2TxQueue,&TxMessage,0);
		
		dataSize -= TxMessage.TxHeader.DLC ;
		pData += TxMessage.TxHeader.DLC ;
	}
	while(dataSize > 0);		
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Поток передачи сообщений из очереди в CAN линию
	* @param	argument: параметры потока FreeRTOS 
	* @reval	None
	*/
static void BSP_CanBus_RxTask(void const *argument)
{
	portBASE_TYPE 			  xStatus;
	static TQueryCanRxData    RxMessage;

	while (1) 
	{
		xStatus=xQueueReceive(xCanbus2RxQueue, &RxMessage, portMAX_DELAY);
		if (xStatus == pdPASS)
		{
			Devices_PackageAnalysis(&RxMessage);
		}
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Поток приема сообщений из CAN линии
	* @param	argument: параметры потока FreeRTOS 
	* @reval	None
	*/
static void BSP_CanBus_TxTask(void const *arg)
{
	portBASE_TYPE xStatus;
	TQueryCanTxData        TxMessage;
	
		while(1){
			xStatus=xQueueReceive(xCanbus2TxQueue, &TxMessage, portMAX_DELAY);
			
			if (xStatus == pdPASS) {
				BSP_CanBus_SendMessage(&TxMessage);
			}
		}
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Rx FIFO 0 message pending callback.
	* @param	CanHandle: pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
	* @reval	None
	*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{ 
	if(CanHandle->Instance == CAN1)
	{
		if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &canRxMessage.RxHeader, canRxMessage.Data) != HAL_OK){
			Error_Handler();
		}
		BSP_CanBus_ReceiveCallback(&canRxMessage);
	}	
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Обработчик прерывания по приему сообщения из линии CAN
	* @param	pRxMessage: указатель на структуру данных принятого сообщения
	* @reval	None
	*/
static void BSP_CanBus_ReceiveCallback(TQueryCanRxData	*pRxMessage)
{
	static portBASE_TYPE xHigherPriorityTaskWoken;
	
	xHigherPriorityTaskWoken = pdFALSE;
	xQueueSendToBackFromISR(xCanbus2RxQueue, pRxMessage, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken != pdFALSE) {
    // Макрос, выполняющий переключение контекста.
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  }
}
/*----------------------------------------------------------------------------------------------------*/
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		xTaskResumeFromISR( canbusTxTaskHandle );
	}
}
/*----------------------------------------------------------------------------------------------------*/
