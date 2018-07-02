#ifndef _CANBUS_H
#define _CANBUS_H


#include "stm32f4xx_hal.h"
#include "cmsis_os.h"


#define CANBUS2_RXD_QUEUE_LENGTH	32
#define CANBUS2_TXD_QUEUE_LENGTH	8

#define CANBUS2_TASK_PRIO    ( tskIDLE_PRIORITY + 2 )

typedef struct
{
	CAN_TxHeaderTypeDef   TxHeader;
	uint8_t								Data[8];
}TQueryCanTxData;

typedef struct
{
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t								Data[8];
}TQueryCanRxData;

void BSP_CanBus_Init(void);
void BSP_CanBus_SendData(uint16_t	StdId,uint8_t 	*pData,uint16_t Size);
#endif
