#ifndef _BSP_MANCHESTER_H
#define _BSP_MANCHESTER_H

#define TX_PACK_SIZE		8

#define TX_SIZE					21
#define RX_SIZE					21

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

typedef union
{
	struct
	{
		uint16_t	Byte:8;				//Байт для отправки
		uint16_t	fTx:1;				//Флаг отправки
		uint16_t	fPre:1;				//Отправка преамбулы
		uint16_t	fBitPos:5;		//текущий передаваемый бит
		uint16_t	Reserved:1;		//Reserved
	}bitField;
	uint16_t	ManTx;
}tManchesterData;

typedef struct
{
	uint8_t		fSync;					//флаг синхронизации
	uint8_t		HiBitCounter;		//счетчик тактов высокого уровня сигнала
	uint8_t		BitPos;					//счетчик текущего бита принимаемой последовательности
	uint8_t		Byte;						//принятый байт
	uint32_t	ManRxData;			//принятая манчестер-последовательность
}tManchesterRxData;

void 				BSP_Manchester_Init(void);
uint8_t 		BSP_Manchester_SendByte(uint8_t	Byte);
uint8_t			BSP_Manchester_SendData(uint8_t	*pData,uint8_t	Len);

#endif
