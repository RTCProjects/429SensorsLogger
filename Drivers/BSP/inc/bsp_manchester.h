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
		uint16_t	Byte:8;				//���� ��� ��������
		uint16_t	fTx:1;				//���� ��������
		uint16_t	fPre:1;				//�������� ���������
		uint16_t	fBitPos:5;		//������� ������������ ���
		uint16_t	Reserved:1;		//Reserved
	}bitField;
	uint16_t	ManTx;
}tManchesterData;

typedef struct
{
	uint8_t		fSync;					//���� �������������
	uint8_t		HiBitCounter;		//������� ������ �������� ������ �������
	uint8_t		BitPos;					//������� �������� ���� ����������� ������������������
	uint8_t		Byte;						//�������� ����
	uint32_t	ManRxData;			//�������� ���������-������������������
}tManchesterRxData;

void 				BSP_Manchester_Init(void);
uint8_t 		BSP_Manchester_SendByte(uint8_t	Byte);
uint8_t			BSP_Manchester_SendData(uint8_t	*pData,uint8_t	Len);

#endif
