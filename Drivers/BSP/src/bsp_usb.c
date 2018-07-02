/*----------------------------------------------------------------------------------------------------*/
/**
  * @file    bsp_usb.c ������ ��� ������ � USB
  * @brief   
**/
/*----------------------------------------------------------------------------------------------------*/
#include "bsp_usb.h"
#include "cmsis_os.h"
#include <string.h>

osThreadId 		usbTaskTxHandle;
osThreadId 		usbTaskRxHandle;

xQueueHandle 	usbTxDataQueue;	
xQueueHandle	usbRxDataQueue;

void	BSP_Usb_TxTask(void const * argument);
void	BSP_Usb_RxTask(void const * argument);

/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	������������� BSP Usb
	* @param	None
	* @reval	None
	*/
void	BSP_Usb_Init()
{
	usbTxDataQueue = xQueueCreate(USB_TX_QUEUE_LEN, sizeof(tUSBTxPackage));
	usbRxDataQueue = xQueueCreate(USB_RX_QUEUE_LEN, sizeof(uint8_t));
	
	vQueueAddToRegistry(usbTxDataQueue,"usbTxDataQueue");

	osThreadDef(usbTxTask, BSP_Usb_TxTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE + 0x100);
 	usbTaskTxHandle = osThreadCreate(osThread(usbTxTask), NULL);
	
	osThreadDef(usbRxTask, BSP_Usb_RxTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE + 0x100);
	usbTaskRxHandle = osThreadCreate(osThread(usbRxTask), NULL);
	
	MX_USB_DEVICE_Init();
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	����� �������� ���� �� ������� � CDC USB
	* @param	argument: ��������� ������ FreeRTOS 
	* @reval	None
	*/
void	BSP_Usb_TxTask(void const * argument)
{
	static tUSBTxPackage	txMsg;
	
	while(1)
	{
		portBASE_TYPE xStatus;
		xStatus=xQueueReceive(usbTxDataQueue, &txMsg, portMAX_DELAY);
			if (xStatus == pdPASS){
				//int	pPackSize = 0;
				//uint8_t *pPackage = protocol_CreatePackageForOutputStream(txMsg.Event,txMsg.Status,txMsg.DataSize,txMsg.pData,&pPackSize);
				//if(pPackage)
				
					
					CDC_Transmit_FS(txMsg.pData,txMsg.Len);
					osDelay(1);
					
					//vPortFree(pPackage);
				}
	}	
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	����� ������ ���� �� ������� � CDC USB
	* @param	argument: ��������� ������ FreeRTOS 
	* @reval	None
	*/
void	BSP_Usb_RxTask(void const * argument)
{
	while(1)
	{
		
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Callback ������� ��������� ����������� �� CDC USB ������ ����
	* @param	pBuf: ��������� �� ����� �������� ������
	* @param	pLen: ��������� �� ���������� �������� ���� � �����
	* @reval 	None
	*/
void BSP_Usb_ReceiveCallback(uint8_t* pBuf, uint32_t *pLen)
{
	
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	���������� ������ ���� � ������� FreeRTOS
	* @param	pData: ��������� �� ����� ������������ ������
	* @param	Len: ������ ������������� ������
	* @reval 	None
	*/
void BSP_Usb_SendPackage(uint8_t	*pData,uint16_t	Len)
{
	static tUSBTxPackage	txMsg;
		
	txMsg.pData = pData;
	txMsg.Len = Len;
	
	xQueueSendToBack(usbTxDataQueue,&txMsg,0);
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	�������� ������ � ������� �������� �� USB
	* @param	String: ��������� �� ����� ������������ ������
	* @reval 	None
	*/
void BSP_Usb_SendString(char	*String)
{
	int Len = strlen(String);
	
	if(Len > 0 && Len < USB_MAX_TX_PACKAGE)
		BSP_Usb_SendPackage((uint8_t*)String,Len);
}
/*----------------------------------------------------------------------------------------------------*/
