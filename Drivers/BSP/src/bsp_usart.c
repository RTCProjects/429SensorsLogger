#include "bsp_usart.h"

uint8_t receiveBuffer[14];

UART_HandleTypeDef bsp_uart1;
UART_HandleTypeDef bsp_uart5;


void	BSP_USART_Init()
{
	bsp_uart1.Instance = USART1;
	bsp_uart1.Init.BaudRate = 115200;
	bsp_uart1.Init.WordLength = UART_WORDLENGTH_8B;
	bsp_uart1.Init.StopBits = UART_STOPBITS_1;
	bsp_uart1.Init.Parity = UART_PARITY_NONE;
	bsp_uart1.Init.Mode = UART_MODE_RX;
	bsp_uart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	bsp_uart1.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&bsp_uart1) != HAL_OK){
		Error_Handler();
	}
	if(HAL_UART_Receive_IT(&bsp_uart1, (uint8_t *)receiveBuffer, 14)!=HAL_OK){
		Error_Handler();
	}
}

void	BSP_WIFI_Init()
{
	bsp_uart5.Instance = UART5;
	bsp_uart5.Init.BaudRate = 57600;
	bsp_uart5.Init.WordLength = UART_WORDLENGTH_8B;
	bsp_uart5.Init.StopBits = UART_STOPBITS_1;
	bsp_uart5.Init.Parity = UART_PARITY_NONE;
	bsp_uart5.Init.Mode = UART_MODE_TX_RX;
	bsp_uart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	bsp_uart5.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&bsp_uart5) != HAL_OK){
		Error_Handler();
	}
}
void	BSP_WIFI_UARTSend(uint8_t *pDyte,uint16_t	Size)
{
	HAL_UART_Transmit(&bsp_uart5,pDyte,Size,5000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

}

__weak void BSP_USART_RxData(uint8_t rxByte)
{

}
