#include "bsp_usart.h"

uint8_t receiveBuffer[14];

UART_HandleTypeDef bsp_uart1;

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

}

__weak void BSP_USART_RxData(uint8_t rxByte)
{

}
