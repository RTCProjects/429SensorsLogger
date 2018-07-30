/*----------------------------------------------------------------------------------------------------*/
/**
  * @file    bsp_usart.c Модуль инициализации UART
  * @brief
**/
/*----------------------------------------------------------------------------------------------------*/
#include "bsp_usart.h"
#include "gps.h"
#include "radar.h"

UART_HandleTypeDef bsp_uart1;
UART_HandleTypeDef bsp_uart5;
UART_HandleTypeDef bsp_uart7;
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Инициализация UART1 для приема данных от радара
	* @reval	None
	*/
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

	if (HAL_UART_Init(&bsp_uart1) != HAL_OK) {
		_Error_Handler("bsp_usart.c",37);
	}

	radar_init(&bsp_uart1);
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief	Инициализация UART5 для приема/передачи данных по WiFi
  * @reval	None
  */
void	BSP_WIFI_Init()
{
	bsp_uart5.Instance = UART5;
	bsp_uart5.Init.BaudRate = 57600;///115200;
	bsp_uart5.Init.WordLength = UART_WORDLENGTH_8B;
	bsp_uart5.Init.StopBits = UART_STOPBITS_1;
	bsp_uart5.Init.Parity = UART_PARITY_NONE;
	bsp_uart5.Init.Mode = UART_MODE_TX_RX;
	bsp_uart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	bsp_uart5.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&bsp_uart5) != HAL_OK){
		_Error_Handler("bsp_usart.c",62);
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief	Инициализация UART7 для приема данных от GPS
  * @reval	None
  */
void	BSP_GPS_UART_Init()
{
	bsp_uart7.Instance = UART7;
	bsp_uart7.Init.BaudRate = 9600;
	bsp_uart7.Init.WordLength = UART_WORDLENGTH_8B;
	bsp_uart7.Init.StopBits = UART_STOPBITS_1;
	bsp_uart7.Init.Parity = UART_PARITY_NONE;
	bsp_uart7.Init.Mode = UART_MODE_RX;
	bsp_uart7.Init.HwFlowCtl = UART_HWCONTROL_RTS;
	bsp_uart7.Init.OverSampling = UART_OVERSAMPLING_16;


	if (HAL_UART_Init(&bsp_uart7) != HAL_OK){
		_Error_Handler("bsp_usart.c",83);
	}
	if(HAL_UART_Receive_IT(&bsp_uart7, (uint8_t *)NMEA_GetGPSBuffer(), 128)!=HAL_OK){
		_Error_Handler("bsp_usart.c",86);
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief	Отправка пакета данных по WiFi
  * @reval	None
  */
void	BSP_WIFI_UARTSend(uint8_t *pDyte,uint16_t	Size)
{
	/**
	 * TODO отправка данных в WiFi - UART bridge осуществялется поллингом в теле главного таска mainTask.
	 * Если WiFi выводить в продакшн, то реализацию передачи данных по WiFi необходимо сделать в виде
	 * отдельного таска c передачей буфера по DMA, по аналогии с radar.c.
	 */
	HAL_UART_Transmit(&bsp_uart5, pDyte, Size, UART_WIFI_POLLING_TIMEOUT);
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief	DMA callback от радара
  * @reval	None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle->Instance == USART1) {
		radar_rx_callback();
	}
}
/*----------------------------------------------------------------------------------------------------*/

__weak void BSP_USART_RxData(uint8_t rxByte)
{

}
/*----------------------------------------------------------------------------------------------------*/
