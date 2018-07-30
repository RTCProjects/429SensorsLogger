#ifndef BSP_INC_BSP_USART_H_
#define BSP_INC_BSP_USART_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <string.h>

#define UART_WIFI_POLLING_TIMEOUT	5000

void	BSP_USART_Init(void);
void	BSP_WIFI_Init(void);
void	BSP_GPS_UART_Init(void);
void	BSP_WIFI_UARTSend(uint8_t *pDyte,uint16_t	Size);
__weak void BSP_USART_RxData(uint8_t rxByte);

#endif
