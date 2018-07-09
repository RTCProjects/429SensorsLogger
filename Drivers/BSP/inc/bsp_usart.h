#ifndef BSP_INC_BSP_USART_H_
#define BSP_INC_BSP_USART_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

void	BSP_USART_Init(void);
__weak void BSP_USART_RxData(uint8_t rxByte);
void	BSP_WIFI_Init(void);
void	BSP_WIFI_UARTSend(uint8_t *pDyte,uint16_t	Size);

#endif
