#ifndef _BSP_RTC_H
#define _BSP_RTC_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

void			 BSP_RTC_Init(void);
RTC_TimeTypeDef	*BSP_RTC_GetTime(void);

#endif
