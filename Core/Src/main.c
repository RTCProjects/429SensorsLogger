/*----------------------------------------------------------------------------------------------------*/
/**
  * @file           main.c
  * @brief          Main program body
**/ 
/*----------------------------------------------------------------------------------------------------*/


/*
 * Сделать запись в лог трехступенчато с использованием NaN
 */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "bsp_sdcard.h"
#include "bsp_usart.h"
#include "devices.h"

#include <stdlib.h>
#include <stdarg.h>

TIM_HandleTypeDef	htim7;
osThreadId 			defaultTaskHandle;

void systemClock_Config(void);
void portClkInit(void);
void mainTask(void const * argument);

xSemaphoreHandle xMainSemaphore;
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  The application entry point.
  * @retval None
  */
int main(void)
{
  HAL_Init();

  systemClock_Config();
  portClkInit();

  osThreadDef(main_task_rtos, mainTask, osPriorityNormal, 0, configMINIMAL_STACK_SIZE + 0x400);
  defaultTaskHandle = osThreadCreate(osThread(main_task_rtos), NULL);
  vSemaphoreCreateBinary(xMainSemaphore);
	
  osKernelStart();	
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 	System Clock Configuration
  * @retval None
  */
void systemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief 		Подключение IO(A,C,D,H) к APB
	* @param 		None
	* @reval		None	
	*/
void portClkInit(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

   GPIO_InitTypeDef  GPIO_InitStruct;
   //настройка пина управления питанием внешним светодиодом
   GPIO_InitStruct.Pin       = GPIO_PIN_4;
   GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
   GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

   //настройка пина управления питанием MPU6050
   GPIO_InitStruct.Pin       = GPIO_PIN_10;
   GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_OD;
   GPIO_InitStruct.Pull      = GPIO_NOPULL;
   GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

   HAL_GPIO_WritePin(GPIOG,GPIO_PIN_10,GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
}
/*----------------------------------------------------------------------------------------------------*/
/** 
  * @brief 			Главный поток программы.
  * @param		 	argument: параметры потока FreeRTOS
  * @reval			None
  */
extern  uint8_t	gpsBuffer[128];
void mainTask(void const * argument)
{		
	char	strBufOutput[300];
	//инициализация обработчика устройств
	/* 4 лидара
	 * 1 - радар
	 * 1 - сонар
	 * 1 - акселерометр/гироскоп
	 * инициализация внешних прерываний
	 */
	Devices_Init();
	//инициализация SDCard SPI
	BSP_SDCard_Init();
	//инициализация UART для WiFi модуля
	BSP_WIFI_Init();
	//инициализация GPS модуля
	BSP_GPS_UART_Init();

	tSDCardWriteData *pSkifCurrentData = (tSDCardWriteData*)IMU_GetSkifCurrentData();

	printf("mainTask - start\n");

	for(;;)
	{
		xSemaphoreTake(xMainSemaphore,portMAX_DELAY);
		sprintf(strBufOutput,"Lc%5d Ll%5d Lr%5d Lf%5d R%5d S%5d\r\nAz:%0.2f Pitch:%0.2f Roll:%0.2f Alt:%f Alt2:%f NMEA:%s VEL:%s\r\n", pSkifCurrentData->sensorsData.ulCenterLidarDistance,
																																		pSkifCurrentData->sensorsData.ulLeftLidarDistance,
																																		pSkifCurrentData->sensorsData.ulRightLidarDistance,
																																		pSkifCurrentData->sensorsData.ulFrontLidarDistance,
																																		pSkifCurrentData->sensorsData.ulRadarDistance,
																																		pSkifCurrentData->sensorsData.ulSonarDistance,
																																		pSkifCurrentData->imuData.fAz,
																																		pSkifCurrentData->imuData.fPitch,
																																		pSkifCurrentData->imuData.fRoll,
																																		pSkifCurrentData->fAltitude,
																																		pSkifCurrentData->fAltitude2,
																																		pSkifCurrentData->strNMEAPosition,
																																		pSkifCurrentData->strNMEAVelocity);
		BSP_WIFI_UARTSend((uint8_t*)strBufOutput,strlen(strBufOutput));
		//BSP_WIFI_UARTSend((uint8_t*)gpsBuffer,strlen(gpsBuffer));

	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Функция запуска потока Main
  * @reval			None
  */
void mainGiveSemaphore()
{
	xSemaphoreGive(xMainSemaphore);
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 			Функция запуска потока Main из прерывания
  * @reval			None
  */
void mainGiveSemaphoreISR()
{
	portBASE_TYPE 	xTaskWoken;
	xSemaphoreGiveFromISR( xMainSemaphore, &xTaskWoken );
	if( xTaskWoken == pdTRUE){
			taskYIELD();
	}
}
/*----------------------------------------------------------------------------------------------------*/

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
volatile uint32_t ulHighFrequencyTimerTicks = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  if(htim->Instance == TIM7){
	  ulHighFrequencyTimerTicks++;
  }
}
/*
 *
 */
uint32_t	GetRunTimeStatsValue()
{
	return ulHighFrequencyTimerTicks;
}
/*
 *
 */
void SetupRunTimeStatsTimer()
{
	RCC_ClkInitTypeDef    clkconfig;
	  uint32_t              uwTimclock = 0;
	  uint32_t              uwPrescalerValue = 0;
	  uint32_t              pFLatency;

	  /*Configure the TIM1 IRQ priority */
	  HAL_NVIC_SetPriority(TIM7_IRQn, 7 ,0);
	  HAL_NVIC_EnableIRQ(TIM7_IRQn);

	  /* Enable TIM1 clock */
	  __HAL_RCC_TIM7_CLK_ENABLE();

	  /* Get clock configuration */
	  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

	  /* Compute TIM1 clock */
	  uwTimclock = 2*HAL_RCC_GetPCLK2Freq();

	  /* Compute the prescaler value to have TIM1 counter clock equal to 10MHz */
	  uwPrescalerValue = (uint32_t) ((uwTimclock / 10000000) - 1);

	  /* Initialize TIM1 */
	  htim7.Instance = TIM7;

	  /* Initialize TIMx peripheral as follow:
	  + Period = [(TIM1CLK/1000) - 1]. to have a (1/1000) s time base.
	  + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
	  + ClockDivision = 0
	  + Counter direction = Up
	  */
	  htim7.Init.Period = (10000000 / 1000) - 1;
	  htim7.Init.Prescaler = uwPrescalerValue;
	  htim7.Init.ClockDivision = 0;
	  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	  if(HAL_TIM_Base_Init(&htim7) == HAL_OK)
	  {
	    /* Start the TIM time Base generation in interrupt mode */
	    HAL_TIM_Base_Start_IT(&htim7);
	  }
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  printf("Error handler in %s file on %d line\n",file,line);

  while(1){
	osDelay(100);
	Devices_LedToggle();
  }
}
/*----------------------------------------------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  
}
/*----------------------------------------------------------------------------------------------------*/
#endif /* USE_FULL_ASSERT */
