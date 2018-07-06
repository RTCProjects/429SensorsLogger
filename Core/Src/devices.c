/*----------------------------------------------------------------------------------------------------*/
/**
  * @file    devices.c Модуль обработки данных от устройств на шине CAN
  * @brief   
**/
/*----------------------------------------------------------------------------------------------------*/
#include <string.h>


#include "devices.h"
#include "bsp_sdcard.h"
#include "bsp_exti.h"
#include "bsp_timers.h"
#include "bsp_usart.h"

uint8_t receivedDataCounter;
char receivedData[20];

uint8_t flag = 0;
uint8_t flag2 = 0;
uint8_t flag3 = 0;
uint8_t flag4 = 0;

uint32_t tempData;

tSensors	SensorsData;
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  Инициализация модуля устройств
  * @retval None
  */
void Devices_Init()
{
	memset(&SensorsData,0,sizeof(tSensors));

	BSP_Timers_TIM3Init();
	BSP_Timers_TIM4Init();
	BSP_USART_Init();
	BSP_EXTI_Init();
	//Инициализация датчика аксселерометра
	IMU_Init();
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  Функция отправки запроса измеренных данных
  * @retval None
  */
void	Devices_SensorsDataRequest()
{
	static tSDCardWriteData	sensorsSDCardData;

	sensorsSDCardData.type = E_RANGEFINDER;
	memcpy(&sensorsSDCardData.sensorsData,&SensorsData,sizeof(tSensors));

	BSP_SDCard_WriteSensorsData(&sensorsSDCardData);
}
/*----------------------------------------------------------------------------------------------------*/
void BSP_EXTI3_Callback()//Sonar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_3) TIM4->CR1 |= TIM_CR1_CEN;
	else {
		TIM4->CR1 &= ~TIM_CR1_CEN;
		SensorsData.ulSonarDistance = TIM4->CNT/58;
		TIM4->CNT = 0;
	}

	__enable_irq();
}
/*----------------------------------------------------------------------------------------------------*/
void BSP_EXTI4_Callback()//Lidar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_4) TIM3->CR1 |= TIM_CR1_CEN;
	else {
		TIM3->CR1 &= ~TIM_CR1_CEN;
		SensorsData.ulLidarDistance = TIM3->CNT/10;
		TIM3->CNT = 0;
	}

	__enable_irq();
}
/*----------------------------------------------------------------------------------------------------*/
void BSP_USART_RxData(uint8_t rxByte)
{
	tempData = rxByte;


				if (tempData == 0xAA) {
					if (flag == 1) {
						flag2 = 1;
						flag = 0;
					} else flag = 1;
				}

				if (flag2 == 1) {
					if(tempData == 0x0C) {
						flag3 = 1;
						flag2 = 0;
					} else flag3 = 0;
				}

				if (flag3 == 1) {
					receivedData[receivedDataCounter] = USART1->DR;
					receivedDataCounter ++;

					if (receivedDataCounter > 5) {
						SensorsData.ulRadarDistance = (receivedData[4] * 256 + receivedData[5]);

						for(uint8_t i = 0; i < 20; i++) {
							receivedData[i] = 0;
						}
						receivedDataCounter  = 0;
						flag3 = 0;
					}
				}
}

void	Devices_LedToggle()
{
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_4);
}

void	Devices_LedOn()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
}

void	Devices_LedOff()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
}

void	Devices_IMUOn()
{
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_10,GPIO_PIN_RESET);
}

void	Devices_IMUOff()
{
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_10,GPIO_PIN_SET);
}
