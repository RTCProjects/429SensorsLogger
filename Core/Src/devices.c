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
#include "bmp180.h"

uint8_t receivedDataCounter;
char receivedData[20];

uint8_t flag = 0;
uint8_t flag2 = 0;
uint8_t flag3 = 0;
uint8_t flag4 = 0;

uint32_t tempData;

uint16_t	ulDistances[6];


tSensors	SensorsData;
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  Инициализация модуля устройств
  * @retval None
  */
void Devices_Init()
{
	memset(&SensorsData,0,sizeof(tSensors));

	BSP_Timers_TIM2Init();//Front lidar
	BSP_Timers_TIM3Init();//Center lidar
	BSP_Timers_TIM4Init();//Sonar
	BSP_Timers_TIM5Init();//Left lidar
	BSP_Timers_TIM6Init();//Rigth lidar
	//USART Radar
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
	/*static tSDCardWriteData	sensorsSDCardData;

	sensorsSDCardData.type = E_RANGEFINDER;
	memcpy(&sensorsSDCardData.sensorsData,&SensorsData,sizeof(tSensors));

	//BSP_SDCard_WriteSensorsData(&sensorsSDCardData);*/
	memcpy(&SensorsData,ulDistances,sizeof(tSensors));
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  Функция получения указателя на блок данных расстояния
  * @retval None
  */
tSensors	*Devices_GetDataPointer()
{
	memcpy(&SensorsData,ulDistances,sizeof(tSensors));
	return &SensorsData;
}

void BSP_EXTI0_Callback()//Front Lidar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_0) TIM2->CR1 |= TIM_CR1_CEN;
	else {
		TIM2->CR1 &= ~TIM_CR1_CEN;
		//SensorsData.ulFrontLidarDistance = TIM2->CNT/10;
		ulDistances[4] = TIM2->CNT/10;
		TIM2->CNT = 0;
	}
	__enable_irq();
}
/*----------------------------------------------------------------------------------------------------*/
void BSP_EXTI1_Callback()//Left Lidar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_1) TIM5->CR1 |= TIM_CR1_CEN;
	else {
		TIM5->CR1 &= ~TIM_CR1_CEN;
		//SensorsData.ulLeftLidarDistance = TIM5->CNT/10;
		ulDistances[2] = TIM5->CNT/10;
		TIM5->CNT = 0;
	}
	__enable_irq();
}
/*----------------------------------------------------------------------------------------------------*/
void BSP_EXTI2_Callback()//Right Lidar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_2) TIM6->CR1 |= TIM_CR1_CEN;
	else {
		TIM6->CR1 &= ~TIM_CR1_CEN;
		//SensorsData.ulRightLidarDistance = TIM6->CNT/10;
		ulDistances[3] = TIM6->CNT/10;
		TIM6->CNT = 0;
	}
	__enable_irq();
}
/*----------------------------------------------------------------------------------------------------*/
void BSP_EXTI3_Callback()//Sonar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_3) TIM4->CR1 |= TIM_CR1_CEN;
	else {
		TIM4->CR1 &= ~TIM_CR1_CEN;
		//SensorsData.ulSonarDistance = TIM4->CNT/58;
		ulDistances[5] = TIM4->CNT/58;
		TIM4->CNT = 0;
	}

	__enable_irq();
}
/*----------------------------------------------------------------------------------------------------*/
void BSP_EXTI4_Callback()//Center Lidar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_4) TIM3->CR1 |= TIM_CR1_CEN;
	else {
		TIM3->CR1 &= ~TIM_CR1_CEN;
		//SensorsData.ulCenterLidarDistance = TIM3->CNT/10;
		ulDistances[1] = TIM3->CNT/10;
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
						//SensorsData.ulRadarDistance = (receivedData[4] * 256 + receivedData[5]);
						ulDistances[0] = (receivedData[4] * 256 + receivedData[5]);
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
