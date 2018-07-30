/**----------------------------------------------------------------------------------------------------*/
/**
  * @file    devices.c ������ ��������� ������ �� ��������� �� ���� CAN
  * @brief   
**/
/**----------------------------------------------------------------------------------------------------*/
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
  * @brief  ������������� ���������
  * @retval None
  */
void Devices_Init()
{
	receivedDataCounter = 0;

	memset(&SensorsData,0,sizeof(tSensors));

	BSP_Timers_TIM2Init();//������ Front lidar
	BSP_Timers_TIM3Init();//������ Center lidar
	BSP_Timers_TIM4Init();//������ Sonar
	BSP_Timers_TIM5Init();//������ Left lidar
	BSP_Timers_TIM6Init();//������ Rigth lidar
	BSP_USART_Init();	//������������� UART ������
	BSP_EXTI_Init();	//������������� ������� ���������� �� �������/�������
	IMU_Init();	//������������� ������� ��������������

}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  	������� ��������� ��������� �� ��������� ������ ��������
  * @retval 	tSensors:��������� �� ��������� ������ ��������
  */
tSensors	*Devices_GetDataPointer()
{
	taskENTER_CRITICAL();

		memcpy(&SensorsData,ulDistances,sizeof(tSensors));
	taskEXIT_CRITICAL();

	return &SensorsData;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ����� 0 - ������� ���������� �� Front Lidar
  * @retval None
  */
void BSP_EXTI0_Callback()//Front Lidar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_0) TIM2->CR1 |= TIM_CR1_CEN;
	else {
		TIM2->CR1 &= ~TIM_CR1_CEN;
		ulDistances[4] = TIM2->CNT/10;
		TIM2->CNT = 0;
	}
	__enable_irq();
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ����� 1 - ������� ���������� �� Left Lidar
  * @retval None
  */
void BSP_EXTI1_Callback()//Left Lidar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_1) TIM5->CR1 |= TIM_CR1_CEN;
	else {
		TIM5->CR1 &= ~TIM_CR1_CEN;
		ulDistances[2] = TIM5->CNT/10;
		TIM5->CNT = 0;
	}
	__enable_irq();
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ����� 2 - ������� ���������� �� Right Lidar
  * @retval None
  */
void BSP_EXTI2_Callback()//Right Lidar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_2) TIM6->CR1 |= TIM_CR1_CEN;
	else {
		TIM6->CR1 &= ~TIM_CR1_CEN;
		ulDistances[3] = TIM6->CNT/10;
		TIM6->CNT = 0;
	}
	__enable_irq();
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ����� 3 - ������� ���������� �� Sonar
  * @retval None
  */
void BSP_EXTI3_Callback()//Sonar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_3) TIM4->CR1 |= TIM_CR1_CEN;
	else {
		TIM4->CR1 &= ~TIM_CR1_CEN;
		ulDistances[5] = TIM4->CNT/58;
		TIM4->CNT = 0;
	}

	__enable_irq();
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ����� 4 - ������� ���������� �� Center Lidar
  * @retval None
  */
void BSP_EXTI4_Callback()//Center Lidar
{
	__disable_irq();

	if(GPIOI->IDR & GPIO_IDR_IDR_4) TIM3->CR1 |= TIM_CR1_CEN;
	else {
		TIM3->CR1 &= ~TIM_CR1_CEN;
		ulDistances[1] = TIM3->CNT/10;
		TIM3->CNT = 0;
	}

	__enable_irq();
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ����� 5 - ������� ���������� �� MPU6050
  * @retval None
  */
void BSP_EXTI5_Callback()
{
	IMU_ExternalISR();
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  	���������� ���������� UART1 �� ������
  * @param		rxByte:������� ���� �� ������ UART1
  * @retval 	None
  */
void BSP_USART_RxData(uint8_t rxByte)
{
	/*receivedData[receivedDataCounter] = rxByte;
	receivedDataCounter++;
	if(receivedDataCounter>=20)
		receivedDataCounter = 0;

	if(receivedData[0] == 0xAA && receivedData[1] == 0xAA){
		if(receivedData[12] == 0x55 && receivedData[13] == 0x55){
			if(receivedData[2] == 0x0C && receivedData[3] == 0x07){
				uint8_t	chkSum = 0;
				for(int i = 4;i<=10;i++)
					chkSum+=receivedData[i];
				if(chkSum == receivedData[11]){
					uint32_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
					ulDistances[0] = (receivedData[6] * 0x100 + receivedData[7]);
					taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);

				}
			}
		}
	}*/
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ������� ���/���� ������� LED ����
  * @retval None
  */
void	Devices_LedToggle()
{
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_4);
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ������� LED ���� ���
  * @retval None
  */
void	Devices_LedOn()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_SET);
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ������� LED ���� ����
  * @retval None
  */
void	Devices_LedOff()
{
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,GPIO_PIN_RESET);
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  MPU6050 ������� ���
  * @retval None
  */
void	Devices_IMUOn()
{
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_10,GPIO_PIN_RESET);
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  MPU6050 ������� ����
  * @retval None
  */
void	Devices_IMUOff()
{
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_10,GPIO_PIN_SET);
}
/*----------------------------------------------------------------------------------------------------*/
