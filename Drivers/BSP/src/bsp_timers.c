#include "bsp_timers.h"

TIM_HandleTypeDef		Tim3LidarHandle;
TIM_HandleTypeDef		Tim4SonarHandle;

//Lidar timer
void	BSP_Timers_TIM3Init()
{
	__HAL_RCC_TIM3_CLK_ENABLE();

	Tim3LidarHandle.Instance = TIM3;

	Tim3LidarHandle.Init.Prescaler     = 84 - 1;
	Tim3LidarHandle.Init.Period        = 50000;
	Tim3LidarHandle.Init.ClockDivision = 0;
	Tim3LidarHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

	if(HAL_TIM_Base_Init(&Tim3LidarHandle) != HAL_OK){
		Error_Handler();
	}
	if(HAL_TIM_Base_Start(&Tim3LidarHandle) != HAL_OK){
		Error_Handler();
	}
}

//Sonar timer
void	BSP_Timers_TIM4Init()
{
	__HAL_RCC_TIM4_CLK_ENABLE();

	Tim4SonarHandle.Instance = TIM4;

	Tim4SonarHandle.Init.Prescaler     = 84 - 1;
	Tim4SonarHandle.Init.Period        = 50000;
	Tim4SonarHandle.Init.ClockDivision = 0;
	Tim4SonarHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

	if(HAL_TIM_Base_Init(&Tim4SonarHandle) != HAL_OK){
		Error_Handler();
	}
	if(HAL_TIM_Base_Start(&Tim4SonarHandle) != HAL_OK){
		Error_Handler();
	}
}
