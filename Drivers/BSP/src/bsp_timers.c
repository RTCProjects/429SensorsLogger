#include "bsp_timers.h"

TIM_HandleTypeDef		Tim2LidarHandle;	//Angle lidar
TIM_HandleTypeDef		Tim3LidarHandle;	//Center lidar
TIM_HandleTypeDef		Tim5LidarHandle;	//Left lidar
TIM_HandleTypeDef		Tim6LidarHandle;	//Right lidar

TIM_HandleTypeDef		Tim4SonarHandle;	//Sonar

//Center Lidar timer
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
//Left Lidar timer
void	BSP_Timers_TIM5Init()
{
	__HAL_RCC_TIM5_CLK_ENABLE();

	Tim5LidarHandle.Instance = TIM5;

	Tim5LidarHandle.Init.Prescaler     = 84 - 1;
	Tim5LidarHandle.Init.Period        = 50000;
	Tim5LidarHandle.Init.ClockDivision = 0;
	Tim5LidarHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

	if(HAL_TIM_Base_Init(&Tim5LidarHandle) != HAL_OK){
		Error_Handler();
	}
	if(HAL_TIM_Base_Start(&Tim5LidarHandle) != HAL_OK){
		Error_Handler();
	}
}
//Right Lidar timer
void	BSP_Timers_TIM6Init()
{
	__HAL_RCC_TIM6_CLK_ENABLE();

	Tim6LidarHandle.Instance = TIM6;

	Tim6LidarHandle.Init.Prescaler     = 84 - 1;
	Tim6LidarHandle.Init.Period        = 50000;
	Tim6LidarHandle.Init.ClockDivision = 0;
	Tim6LidarHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

	if(HAL_TIM_Base_Init(&Tim6LidarHandle) != HAL_OK){
		Error_Handler();
	}
	if(HAL_TIM_Base_Start(&Tim6LidarHandle) != HAL_OK){
		Error_Handler();
	}
}
//Angle lidar timer
void	BSP_Timers_TIM2Init()
{
	__HAL_RCC_TIM2_CLK_ENABLE();

	Tim2LidarHandle.Instance = TIM2;

	Tim2LidarHandle.Init.Prescaler     = 84 - 1;
	Tim2LidarHandle.Init.Period        = 50000;
	Tim2LidarHandle.Init.ClockDivision = 0;
	Tim2LidarHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

	if(HAL_TIM_Base_Init(&Tim2LidarHandle) != HAL_OK){
		Error_Handler();
	}
	if(HAL_TIM_Base_Start(&Tim2LidarHandle) != HAL_OK){
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

//200Hz timer
void	BSP_Timers_TIM8Init()
{

}

