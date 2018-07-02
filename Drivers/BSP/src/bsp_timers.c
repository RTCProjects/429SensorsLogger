#include "bsp_timers.h"

TIM_HandleTypeDef		Tim3LidarHandle;

void	BSP_Timers_TIM3Init()
{
	__HAL_RCC_TIM3_CLK_ENABLE();

	Tim3LidarHandle.Instance = TIM3;

	Tim3LidarHandle.Init.Prescaler     = 72 - 1;
	Tim3LidarHandle.Init.Period        = 50000;
	Tim3LidarHandle.Init.ClockDivision = 0;
	Tim3LidarHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

	if(HAL_TIM_Base_Init(&Tim3LidarHandle) != HAL_OK){
			Error_Handler();
	}
}
