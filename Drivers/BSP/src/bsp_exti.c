#include "bsp_exti.h"

void	BSP_EXTI_Init()
{
	 GPIO_InitTypeDef   GPIO_InitStructure;

	  /* Enable GPIOB clock */
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /* Configure PB5 pin as input floating */
	  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStructure.Pull = GPIO_NOPULL;
	  GPIO_InitStructure.Pin = GPIO_PIN_5;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Enable and set EXTI Line9-5 Interrupt to the lowest priority */
	  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 10, 0);
	  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	  //Lidar IRQ pin
	  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
	  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	  GPIO_InitStructure.Pin = GPIO_PIN_4;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	  HAL_NVIC_SetPriority(EXTI4_IRQn, 10, 1);
	  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_5)
	{
		BSP_EXTI5_Callback();
	}
	if(GPIO_Pin == GPIO_PIN_4)
	{
		BSP_EXTI4_Callback();
	}
}

__weak void BSP_EXTI5_Callback()
{

}

__weak void BSP_EXTI4_Callback()
{

}

