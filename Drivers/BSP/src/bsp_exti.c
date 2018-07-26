/*----------------------------------------------------------------------------------------------------*/
/**
  * @file           bsp_exti.c
  * @brief          ћодуль инициализации внешних прерываний
**/
/*----------------------------------------------------------------------------------------------------*/
#include "bsp_exti.h"
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  Ќастройка портов GPIO и приоритетов прерываний
  * @retval None
  */
void	BSP_EXTI_Init()
{
	 GPIO_InitTypeDef   GPIO_InitStructure;

	  /* Enable GPIOB clock */
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOI_CLK_ENABLE();

	  /* Configure PB5 pin as input floating */
	  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStructure.Pull = GPIO_NOPULL;
	  GPIO_InitStructure.Pin = GPIO_PIN_5;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Enable and set EXTI Line9-5 Interrupt to the lowest priority */
	  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 9, 0);
	  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	  //EXTI lidars/sonar 0 - 4
	  GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
	  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	  GPIO_InitStructure.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
	  HAL_GPIO_Init(GPIOI, &GPIO_InitStructure);

	  //Angle Lidar IRQ pin
	  HAL_NVIC_SetPriority(EXTI0_IRQn, 7, 0);
	  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	  //Left Lidar IRQ pin
	  HAL_NVIC_SetPriority(EXTI1_IRQn, 7, 1);
	  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	  //Right Lidar IRQ pin
	  HAL_NVIC_SetPriority(EXTI2_IRQn, 7, 2);
	  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	  //Center Lidar IRQ pin
	  HAL_NVIC_SetPriority(EXTI3_IRQn, 7, 3);
	  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	  //Sonar IRQ pin
	  HAL_NVIC_SetPriority(EXTI4_IRQn, 7, 4);
	  HAL_NVIC_EnableIRQ(EXTI4_IRQn);


}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  HAL Callback внешних прерываний
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_5)//прерывани€ пина INT от MPU6050
		BSP_EXTI5_Callback();
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  weak Callback MPU6050
  * @retval None
  */
__weak void BSP_EXTI5_Callback()
{

}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  weak Callback Center Lidar
  * @retval None
  */
__weak void BSP_EXTI4_Callback()
{
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  weak Callback Sonar
  * @retval None
  */
__weak void BSP_EXTI3_Callback()
{

}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  weak Callback Right Lidar
  * @retval None
  */
__weak void BSP_EXTI2_Callback()
{

}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  weak Callback Left Lidar
  * @retval None
  */
__weak void BSP_EXTI1_Callback()
{

}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief  weak Callback Front Lidar
  * @retval None
  */
__weak void BSP_EXTI0_Callback()
{

}
/*----------------------------------------------------------------------------------------------------*/
