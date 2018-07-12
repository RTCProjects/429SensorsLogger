/*----------------------------------------------------------------------------------------------------*/
/**
  * @file           bsp_iic.c
  * @brief          I2C модуль
**/
/*----------------------------------------------------------------------------------------------------*/
#include "bsp_iic.h"

I2C_HandleTypeDef I2C1Handle;
I2C_HandleTypeDef I2C2Handle;

/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 		I2C Инициализация
  * @reval		None
  */
void	BSP_I2C_Init()
{
	I2C1Handle.Instance             = I2C1;

	I2C1Handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	I2C1Handle.Init.ClockSpeed      = 400000;
	I2C1Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2C1Handle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
	I2C1Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2C1Handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	I2C1Handle.Init.OwnAddress1     = 0;
	I2C1Handle.Init.OwnAddress2     = 0;

	if(HAL_I2C_Init(&I2C1Handle) != HAL_OK){
	  Error_Handler();
	}
}

void	BSP_I2C_DeInit()
{
	if(HAL_I2C_DeInit(&I2C1Handle) != HAL_OK){
		 Error_Handler();
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 		I2C Чтение байта
  * @param		:addr - адрес на шине
  * @param		:reg - адрес регистра
  * @reval		значение регистра
  */
uint8_t BSP_I2C_Read_Byte(uint8_t addr, uint8_t reg)
{
	uint8_t data = 0;
	uint8_t d;
	while (HAL_I2C_GetState(&I2C1Handle) != HAL_I2C_STATE_READY);
	d = HAL_I2C_Master_Transmit(&I2C1Handle, addr << 1, &reg, 1, 100);
	if ( d != HAL_OK) {
		return d;
	}

	while (HAL_I2C_GetState(&I2C1Handle) != HAL_I2C_STATE_READY);
	d = HAL_I2C_Master_Receive(&I2C1Handle, addr << 1, &data, 1, 100);
	if ( d != HAL_OK) {
		return d;
	}
	return data;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 		I2C Чтение байтлов
  * @param		:addr - адрес на шине
  * @param		:reg - адрес регистра
  * @param		:data - указатель на массив байт
  * @param		:count - количество читаемых байт
  * @reval		None
  */
void BSP_I2C_Read_Bytes(uint8_t addr, uint8_t reg,uint16_t	count,uint8_t	*data)
{
	//uint8_t data = 0;
	uint8_t d;
	while (HAL_I2C_GetState(&I2C1Handle) != HAL_I2C_STATE_READY);
	d = HAL_I2C_Master_Transmit(&I2C1Handle, addr << 1, &reg, 1, 100);
	if ( d != HAL_OK) {
		return;
	}

	while (HAL_I2C_GetState(&I2C1Handle) != HAL_I2C_STATE_READY);
	d = HAL_I2C_Master_Receive(&I2C1Handle, addr << 1, data, count, 100);
	if ( d != HAL_OK) {
		return;
	}
	return;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 		I2C Запись байта
  * @param		addr: адрес на шине
  * @param		reg: адрес регистра
  * @param		data: данные регистра
  * @reval		значение регистра
  */
uint8_t BSP_I2C_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
	uint8_t buf[] = {reg, data};
	uint8_t d;
	while (HAL_I2C_GetState(&I2C1Handle) != HAL_I2C_STATE_READY);
	d = HAL_I2C_Master_Transmit(&I2C1Handle, addr << 1, buf, 2, 100);
	if ( d != HAL_OK) {
		return d;
	}
	return HAL_OK;
}
/*----------------------------------------------------------------------------------------------------*/

void I2C_ClearBusyFlagErratum(I2C_Module *i2c)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // 1. Clear PE bit.
  i2c->instance.Instance->CR1 &= ~(0x0001);

  //  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C1;
  GPIO_InitStructure.Pull         = GPIO_PULLUP;
  GPIO_InitStructure.Speed        = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 3. Check SCL and SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_RESET);

  //  5. Check SDA Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_RESET);

  //  7. Check SCL Low level in GPIOx_IDR.
  while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  // 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sclPort, i2c->sclPin, GPIO_PIN_SET);

  // 9. Check SCL High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sclPort, i2c->sclPin))
  {
    asm("nop");
  }

  // 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
  HAL_GPIO_WritePin(i2c->sdaPort, i2c->sdaPin, GPIO_PIN_SET);

  // 11. Check SDA High level in GPIOx_IDR.
  while (GPIO_PIN_SET != HAL_GPIO_ReadPin(i2c->sdaPort, i2c->sdaPin))
  {
    asm("nop");
  }

  // 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
  GPIO_InitStructure.Mode         = GPIO_MODE_AF_OD;
  GPIO_InitStructure.Alternate    = GPIO_AF4_I2C1;

  GPIO_InitStructure.Pin          = i2c->sclPin;
  HAL_GPIO_Init(i2c->sclPort, &GPIO_InitStructure);

  GPIO_InitStructure.Pin          = i2c->sdaPin;
  HAL_GPIO_Init(i2c->sdaPort, &GPIO_InitStructure);

  // 13. Set SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 |= 0x8000;

  asm("nop");

  // 14. Clear SWRST bit in I2Cx_CR1 register.
  i2c->instance.Instance->CR1 &= ~0x8000;

  asm("nop");

  // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
  i2c->instance.Instance->CR1 |= 0x0001;

  // Call initialization function.
  HAL_I2C_Init(&(i2c->instance));
}
/*
 *
 *
 *
 *
 *
 *
 */
/*----------------------------------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 		I2C Инициализация
  * @reval		None
  */
void	BSP_I2C2_Init()
{
	I2C2Handle.Instance             = I2C2;

	I2C2Handle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	I2C2Handle.Init.ClockSpeed      = 400000;
	I2C2Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2C2Handle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
	I2C2Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2C2Handle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	I2C2Handle.Init.OwnAddress1     = 0;
	I2C2Handle.Init.OwnAddress2     = 0;

	if(HAL_I2C_Init(&I2C2Handle) != HAL_OK){
	  Error_Handler();
	}
}

void	BSP_I2C2_DeInit()
{
	if(HAL_I2C_DeInit(&I2C2Handle) != HAL_OK){
		 Error_Handler();
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 		I2C Чтение байта
  * @param		:addr - адрес на шине
  * @param		:reg - адрес регистра
  * @reval		значение регистра
  */
uint8_t BSP_I2C2_Read_Byte(uint8_t addr, uint8_t reg)
{
	uint8_t data = 0;
	uint8_t d;
	while (HAL_I2C_GetState(&I2C2Handle) != HAL_I2C_STATE_READY);
	d = HAL_I2C_Master_Transmit(&I2C2Handle, addr << 1, &reg, 1, 100);
	if ( d != HAL_OK) {
		return d;
	}

	while (HAL_I2C_GetState(&I2C2Handle) != HAL_I2C_STATE_READY);
	d = HAL_I2C_Master_Receive(&I2C2Handle, addr << 1, &data, 1, 100);
	if ( d != HAL_OK) {
		return d;
	}
	return data;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 		I2C Чтение байтлов
  * @param		:addr - адрес на шине
  * @param		:reg - адрес регистра
  * @param		:data - указатель на массив байт
  * @param		:count - количество читаемых байт
  * @reval		None
  */
void BSP_I2C2_Read_Bytes(uint8_t addr, uint8_t reg,uint16_t	count,uint8_t	*data)
{
	//uint8_t data = 0;
	uint8_t d;
	while (HAL_I2C_GetState(&I2C2Handle) != HAL_I2C_STATE_READY);
	d = HAL_I2C_Master_Transmit(&I2C2Handle, addr << 1, &reg, 1, 100);
	if ( d != HAL_OK) {
		return;
	}

	while (HAL_I2C_GetState(&I2C2Handle) != HAL_I2C_STATE_READY);
	d = HAL_I2C_Master_Receive(&I2C2Handle, addr << 1, data, count, 100);
	if ( d != HAL_OK) {
		return;
	}
	return;
}
/*----------------------------------------------------------------------------------------------------*/
/**
  * @brief 		I2C Запись байта
  * @param		addr: адрес на шине
  * @param		reg: адрес регистра
  * @param		data: данные регистра
  * @reval		значение регистра
  */
uint8_t BSP_I2C2_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data)
{
	uint8_t buf[] = {reg, data};
	uint8_t d;
	while (HAL_I2C_GetState(&I2C2Handle) != HAL_I2C_STATE_READY);
	d = HAL_I2C_Master_Transmit(&I2C2Handle, addr << 1, buf, 2, 100);
	if ( d != HAL_OK) {
		return d;
	}
	return HAL_OK;
}
/*----------------------------------------------------------------------------------------------------*/

