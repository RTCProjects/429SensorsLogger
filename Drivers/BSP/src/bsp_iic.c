/*----------------------------------------------------------------------------------------------------*/
/**
  * @file           bsp_iic.c
  * @brief          I2C модуль
**/
/*----------------------------------------------------------------------------------------------------*/
#include "bsp_iic.h"

I2C_HandleTypeDef I2C1Handle;
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
