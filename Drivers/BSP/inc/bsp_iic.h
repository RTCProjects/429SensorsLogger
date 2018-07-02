#ifndef BSP_INC_BSP_IIC_H_
#define BSP_INC_BSP_IIC_H_

#include "stm32f4xx.h"
#include "cmsis_os.h"

void	BSP_I2C_Init(void);
uint8_t BSP_I2C_Read_Byte(uint8_t addr, uint8_t reg);
void 	BSP_I2C_Read_Bytes(uint8_t addr, uint8_t reg,uint16_t	count,uint8_t	*data);
uint8_t BSP_I2C_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data);

#endif
