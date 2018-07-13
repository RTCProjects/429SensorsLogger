#ifndef BSP_INC_BSP_IIC_H_
#define BSP_INC_BSP_IIC_H_

#include "stm32f4xx.h"
#include "cmsis_os.h"
#include "bsp_sdcard.h"

typedef struct
{
  I2C_HandleTypeDef   instance;
  uint16_t            sdaPin;
  GPIO_TypeDef*       sdaPort;
  uint16_t            sclPin;
  GPIO_TypeDef*       sclPort;
}I2C_Module;

void	BSP_I2C_Init(void);
void	BSP_I2C_DeInit(void);
uint8_t BSP_I2C_Read_Byte(uint8_t addr, uint8_t reg);
void 	BSP_I2C_Read_Bytes(uint8_t addr, uint8_t reg,uint16_t	count,uint8_t	*data);
uint8_t BSP_I2C_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data);
void I2C_ClearBusyFlagErratum(I2C_Module* i2c);

void	BSP_I2C2_Init(void);
void	BSP_I2C2_DeInit(void);
uint8_t BSP_I2C2_Read_Byte(uint8_t addr, uint8_t reg);
void 	BSP_I2C2_Read_Bytes(uint8_t addr, uint8_t reg,uint16_t	count,uint8_t	*data);
uint8_t BSP_I2C2_Write_Byte(uint8_t addr, uint8_t reg, uint8_t data);


#endif
