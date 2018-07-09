#ifndef BSP_INC_BSP_EXTI_H_
#define BSP_INC_BSP_EXTI_H_

#include <stm32f4xx.h>
#include <cmsis_os.h>

void	BSP_EXTI_Init(void);
__weak 	void BSP_EXTI5_Callback(void);
__weak	void BSP_EXTI4_Callback(void);
__weak	void BSP_EXTI3_Callback(void);
__weak	void BSP_EXTI2_Callback(void);
__weak	void BSP_EXTI1_Callback(void);
__weak	void BSP_EXTI0_Callback(void);



#endif
