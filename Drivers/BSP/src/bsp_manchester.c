/*----------------------------------------------------------------------------------------------------*/
/**
  * @file    bsp_manchester.c Модуль для работы с Manchester code
  * @brief   
**/
/*----------------------------------------------------------------------------------------------------*/
#include "bsp_manchester.h"
#include "bsp_usb.h"

#define _pinset_() 	 	GPIOB->BSRR |= GPIO_BSRR_BS9
#define _pinreset_() 	GPIOB->BSRR |= GPIO_BSRR_BR9
#define _pintoggle_() GPIOB->ODR ^= GPIO_PIN_9;

#define _getrxpin()		(GPIOA->IDR & 0x00000001)
#define _gettxpin()		(GPIOB->IDR & 0x00000100)

static tManchesterData		txManData;
static tManchesterRxData	rxManData;

static uint8_t						*pManDataBuf;
static uint8_t						uManTxByteIndex;
static uint8_t						uManTxBufLen;

uint8_t	manRxArr[256];

uint32_t	BSP_ByteToManEncode(uint8_t	Byte);
uint8_t		BSP_ManToByteDecode(uint32_t Word);
void 			BSP_Manchester_TxEnd_Callback(void);
void 			BSP_Manchester_RxEndCallback(void);
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Инициализация BSP Manchester
	* @note		Настройка портов ввода вывода для генерации сигналов манчестерского кодирования
	* настройка таймеров TIM5 и TIM6 для приема и передачи кодированного сигнала. Период таймеров - 5 uS. 
	* @param	None
	* @reval	None
	*/
void BSP_Manchester_Init()
{
	RCC->APB1ENR |= (RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM5EN);
	RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOAEN);
	
	//TIM6 - Generate output signal
	TIM6->PSC = 21 - 1;
	TIM6->ARR = 20 - 1;
	TIM6->DIER = TIM_DIER_UIE;
	
	NVIC_SetPriority(TIM6_DAC_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 9, 0));
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	TIM6->CR1 = (TIM_CR1_CEN | TIM_CR1_ARPE);
	
	//TIM5 - Capture input signal
	TIM5->PSC = 21 - 1;
	TIM5->ARR = 20 - 1;
//	TIM5->CCMR1 = TIM_CCMR1_CC1S_0;
//	TIM5->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P | TIM_CCER_CC1NP;
	TIM5->DIER = TIM_DIER_UIE;
	
	NVIC_SetPriority(TIM5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 10, 0));
	NVIC_EnableIRQ(TIM5_IRQn);
	
	TIM5->CR1 = (TIM_CR1_CEN | TIM_CR1_ARPE);

	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0_0|GPIO_OSPEEDER_OSPEEDR0_1);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_1;
	
	GPIOB->MODER |= GPIO_MODER_MODE9_0;
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR9_0|GPIO_OSPEEDER_OSPEEDR9_1);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD9_1;
	
	GPIOB->MODER |= GPIO_MODER_MODE8_0;
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR8_0|GPIO_OSPEEDER_OSPEEDR8_1);
	GPIOB->PUPDR |= GPIO_PUPDR_PUPD8_1;
	
	
	txManData.bitField.Byte = 0;
	txManData.bitField.fTx = 0;
	txManData.bitField.fBitPos = 0;
	
	rxManData.BitPos = RX_SIZE;
	rxManData.HiBitCounter = 0;
	rxManData.fSync = 0;
	rxManData.ManRxData = 0; 
	
	uManTxBufLen = 0;
	uManTxByteIndex = 0;
	pManDataBuf = 0;
	
	memset(manRxArr,0,sizeof(uint8_t) * TX_PACK_SIZE);
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Передача массива данных в линию
	* @param	pData: указатель на начало массива для передачи в линию
	* @param	Len: длина передаваемого массива
	* @reval	0 - передача не запущена, 1 - передача запущена
	*/
uint8_t	BSP_Manchester_SendData(uint8_t	*pData,uint8_t	Len)
{
	uManTxBufLen = Len;
	uManTxByteIndex = 0;
	pManDataBuf = pData;
		
	if(!BSP_Manchester_SendByte(pManDataBuf[0]))	return 0;
	else return 1;
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Передача байта данных в линию
	* @param	Byte: байт для передачи
	* @reval	0 - передача не запущена, 1 - передача запущена
	*/
uint8_t BSP_Manchester_SendByte(uint8_t	Byte)
{
	if(txManData.bitField.fTx)return 0;
	
	txManData.bitField.Byte = Byte;
	txManData.bitField.fBitPos = TX_SIZE;
	txManData.bitField.fTx = 1;
	
	return 1;
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Callback функция окончания передачи байта
	* @reval	None
	*/
void BSP_Manchester_TxEnd_Callback()
{
	if(uManTxByteIndex >= uManTxBufLen - 1)return;
	else
	{
		uManTxByteIndex++;		
		for(int i = 0;i<0x5000;i++)__NOP();
			BSP_Manchester_SendByte(pManDataBuf[uManTxByteIndex]);
	}
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Callback функция приема байта из линии
	* @reval	None
	*/
uint8_t	testRxInd = 0;


void BSP_Manchester_RxEndCallback()
{
	__IO uint32_t	curManData = rxManData.ManRxData;
	
	rxManData.Byte = BSP_ManToByteDecode(curManData);
	
	manRxArr[testRxInd++] = rxManData.Byte;
	/*
	
	*/
	/*
	manRxArr[testRxInd] = rxManData.Byte;
	testRxInd++;*/
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Функия кодирования последовательности битов одного байта в последовательность битов 
	* манчестерского кода согласно IEEE 802
	* @param	Byte: кодируемый байт
	* @reval	16 битовая последовательность манчестерского кода
	*/
uint32_t	BSP_ByteToManEncode(uint8_t	Byte)
{
	uint32_t	uResult = 0;
	uint8_t		bIndex = 0;
	
	for(int i = 0;i<8;i++){
		if( (Byte >> i) & 0x1)
			(uResult) |= (1<<bIndex);
		else
			(uResult) |= (2<<bIndex);
			
		bIndex+=2;
	}
	return uResult;
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Функия декодирования последовательности 16 битов манчестерского кода в байт
	* @param	Word: декодируемая последовательность
	* @reval	24 битовая последовательность манчестерского кода
	*/
uint8_t	BSP_ManToByteDecode(uint32_t Word)
{
	uint8_t	uResult = 0;
	uint8_t	bIndex = 0;
	
	for(int i = 0;i<16;i+=2){
		if( (Word >> i) & 0x1)
			uResult |= (1<<bIndex);
		bIndex++;
	}
	return uResult;
}
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Обработчикр прерывания по переполнению TIM6 - таймер передачи
	* @reval	None
	*/
void TIM6_DAC_IRQHandler()
{
	TIM6->SR &= ~TIM_SR_UIF;
	//GPIOB->ODR ^= GPIO_PIN_8;
	
	if(!txManData.bitField.fTx)return;
	/*
	проверка переполнения 5 битового поля счетчика битов передачи
		-гасим линию передачи по окончании передачи последнего бита
		-вызываем callback окончания передачи последовательности
	*/
	if(txManData.bitField.fBitPos==0x1F){
		txManData.bitField.fTx = 0;
		_pinreset_();
		BSP_Manchester_TxEnd_Callback();
	}
	
	uint32_t	sendData = BSP_ByteToManEncode(txManData.bitField.Byte);	//кодирование передаваемого байта в манчестерский код

	//наложение маски SFD битов (111000xxxxxxxxxxxxxxxx)
	sendData |= 0x380000;
	
	if((sendData >> txManData.bitField.fBitPos) & 0x01)_pinset_();
	else _pinreset_();
	
		txManData.bitField.fBitPos--;
}
/*
	TIM5 - 5 Us time upd 
*/
/*__IO uint8_t	bitCounter = 21;
__IO uint32_t	dataIn = 0;

uint8_t	counterHi = 0,syncByte = 0;
*/
/*----------------------------------------------------------------------------------------------------*/
/**
	* @brief	Обработчикр прерывания по переполнению TIM5 - таймер приема
	* @reval	None
	*/
void TIM5_IRQHandler()
{	
	TIM5->SR &= ~TIM_SR_UIF;
	//выполняем синхронизацию по первому полученному положительному фронту
	if((GPIOA->IDR & 0x1) && (rxManData.fSync == 0)){
		rxManData.fSync = 1;
		TIM5->CNT = 0;
	}
	//счетчик строба положительного сигнала на линии. Начало SDF последовательности - 3 бита
	if(rxManData.HiBitCounter == 3){
		rxManData.HiBitCounter = 0;
		rxManData.BitPos = RX_SIZE;
	}
	//читаем пин порта, формируя битовую последовательность манчестерского кодирования
	if((GPIOA->IDR & 0x01)){
		rxManData.HiBitCounter++;
		rxManData.ManRxData |= (1<<rxManData.BitPos);	
	}
	else
		rxManData.HiBitCounter = 0;
	
	//фиксируем конец передачи
	if(rxManData.BitPos > 0){
		rxManData.BitPos--;
		if(rxManData.BitPos == 0){
			//по окончании убираем 3 бита конца SDF последовательности, вызываем callback и очищаем приемник данных
			rxManData.ManRxData = rxManData.ManRxData >> 3;
			BSP_Manchester_RxEndCallback();
			rxManData.ManRxData = 0;
		}
	}
	
	/*
	if((GPIOA->IDR & 0x1) && (syncByte == 0)){
		syncByte = 1;
		TIM5->CNT = 0;
	}

	if(counterHi == 3){
		counterHi = 0;
		bitCounter = TX_SIZE;
		dataIn = dataIn >> 3;
		
		BSP_Manchester_RxEndCallback(&dataIn);
		
		dataIn = 0;
	}
	
	if((GPIOA->IDR & 0x01)){
		counterHi++;
		dataIn |= (1<<bitCounter);	
	}
	else
		counterHi = 0;
	
	bitCounter--;*/
}
/*----------------------------------------------------------------------------------------------------*/
