/**----------------------------------------------------------------------------------------------------*/
/**
  * @file    radar.h ћодуль обработки данных от радара
  * @brief
**/
/**----------------------------------------------------------------------------------------------------*/
#include "radar.h"

xQueueHandle		os_rx_query;	//очередь на приЄм сообщений от радара
xQueueHandle		os_data_query;	//очередь на отправку обработанных данных радара

osThreadId 			os_radar_task;
radar_rx_buffer_t	rx_buffer;

void			radar_task(void const * argument);
static uint8_t	radar_calc_chksum(uint8_t	*data);

static radar_queue_data_t target_distance;

__IO static radar_target_sensor_version_msg_t	target_sensor_version;
/**----------------------------------------------------------------------------------------------------*/
/**
  * @brief  »нициализаци€ процесса обрабокти данных от NRA24 Radar
  * @param	uart_hdl - указатель на Handle инициализированного Uart порта
  * @retval None
  */
void	radar_init(UART_HandleTypeDef *uart_hdl)
{
	osThreadDef(radar_task_rtos, radar_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE );
	os_radar_task = osThreadCreate(osThread(radar_task_rtos), NULL);

	os_rx_query		= xQueueCreate(RADAR_RX_BUFFER_QUERY_SIZE, sizeof(radar_rx_buffer_t));
	os_data_query 	= xQueueCreate(RADAR_RX_DATA_QUERY_SIZE, sizeof(radar_queue_data_t));

	HAL_UART_Receive_DMA(uart_hdl,(uint8_t *)rx_buffer.data,RADAR_RX_BUFFER_SIZE);

}
/**----------------------------------------------------------------------------------------------------*/
/**
  * @brief  «адача обработки данных радара
  * @retval None
  */
void	radar_task(void const * argument)
{
	printf("radar_task - start\n");

	static radar_rx_buffer_t current_rx_buffer;
	while(1)
	{
		portBASE_TYPE os_xstatus;
		os_xstatus = xQueueReceive(os_rx_query, &current_rx_buffer, portMAX_DELAY);
		if (os_xstatus == pdPASS) {
			radar_package_analysis(&current_rx_buffer);
		}
	}
}
/**----------------------------------------------------------------------------------------------------*/
/**
  * @brief  ‘ункци€ анализа буфера данных
  * @param	radar_rx_buffer - указатель на приемный DMA буфер
  * @retval None
  */
void	radar_package_analysis(radar_rx_buffer_t	*radar_rx_buffer)
{
	radar_rx_package_t	radar_package;
	uint8_t	*data = radar_rx_buffer->data;

	for(uint16_t i = 0;i < RADAR_RX_BUFFER_SIZE - sizeof(radar_rx_package_t) ;i++) {
		if(data[i] == 0xAA && data[i+1] == 0xAA) {	//find 0xAAAA start seq

			memcpy(&radar_package,&data[i],sizeof(radar_rx_package_t));
			//определ€ем границы пакета
			if(radar_package.start_seq == 0xAAAA && radar_package.end_seq == 0x5555) {
				if(radar_calc_chksum(radar_package.data) == radar_package.chk_sum) {

					switch(radar_package.msg_id)
					{
						case RADAR_SENSOR_BACK_MSGID: {
							memcpy((uint8_t*)&target_sensor_version,(uint8_t*)&radar_package.data,sizeof(radar_target_sensor_version_msg_t));
						}break;

						case RADAR_SENSOR_STATUS_MSGID: {

						}break;

						case RADAR_SENSOR_TARGET_STATUS_MSGID: {

						}break;

						case RADAR_SENSOR_TARGET_INFO_MSGID: {
							radar_target_info_msg_t	target_info;
							radar_queue_data_t		target_data;

								memcpy((uint8_t*)&target_info,(uint8_t*)&radar_package.data,sizeof(uint8_t) * RADAR_RX_PACK_SIZE);

								target_data.distance = target_info.range_h << 8 | target_info.range_l;
								target_data.vspeed = target_info.bytefield.vrelh;

							xQueueSendToBack(os_data_query, &target_data, 0);

							break;
						}
					}
				}
			}
		}
	}
}
/**----------------------------------------------------------------------------------------------------*/
/**
  * @brief  «апрос структуры с измеренным рассто€нием
  * @retval radar_queue_data_t - указатель на структуру с измеренным рассто€нием
  */
radar_queue_data_t		*radar_get_targetdata()
{
	//TODO провер€ть статус запроса из очереди и произвоидть копирование струтуры на стеке
	//portBASE_TYPE os_xstatus;
	/*os_xstatus = */xQueueReceive(os_data_query, &target_distance, 0);

	return &target_distance;
}
/**----------------------------------------------------------------------------------------------------*/
/**
  * @brief  DMA callback
  * @retval None
  */
void	radar_rx_callback()
{
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	xQueueSendToBackFromISR(os_rx_query, &rx_buffer, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken != pdFALSE) {
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
}
/**----------------------------------------------------------------------------------------------------*/
/**
  * @brief  DMA callback
  * @param	data - расчЄт контрльной суммы
  * @retval chksum
  */
static uint8_t	radar_calc_chksum(uint8_t	*data)
{
	uint8_t	chk_sum = 0;
	if(data) {
		for(uint8_t i = 0;i < RADAR_RX_PACK_SIZE;i++) {
			chk_sum += data[i];
		}
	}
	return chk_sum;
}
/**----------------------------------------------------------------------------------------------------*/
