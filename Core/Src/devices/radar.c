#include "radar.h"
#include "main.h"

xQueueHandle		os_rx_query;
osSemaphoreId 		os_rx_semaphore;
osThreadId 			os_radar_task;
radar_rx_buffer_t	rx_buffer;

void			radar_task(void const * argument);
static uint8_t	radar_calc_chksum(uint8_t	*data);

void	radar_init(UART_HandleTypeDef *uart_hdl)
{
	vSemaphoreCreateBinary(os_rx_semaphore);

	osThreadDef(radar_task_hdl, radar_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE );
	os_radar_task = osThreadCreate(osThread(radar_task_hdl), NULL);

	os_rx_query = xQueueCreate(1, sizeof(radar_rx_buffer_t));

	HAL_UART_Receive_DMA(uart_hdl,(uint8_t *)rx_buffer.data,RADAR_RX_BUFFER_SIZE);

}

void	radar_task(void const * argument)
{
	printf("radar_task - start\n");

	static radar_rx_buffer_t current_rx_buffer;
	while(1)
	{
		portBASE_TYPE os_xstatus;

		os_xstatus = xQueueReceive(os_rx_query, &current_rx_buffer, portMAX_DELAY);

		if (os_xstatus == pdPASS){
			radar_package_analysis(&current_rx_buffer);

		}
	}
}
extern uint16_t	ulDistances[6];
void	radar_package_analysis(radar_rx_buffer_t	*radar_rx_buffer)
{
	__IO radar_target_info_msg_t	temp1;

	radar_rx_package_t	radar_package;
	uint8_t	*data = radar_rx_buffer->data;

	for(uint16_t i = 0;i < RADAR_RX_BUFFER_SIZE - sizeof(radar_rx_package_t) ;i++) {
		if(data[i] == 0xAA && data[i+1] == 0xAA) {
			memcpy(&radar_package,&data[i],sizeof(radar_rx_package_t));

			if(radar_package.start_seq == 0xAAAA && radar_package.end_seq == 0x5555) {
				if(radar_calc_chksum(radar_package.data) == radar_package.chk_sum) {

					switch(radar_package.msg_id)
					{
						case RADAR_SENSOR_CONFIGURATUION_MSGID: {

						}break;

						case RADAR_SENSOR_BACK_MSGID: {

						}break;

						case RADAR_SENSOR_STATUS_MSGID: {

						}break;

						case RADAR_SENSOR_TARGET_STATUS_MSGID: {

						}break;

						case RADAR_SENSOR_TARGET_INFO_MSGID: {
							memcpy((uint8_t*)&temp1,(uint8_t*)&radar_package.data,sizeof(uint8_t) * RADAR_RX_PACK_SIZE);
							printf("Distance %d\n",temp1.range_l);

							break;
						}
					}

				}
			}
		}
	}
}

void	radar_rx_callback()
{
	static portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	xQueueSendToBackFromISR(os_rx_query, &rx_buffer, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken != pdFALSE) {
		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
}

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

