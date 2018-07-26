#ifndef RADAR_H_
#define RADAR_H_

#include "stm32f4xx.h"
#include "cmsis_os.h"

#include <string.h>

#define RADAR_RX_BUFFER_SIZE	64
#define RADAR_RX_PACK_SIZE		7

#define RADAR_SENSOR_CONFIGURATUION_MSGID	0x200
#define RADAR_SENSOR_BACK_MSGID				0x400
#define RADAR_SENSOR_STATUS_MSGID			0x60A
#define RADAR_SENSOR_TARGET_STATUS_MSGID	0x70B
#define RADAR_SENSOR_TARGET_INFO_MSGID		0x70C


typedef struct
{
	uint16_t	start_seq;
	uint16_t	msg_id;
	uint8_t		data[RADAR_RX_PACK_SIZE];
	uint8_t		chk_sum;
	uint16_t	end_seq;
}radar_rx_package_t;

typedef struct
{
	uint8_t	vrelh:3;
	uint8_t	rsvd:3;
	uint8_t	roll_count:2;
}radar_target_into_bytefield_t;

typedef struct
{
	uint8_t		index;
	uint8_t		rcs;
	uint8_t		range_h;
	uint8_t		range_l;
	uint8_t		rsvd;
	radar_target_into_bytefield_t	bytefield;
	uint8_t		vrell;
}radar_target_info_msg_t;

typedef struct
{
	uint8_t		data[RADAR_RX_BUFFER_SIZE];
}radar_rx_buffer_t;


void	radar_init(UART_HandleTypeDef *uart_hdl);
void	radar_rx_callback(void);
void	radar_package_analysis(radar_rx_buffer_t	*data);

#endif
