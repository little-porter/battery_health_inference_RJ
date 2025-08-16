#ifndef __OTA_H
#define __OTA_H

#include "sys.h"
#include "project.h"

#define OTA_FIFO_SIZE		1024*5
typedef struct _ota_fifo
{
	uint8_t data[OTA_FIFO_SIZE];
	uint16_t pos;
	uint16_t tail;
}ota_fifo_t;


void ota_data_deal_handler(uint8_t *pdata,uint16_t num);

void ota_init(void);

#endif
