#ifndef __OTA_H
#define __OTA_H

#include "sys.h"

void iap_task(void);

#define OTA_FIFO_SIZE		1024*3
typedef struct _ota_fifo
{
	uint8_t data[OTA_FIFO_SIZE];
	uint16_t pos;
	uint16_t tail;
}ota_fifo_t;


void ota_data_write_to_fifo(ota_fifo_t *fifo,uint8_t *pdata,uint16_t num);
extern ota_fifo_t inf_ota_fifo;

void ota_init(void);

#endif
