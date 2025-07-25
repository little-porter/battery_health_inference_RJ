#ifndef __SYS_H__
#define __SYS_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <stdint.h>
#include <string.h>


void ota_push_msg_to_queue(uint8_t *pdata,uint16_t num,uint32_t timeout);
void iap_flag_start(void);
void iap_flag_upgrading(void);
void iap_flag_finish(void);
void ota_crc_set(uint16_t crc);

#endif

