#ifndef __COLLECT_DEVICE_H__
#define __COLLECT_DEVICE_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"


#include "driver/uart.h"


/*获取设备在线状态*/
bool collect_device_online_status_get(void);

void collect_device_init(void);

#endif

