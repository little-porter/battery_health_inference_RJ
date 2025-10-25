#include <stdio.h>
#include "sys.h"

#include "project.h"
#include "littlefs_ops.h"

#include "rs485.h"
#include "modbus.h"
#include "ota.h"
#include "iap.h"
#include "led.h"
#include "collect_device.h"

#include "devConfig.h"
#include "alarm.h"

#include "tflm.h"
#include "soc.h"
#include "soh.h"
#include "rsk.h"

#include "battery_data.h"
#include "configJson.h"

#include "socModelVerify.h"
#include "rskModelVerify.h"

static const char *TAG = "PRJ_MAIN";

void collect_device_power_on(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // 禁用�?�?
    io_conf.mode = GPIO_MODE_OUTPUT; // 设置为输出模�?
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_46); // 选择具体的GPIO
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 禁用下拉电阻
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // 禁用上拉电阻
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_46,1);
}


void app_main(void)
{
    collect_device_power_on();

    littlefs_ops_init();
    devConfig_init();

    modbus_generate_crcTable();

    // sysEvent_init();

    
    ota_init();
    iap_init();
    rs485_driver_init(&rs485_driver);
    led_init();
    
    
    collect_device_init();

    //ģ�����
    tflm_init();
    soc_modle_init();
    // soh_modle_init();
    // rsk_modle_init();
    battery_data_init();

    // socModelVerify_init();
    // rskModelVerify_init();
    // alarm_init();
    configJson_init();

    while (1)
    {
        /* code */
        vTaskDelay(pdMS_TO_TICKS(5000));
        littlefs_ops_read_file_info();
        ESP_LOGI(TAG,"sys running...");
        // soc_init_value_get();
    }
    
}


