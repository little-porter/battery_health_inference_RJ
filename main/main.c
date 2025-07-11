#include <stdio.h>

#include "modbus.h"
#include "rs485.h"

#include "net_config.h"
#include "littlefs_ops.h"
#include "collect_device.h"
#include "led.h"

uint16_t cofig_data[5] = {
    ((DEVICE_ID>>8)&0xFF)|((DEVICE_ID&0xFF)<<8),0x1400,0x0100,0x0200,0x0100
};
uint16_t data_reg[9] = {
    0xE02E,0x10A7,0x2003,0x0300,0x0100,0x1600,0xF401,0xf401,0x0000
};

void app_main(void)
{
    littlefs_ops_init();
    modbus_generate_crcTable();             //生成CRC表
    rs485_driver_init(&rs485_driver);       //初始化RS485串口驱动
    collect_device_init();
    led_init();

    modbus_reg_write(0x0000,cofig_data,5);
    modbus_reg_write(0x1000,data_reg,9);

    uint32_t time_ms = 0;

     // 获取ESP32的芯片ID
    // net_config_init();

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // 禁用中断
    io_conf.mode = GPIO_MODE_OUTPUT; // 设置为输出模式
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_46); // 选择具体的GPIO
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 禁用下拉电阻
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // 禁用上拉电阻
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_46,1);


    while(1)
    {
        time_ms++;
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP_LOGI("device","run time %d",(int)time_ms);
    }
}
