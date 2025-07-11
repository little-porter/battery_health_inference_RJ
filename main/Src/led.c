#include "led.h"

#define LED_PIN     GPIO_NUM_1

void led_task_handler(void *pvParameters);
void led_init(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // 禁用中断
    io_conf.mode = GPIO_MODE_OUTPUT; // 设置为输出模式
    io_conf.pin_bit_mask = (1ULL << LED_PIN); // 选择具体的GPIO
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 禁用下拉电阻
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // 禁用上拉电阻
    gpio_config(&io_conf);

    xTaskCreatePinnedToCore(led_task_handler,"led_task",1024*2,NULL,5,NULL,0);
}

void led_task_handler(void *pvParameters)
{
    while(1)
    {
        gpio_set_level(LED_PIN,1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN,0);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}




