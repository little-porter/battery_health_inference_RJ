#include "soc.h"
#include "tflm.h"

static const char *TAG = "PRJ_SOC";

extern const int soc_model_data_len;
extern const unsigned char soc_model_data[];

tflm_module_t soc_modle;
#define SOC_INPUT_SIZE 100
#define SOC_OUTPUT_SIZE 10
float soc_input_data[SOC_INPUT_SIZE];
float soc_output_data[SOC_OUTPUT_SIZE];



void soc_inference_task_handler(void *parameters)
{
    int64_t start_time, end_time;
    int64_t elapsed_time_ms;
    while(1){
        // 记录开始时间 (微秒)
        start_time = esp_timer_get_time();
        ESP_LOGI(TAG,"SOC modle inference start...");
        tflm_run(&soc_modle,soc_input_data,SOC_INPUT_SIZE,soc_output_data,SOC_OUTPUT_SIZE);
        // 记录结束时间 (微秒)
        end_time = esp_timer_get_time();
        // 计算耗时 (微秒)
        elapsed_time_ms = (end_time - start_time)/1000;
        ESP_LOGI(TAG,"SOC modle inference finish...,use time = %d ms",elapsed_time_ms);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void soc_modle_init(void)
{
    soc_modle.interpreter = NULL;
    soc_modle.model_data = soc_model_data;
    tflm_create(&soc_modle);
    xTaskCreatePinnedToCore(soc_inference_task_handler,"soc_task",1024*4,NULL,7,NULL,1);
}
