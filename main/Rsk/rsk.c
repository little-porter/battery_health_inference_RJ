#include "rsk.h"


#include "tflm.h"
#include "esp_timer.h"

static const char *TAG = "PRJ_RSK";

tflm_module_t rsk_modle;
#define RSK_INPUT_TYPE_NUM              15
#define RSK_INPUT_WINDOW_SIZE           180
#define RSK_OUTPUT_SIZE                 20

float rsk_input_data[RSK_INPUT_WINDOW_SIZE][RSK_INPUT_TYPE_NUM];
float rsk_output_data[RSK_OUTPUT_SIZE];

extern const int rsk_model_data_len;
extern const unsigned char rsk_model_data[];

extern const int rsk1_model_data_len;
extern const unsigned char rsk1_model_data[];


void rsk_inference_task_handler(void *parameters)
{
    static int64_t max_time;
    int64_t start_time, end_time;
    int64_t elapsed_time_ms;
    while(1){
        // 获取开始时间
        start_time = esp_timer_get_time();
        ESP_LOGI(TAG,"RSK modle inference start...");
        tflm_run(&rsk_modle,rsk_input_data,RSK_INPUT_WINDOW_SIZE*RSK_INPUT_TYPE_NUM,rsk_output_data,RSK_OUTPUT_SIZE);
        // 获取结束时间
        end_time = esp_timer_get_time();
        // 计算推理耗时
        elapsed_time_ms = (end_time - start_time)/1000;
        if(elapsed_time_ms > max_time)  max_time = elapsed_time_ms;
        ESP_LOGI(TAG,"RSK modle inference finish...,use time = %d ms",(int)elapsed_time_ms);
        ESP_LOGW(TAG,"RSK modle inference finish...,max use time = %d ms",(int)max_time);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void rsk_modle_init(void)
{

    rsk_modle.interpreter = NULL;
    rsk_modle.model_data = rsk1_model_data;
    tflm_create(&rsk_modle);
    xTaskCreatePinnedToCore(rsk_inference_task_handler,"rsk_task",1024*7,NULL,7,NULL,1);
}




