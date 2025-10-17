#include "interSOH.h"

#include "tflm.h"
#include "esp_timer.h"

#include "uniformzation.h"

static const char *TAG = "PRJ_SOH";

tflm_module_t soh_modle;

typedef enum _soh_input_type
{
    SOH_INPUT_TYPE_SOH      = 0,
    SOH_INPUT_TYPE_VOLTAGE  = 1,
    SOH_INPUT_TYPE_CURRENT  = 2,
    SOH_INPUT_TYPE_CRATE    = 3,
    SOH_INPUT_TYPE_ICAREA   = 4,
    SOH_INPUT_TYPE_QVF      = 5,
}soh_input_type_t;


#define SOH_INPUT_TYPE_NUM              6
#define SOH_INPUT_WINDOW_SIZE           240
#define SOH_OUTPUT_SIZE                 10

float soh_input_data[SOH_INPUT_WINDOW_SIZE][SOH_INPUT_TYPE_NUM];
float soh_output_data[SOH_INPUT_WINDOW_SIZE];
uint16_t soh_input_index = 0;
bool soh_input_full  = false;

extern const int soh_model_data_len;
extern const unsigned char soh_model_data[];

extern const int soh1_model_data_len;
extern const unsigned char soh1_model_data[];

extern const int soh4_model_data_len;
extern const unsigned char soh4_model_data[];

void soh_inference_task_handler(void *parameters)
{
    if(!soh_input_full){
        ESP_LOGW(TAG,"SOH input fifo is not full...  index = %d",(int)soh_input_index);
        return;
    }
    if(soh_modle.interpreter == NULL){
        ESP_LOGE(TAG,"SOH modle interpreter is not create...");
        return;
    }
    static int64_t max_time;
    int64_t start_time, end_time;
    int64_t elapsed_time_ms;
   
    float *inputWicket = heap_caps_malloc(SOH_INPUT_WINDOW_SIZE*SOH_INPUT_TYPE_NUM*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    uniformization_interface(soh_input_data,inputWicket,SOH_INPUT_WINDOW_SIZE,SOH_INPUT_TYPE_NUM,soh_input_index);

    //获取开始时间 (微秒)
    start_time = esp_timer_get_time();
    ESP_LOGI(TAG,"SOH modle inference start...");
    tflm_run(&soh_modle,inputWicket,SOH_INPUT_WINDOW_SIZE*SOH_INPUT_TYPE_NUM,soh_output_data,SOH_OUTPUT_SIZE);
    // 获取结束时间 (微秒)
    end_time = esp_timer_get_time();
    // 计算时间差 (毫秒)
    elapsed_time_ms = (end_time - start_time)/1000;
    if(elapsed_time_ms > max_time)  max_time = elapsed_time_ms;
    ESP_LOGI(TAG,"SOH modle inference finish...,use time = %d ms",(int)elapsed_time_ms);
    ESP_LOGW(TAG,"SOH modle inference finish...,max use time = %d ms",(int)max_time);

    for(int i=0;i<soh_modle.result_num;i++){
        printf("SOC modle inference result[%d]: %f",i,soh_output_data[i]);
    }
    printf("\r\n");
    heap_caps_free(inputWicket);
    // vTaskDelay(pdMS_TO_TICKS(1000));
}

void soh_input_data_fill(float *data,uint16_t num,uint32_t index){
    soh_input_index = index%SOH_INPUT_WINDOW_SIZE;
    memcpy(soh_input_data[soh_input_index],data,num*sizeof(float));
    // soh_input_index++;
    // soh_input_index %= SOH_INPUT_WINDOW_SIZE;

    if(index >= SOH_INPUT_WINDOW_SIZE){
        soh_input_full = true;
    }else{;}
}
void soh_modle_init(void)
{

    soh_modle.interpreter = NULL;
    soh_modle.model_data = soh_model_data;
    
    tflm_create(&soh_modle);
    // xTaskCreatePinnedToCore(soh_inference_task_handler,"soh_task",1024*4,NULL,7,NULL,1);
}





