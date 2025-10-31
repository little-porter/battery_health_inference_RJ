#include "soc.h"
#include "tflm.h"
#include "esp_timer.h"
#include "modbus.h"

#include "uniformzation.h"

static const char *TAG = "PRJ_SOC";

#define PRJ_SOC_LOG_ENABLE      1                     //soh log enable
#if PRJ_SOC_LOG_ENABLE
#define PRJ_SOC_PRINTF(x,...)           printf(x,##__VA_ARGS__)
#define PRJ_SOC_LOGI(format, ...)       ESP_LOGI(TAG,format, ##__VA_ARGS__)
#define PRJ_SOC_LOGW(format, ...)       ESP_LOGW(TAG,format, ##__VA_ARGS__)
#else
#define PRJ_SOC_PRINTF(x,...)          
#define PRJ_SOC_LOGI(format, ...)       
#define PRJ_SOC_LOGW(format, ...)       
#endif

#define PRJ_SOC_LOGE(format, ...)       ESP_LOGE(TAG,format, ##__VA_ARGS__)



extern const int soc_model_data_len;
extern const unsigned char soc_model_data[];

extern const int soc1_model_data_len;
extern const unsigned char soc1_model_data[];



tflm_module_t soc_modle;


float m_soc;
uint16_t capacity;
float m_capacity;

#define SOC_INPUT_TYPE_NUM              5
typedef enum _SOC_INPUT_TYPE
{
    SOC_INPUT_CFDZT = 0,    //充放电状态 
    SOC_INPUT_VOLTAGE,      //电压
    SOC_INPUT_CFDSJ,        //充放电时间
    SOC_INPUT_SOC,          //SOC
    SOC_INPUT_DQ,           //增量容量
}SOC_INPUT_TYPE;


#define SOC_INPUT_WINDOW_TIME           120             //输入时间窗口
#define SOC_INPUT_WINDOW_SPAN           3               //输入时间间隔  
#define SOC_INPUT_WINDOW_SIZE           40              //输入窗口数量
float *soc_input_window = NULL;



#define SOC_OUTPUT_SIZE                 2

float soc_input_data[SOC_INPUT_WINDOW_SIZE][SOC_INPUT_TYPE_NUM];
float soc_output_data[SOC_OUTPUT_SIZE];
uint16_t soc_input_index = 0;
bool soc_input_full  = false;

bool socPredFlag = false;


#define SOC_PREDICTION_1H_REG        0x2001
#define SOC_PREDICTION_2H_REG        0x2002

void soc_inference_task_handler(void *parameters)
{
    if(!soc_input_full){
        PRJ_SOC_LOGW("SOC input fifo is not full...  index = %d",(int)soc_input_index);
        return;
    }
    if(soc_modle.interpreter == NULL){
        PRJ_SOC_LOGE("SOC modle interpreter is not create...");
        return;
    }
    static int64_t max_time;
    int64_t start_time, end_time;
    int64_t elapsed_time_ms;
    uint64_t row = SOC_INPUT_WINDOW_SIZE;
    uint64_t column = SOC_INPUT_TYPE_NUM;

    float *inputWicket = heap_caps_malloc(SOC_INPUT_WINDOW_SIZE*SOC_INPUT_TYPE_NUM*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    // for(int i=0;i<SOC_INPUT_WINDOW_SIZE;i++){
    //     PRJ_SOC_PRINTF("soc_input_data[%d]",i);
    //     for(int j=0;j<SOC_INPUT_TYPE_NUM;j++){
    //         PRJ_SOC_PRINTF(" %f",soc_input_data[i][j]);
    //     }
    //     PRJ_SOC_PRINTF("\r\n");
    // }
    PRJ_SOC_LOGI("SOC modle uniformization start.......");
    uniformization_interface(soc_input_data,inputWicket,row,column,soc_input_index,1<<3);
    for(int i=0;i<SOC_INPUT_WINDOW_SIZE;i++){
        PRJ_SOC_PRINTF("inputWicket[%d]",i);
        for(int j=0;j<SOC_INPUT_TYPE_NUM;j++){
            PRJ_SOC_PRINTF(" %f",inputWicket[i*column+j]);
        }
        PRJ_SOC_PRINTF("\r\n");
    }
    // 记录开始时间(us)
    start_time = esp_timer_get_time();
    PRJ_SOC_LOGI("SOC modle inference start...");
    tflm_run(&soc_modle,inputWicket,SOC_INPUT_WINDOW_SIZE*SOC_INPUT_TYPE_NUM,soc_output_data,SOC_OUTPUT_SIZE);
    // 记录结束时间(us)
    end_time = esp_timer_get_time();
    // 计算耗时 (ms)
    elapsed_time_ms = (end_time - start_time)/1000;
    if(elapsed_time_ms > max_time)  max_time = elapsed_time_ms;
    PRJ_SOC_LOGI("SOC modle inference finish...,use time = %d ms",(int)elapsed_time_ms);
    PRJ_SOC_LOGW("SOC modle inference finish...,max use time = %d ms",(int)max_time);
    for(int i=0;i<soc_modle.result_num;i++){
        PRJ_SOC_PRINTF("SOC modle inference result[%d]: %f",i,soc_output_data[i]);
    }
    // uint16_t soc_1h = soc_output_data[0]*10000;
    // uint16_t soc_2h = soc_output_data[1]*10000;
    // modbus_reg_write(SOC_PREDICTION_1H_REG,(uint16_t *)&soc_1h,1);
    // modbus_reg_write(SOC_PREDICTION_2H_REG,(uint16_t *)&soc_2h,1);
    socPredFlag = true;

    PRJ_SOC_PRINTF("\r\n");
    heap_caps_free(inputWicket);
    // vTaskDelay(pdMS_TO_TICKS(1000));
}

float *soc_prediction_result_get(void){
    return soc_output_data;
}

void soc_input_data_fill(float *data,uint16_t num){
    memcpy(soc_input_data[soc_input_index],data,num*sizeof(float));
    soc_input_index++;
    soc_input_index %= SOC_INPUT_WINDOW_SIZE;

    if(soc_input_index == 0){
        soc_input_full = true;
    }else{;}
}

void soc_modle_init(void)
{
    modbus_reg_read(0x0003,&capacity,1);

    soc_modle.interpreter = NULL;
    soc_modle.model_data = soc_model_data;

    tflm_create(&soc_modle);
    // xTaskCreatePinnedToCore(soc_inference_task_handler,"soc_task",1024*4,NULL,8,NULL,1);
}






