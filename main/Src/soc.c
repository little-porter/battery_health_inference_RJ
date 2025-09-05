#include "soc.h"
#include "tflm.h"
#include "esp_timer.h"
#include "modbus.h"

static const char *TAG = "PRJ_SOC";

extern const int soc_model_data_len;
extern const unsigned char soc_model_data[];

tflm_module_t soc_modle;


float m_soc;
uint16_t capacity;
float m_capacity;

#define SOC_OCV_TABLE_SIZE      11
float soc_ocv_table[2][SOC_OCV_TABLE_SIZE] = {
    {12.725,12.475,12.375,12.275,12.175,12.075,11.975,11.825,11.675,11.475,10.65},
    {100,90,80,70,60,50,40,30,20,10,0}
};

#define SOC_INPUT_WINDOW_TIME           120         //输入时间窗，单位分钟
#define SOC_INPUT_WINDOW_SPAN             3           //输入时间窗内数据间隔时间，单位分钟    
#define SOC_INPUT_WINDOW_SIZE           (SOC_INPUT_WINDOW_TIME/SOC_INPUT_WINDOW_SPAN)  //输入数据个数 120/3 = 40
float *soc_input_window = NULL;

uint16_t soc_input_index = 0;
uint16_t soc_input_full  = 0;

#define SOC_OUTPUT_SIZE                 2
#define SOC_INPUT_TYPE_NUM              5
typedef enum _SOC_INPUT_TYPE
{
    SOC_INPUT_CFDZT = 0,    //充放电状态  
    SOC_INPUT_VOLTAGE,      //电压
    SOC_INPUT_CFDSJ,        //充放电时间
    SOC_INPUT_SOC,          //SOC
    SOC_INPUT_DQ,           //增量容量
}SOC_INPUT_TYPE;
float soc_input_data[SOC_INPUT_TYPE_NUM];
float soc_output_data[SOC_OUTPUT_SIZE];

void soc_inference_task_handler(void *parameters)
{
    int64_t start_time, end_time;
    int64_t elapsed_time_ms;
    while(1){
        // 璁板綍寮�濮嬫椂闂? (寰?绉?)
        start_time = esp_timer_get_time();
        ESP_LOGI(TAG,"SOC modle inference start...");
        tflm_run(&soc_modle,soc_input_data,SOC_INPUT_WINDOW_SIZE,soc_output_data,SOC_OUTPUT_SIZE);
        // 璁板綍缁撴潫鏃堕棿 (寰?绉?)
        end_time = esp_timer_get_time();
        // 璁＄畻鑰楁椂 (寰?绉?)
        elapsed_time_ms = (end_time - start_time)/1000;
        ESP_LOGI(TAG,"SOC modle inference finish...,use time = %d ms",(int)elapsed_time_ms);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void soc_modle_init(void)
{
    modbus_reg_read(0x0003,&capacity,1);

    soc_modle.interpreter = NULL;
    soc_modle.model_data = soc_model_data;
    tflm_init();
    tflm_create(&soc_modle);
    // xTaskCreatePinnedToCore(soc_inference_task_handler,"soc_task",1024*4,NULL,7,NULL,1);
}


void soc_ocv_estimate(void)
{
    uint16_t voltage = 0;
    modbus_reg_read(0x1003,&voltage,1);
    float volt = voltage/1000.0;
    ESP_LOGI(TAG, "battery voltage = %.3f V",volt);
    int index = 0;
    for(int i=0;i<SOC_OCV_TABLE_SIZE;i++){
        index = i;
        if(volt >= soc_ocv_table[0][i]){
            if(i == 0 || i == (SOC_OCV_TABLE_SIZE-1)){
                m_soc = soc_ocv_table[1][i];
            }else{
                float k = (soc_ocv_table[1][i-1] - soc_ocv_table[1][i])/(soc_ocv_table[0][i-1] - soc_ocv_table[0][i]);
                m_soc = soc_ocv_table[1][i] + k*(volt - soc_ocv_table[0][i]);
            }
            break;
        }else{
            continue;
        }
    }
}

void soc_caculate(void)
{
    //soc = soc + dq/dt;
    uint16_t current = 0;
    modbus_reg_read(0x1000,&current,1);
    float curr = current/1000.0;

    m_soc = m_soc + (curr*1.0/3600.0)/capacity;
    if(m_soc < 0){
        m_soc = 0;
    }else if(m_soc > 1){
        m_soc = 1;
    }else{;}
    soc_input_data[SOC_INPUT_SOC] = m_soc;
}

void uniformization(float *data,uint16_t length)
{

}


void uniformization_interface(void *src_window,void *des_window,uint64_t row,uint64_t column,uint64_t now_index)
{
    float *max = (float *)heap_caps_calloc(1,SOC_INPUT_TYPE_NUM*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);       //最大值
    float *min = (float *)heap_caps_calloc(1,SOC_INPUT_TYPE_NUM*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);       //最小值
    float *diff = (float *)heap_caps_calloc(1,SOC_INPUT_TYPE_NUM*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);      //最大最小值差

    float (*data_window)[column] = src_window;
    float (*uniformization_data)[column] = des_window;

    //初始化最大/最小值数组
    for(int i=0;i<column;i++){
        max[i] = data_window[0][i];
        min[i] = data_window[0][i];
    }

    for(int i=0;i<column;i++){              //按列寻找最大值、最小值，计算最大最小值差
        for(int j=0;j<row;j++){       //按行寻找最大值、最小值
            if(data_window[j][i] > max[j]){
                max[j] = data_window[j][i];
            }else{;}
            if(data_window[j][i] < min[j]){
                min[j] = data_window[j][i];
            }else{;}
        }
        diff[i] = max[i] - min[i];
    }

    //归一窗口数据按时间序列填充
    for(int i=0;i<row;i++){
        uint16_t index_row = 0;
        if(soc_input_index+i > row)
        {
            index_row = row + i - soc_input_index;
        }
        else
        {
            index_row = soc_input_index + i;
        }

        for(int j=0;j<column;j++){
            if(diff[j] != 0){
                uniformization_data[i][j] = (data_window[index_row][j] - min[j])/diff[j];
            }else{
                uniformization_data[i][j] = 0.999;
            }
        }
    }

    heap_caps_free(max);
    heap_caps_free(min);
    heap_caps_free(diff);
}

void soc_input_data_uniformization(void *window,uint64_t size)
{
    
}

void soc_inference(void)
{
    uint64_t   window_size = SOC_INPUT_WINDOW_SIZE*SOC_INPUT_TYPE_NUM*sizeof(float);
    float *uniformization_window = (float *)heap_caps_malloc(window_size,MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    


}


