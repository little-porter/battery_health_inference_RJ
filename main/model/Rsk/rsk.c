#include "rsk.h"
#include "math.h"
#include "dl_rfft.h"

#include "tflm.h"
#include "esp_timer.h"
#include "uniformzation.h"

static const char *TAG = "PRJ_RSK";

tflm_module_t rsk_modle;
#define RSK_INPUT_TYPE_NUM              15
#define RSK_INPUT_WINDOW_SIZE           180
#define RSK_OUTPUT_SIZE                 20
static uint64_t rsk_input_index = 0;
bool rsk_input_full = false;

float rsk_input_data[RSK_INPUT_WINDOW_SIZE][RSK_INPUT_TYPE_NUM];
float rsk_output_data[RSK_OUTPUT_SIZE];

typedef enum _rsk_input_type{
    RSK_INPUT_TYPE_AVERVOLTAGE      = 0,
    RSK_INPUT_TYPE_MAXVOLTAGE,
    RSK_INPUT_TYPE_MINVOLTAGE,
    RSK_INPUT_TYPE_FFTVOLTAGE,
    RSK_INPUT_TYPE_STFFTVOLTAGE,
    RSK_INPUT_TYPE_AVERTEMPERATURE,
    RSK_INPUT_TYPE_MAXTEMPERATURE,
    RSK_INPUT_TYPE_MINTEMPERATURE,
    RSK_INPUT_TYPE_FFTTEMPERATURE,
    RSK_INPUT_TYPE_STFFTTEMPERATURE,
    RSK_INPUT_TYPE_AVERENVTEMPERATURE,
    RSK_INPUT_TYPE_MAXENVTEMPERATURE,
    RSK_INPUT_TYPE_MINENVTEMPERATURE,
    RSK_INPUT_TYPE_FFTENVTEMPERATURE,
    RSK_INPUT_TYPE_STFFTENVTEMPERATURE,
}rsk_input_type_t;

typedef enum _rsk_fft_input_type{
    RSK_FFT_INPUT_TYPE_VOLTAGE      = 0,
    RSK_FFT_INPUT_TYPE_TEMPERATURE,
    RSK_FFT_INPUT_TYPE_ENVTEMPERATURE,
}rsk_fft_input_type_t;


#define RSK_FFT_INPUT_TYPE_NUM           3
#define RSK_FFT_INPUT_WINDOW_SIZE        120
static uint64_t rsk_fft_input_index = 0;
static bool rsk_fft_input_full = false;

float rsk_fft_input_data[RSK_FFT_INPUT_TYPE_NUM][RSK_FFT_INPUT_WINDOW_SIZE];


extern const int rsk_model_data_len;
extern const unsigned char rsk_model_data[];

extern const int rsk1_model_data_len;
extern const unsigned char rsk1_model_data[];

void rsk_input_data_full(void);

void rsk_inference_task_handler(void *parameters)
{
    rsk_input_data_full();
    if(!rsk_input_full){
        ESP_LOGW(TAG,"RSK input fifo is not full...  index = %d",(int)rsk_input_index);
        return;
    }
    if(rsk_modle.interpreter == NULL){
        ESP_LOGE(TAG,"RSK modle interpreter is not create...");
        return;
    }
    static int64_t max_time;
    int64_t start_time, end_time;
    int64_t elapsed_time_ms;
   
    // 获取开始时间
    start_time = esp_timer_get_time();
    ESP_LOGI(TAG,"RSK modle inference start...");
    float *inputWicket = heap_caps_malloc(RSK_INPUT_WINDOW_SIZE*RSK_INPUT_TYPE_NUM*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    ESP_LOGI(TAG,"RSK modle uniformization start.......");
    uniformization_interface(rsk_input_data,inputWicket,RSK_INPUT_WINDOW_SIZE,RSK_INPUT_TYPE_NUM,rsk_input_index,0);

    tflm_run(&rsk_modle,inputWicket,RSK_INPUT_WINDOW_SIZE*RSK_INPUT_TYPE_NUM,rsk_output_data,RSK_OUTPUT_SIZE);
    // 获取结束时间
    end_time = esp_timer_get_time();
    // 计算推理耗时
    elapsed_time_ms = (end_time - start_time)/1000;
    if(elapsed_time_ms > max_time)  max_time = elapsed_time_ms;
    ESP_LOGI(TAG,"RSK modle inference finish...,use time = %d ms",(int)elapsed_time_ms);
    ESP_LOGW(TAG,"RSK modle inference finish...,max use time = %d ms",(int)max_time);

    for(int i=0;i<rsk_modle.result_num;i++){
        printf("RSK modle inference result[%d]: %f",i,rsk_output_data[i]);
    }
    printf("\r\n");
    heap_caps_free(inputWicket);
    // vTaskDelay(pdMS_TO_TICKS(1000));
}



void rsk_fft_input_data_fill(float *data,uint16_t num){
    uint16_t size = 0;
    if(num > RSK_FFT_INPUT_TYPE_NUM){
        size = RSK_FFT_INPUT_TYPE_NUM;
    }else{
        size = num;
    }
    
    for(int i=0; i<size; i++){
        rsk_fft_input_data[i][rsk_fft_input_index] = data[i];
    }
    rsk_fft_input_index++;
    rsk_fft_input_index %= RSK_FFT_INPUT_WINDOW_SIZE;
    if(rsk_fft_input_index == 0){
        rsk_fft_input_full = true;
    }else{;}
}


void rsk_fft_wicket_full(float *wicket_data,const float *input_data,int len)
{
    for (int i = 0; i < len; i++) {
        float win = 0.5f * (1.0f - cosf(2 * M_PI  * i / (len - 1)));        //汉宁窗算法
        wicket_data[i] = input_data[i] * win;
    }
}


float rsk_fft_run(float *input_data,uint16_t len,uint16_t nowIndex){
    /* FFT 输入为2的幂指数 */
    int point = 1;
    while(point < len) {point <<= 1;}

    // FFT初始化
    dl_fft_f32_t *fft_handle = dl_rfft_f32_init(point,MALLOC_CAP_SPIRAM);
    printf("fft_handle.log2n = %d\n",fft_handle->log2n);

    //输入数据按时间顺序排列
    float *input_wicket = (float *)heap_caps_malloc(point * sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    memset(input_wicket, 0, point * sizeof(float));
    for(int i = 0;i < len;i++){
        int index = (nowIndex + i) % len;
        input_wicket[i] = input_data[index];
    }
    // for(int i = 0;i < point;i++){
    //     printf(" %f",input_wicket[i]);
    // }
    // printf("\r\n");
    //加窗处理
    float *wicket = (float *)heap_caps_malloc(point * sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    memset(wicket, 0, point * sizeof(float));
    memcpy(wicket, input_wicket, len * sizeof(float));
    // rsk_fft_wicket_full(wicket,input_wicket,len);
    // for(int i = 0;i < point;i++){
    //     printf(" %f",wicket[i]);
    // }
    printf("\r\n");
    // FFT处理
    dl_rfft_f32_run(fft_handle,wicket);
    // for(int i = 0;i < point;i++){
    //     printf(" %f",wicket[i]);
    // }
    // printf("\r\n");

    //计算输出
    float output_data = 0;
     
    for(int i = 0;i < point/2;i++)
    {
        float real = wicket[2 * i];
        float imag = wicket[2 * i + 1];
        float magnitude = sqrtf(real * real + imag * imag);
        if(i == 0){
            printf("     magnitude = %f\r\n",magnitude);
            printf("i=%d: real = %f,imag = %f\r\n",i,real,imag);
            output_data = magnitude;
        }
        // output_data += magnitude;
    }
    printf(" output = %f\r\n",output_data);
    dl_rfft_f32_deinit(fft_handle);
    heap_caps_free(input_wicket);
    heap_caps_free(wicket);
    return output_data;
}

void rsk_input_data_full(void){
    if(!rsk_fft_input_full){
        ESP_LOGW(TAG,"RSK FFT input fifo is not full...  index = %d",(int)rsk_fft_input_index);
        return;
    }
    //计算fft窗口单类数据和,寻找最大最小值
    float sumVoltage = 0,sumTemperature = 0,sumEnvTemperature = 0;
    float minVoltage = 0,minTemperature = 0,minEnvTemperature = 0;
    float maxVoltage = 0,maxTemperature = 0,maxEnvTemperature = 0;
    minVoltage = maxVoltage = rsk_fft_input_data[0][0];
    minTemperature = maxTemperature = rsk_fft_input_data[1][0];
    minEnvTemperature = maxEnvTemperature = rsk_fft_input_data[2][0];

    for(int i=0;i<RSK_FFT_INPUT_WINDOW_SIZE;i++){
        sumVoltage += rsk_fft_input_data[0][i];
        sumTemperature += rsk_fft_input_data[1][i];
        sumEnvTemperature += rsk_fft_input_data[2][i];

        if(rsk_fft_input_data[0][i] < minVoltage){
            minVoltage = rsk_fft_input_data[0][i];
        }else{;}
        if(rsk_fft_input_data[0][i] > maxVoltage){
            maxVoltage = rsk_fft_input_data[0][i];
        }else{;}
        if(rsk_fft_input_data[1][i] < minTemperature){
            minTemperature = rsk_fft_input_data[1][i];
        }else{;}
        if(rsk_fft_input_data[1][i] > maxTemperature){
            maxTemperature = rsk_fft_input_data[1][i];
        }else{;}
        if(rsk_fft_input_data[2][i] < minEnvTemperature){
            minEnvTemperature = rsk_fft_input_data[2][i];
        }else{;}
        if(rsk_fft_input_data[2][i] > maxEnvTemperature){
            maxEnvTemperature = rsk_fft_input_data[2][i];
        }else{;}
    }   

    //计算fft窗口单类数据均值
    float avgVoltage = sumVoltage / RSK_FFT_INPUT_WINDOW_SIZE;
    float avgTemperature = sumTemperature / RSK_FFT_INPUT_WINDOW_SIZE;
    float avgEnvTemperature = sumEnvTemperature / RSK_FFT_INPUT_WINDOW_SIZE;


    float fftVoltage = rsk_fft_run(rsk_fft_input_data[0],RSK_FFT_INPUT_WINDOW_SIZE,rsk_fft_input_index);
    ESP_LOGE(TAG,"RSK FFT Voltage = %f",fftVoltage);
    float fftTemperature = rsk_fft_run(rsk_fft_input_data[1],RSK_FFT_INPUT_WINDOW_SIZE,rsk_fft_input_index);
    ESP_LOGE(TAG,"RSK FFT fftTemperature = %f",fftTemperature);
    float fftEnvTemperature = rsk_fft_run(rsk_fft_input_data[2],RSK_FFT_INPUT_WINDOW_SIZE,rsk_fft_input_index);
    ESP_LOGE(TAG,"RSK FFT fftEnvTemperature = %f",fftEnvTemperature);
    
    //填充rsk输入数据
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_AVERVOLTAGE] = avgVoltage;
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_MAXVOLTAGE] = maxVoltage;
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_MINVOLTAGE] = minVoltage;
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_FFTVOLTAGE] = fftVoltage;
    // rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_STFFTVOLTAGE] = fftVoltage / avgVoltage;
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_AVERTEMPERATURE] = avgTemperature;
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_MAXTEMPERATURE] = maxTemperature;
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_MINTEMPERATURE] = minTemperature;
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_FFTTEMPERATURE] = fftTemperature;
    // rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_STFFTTEMPERATURE] = fftTemperature / avgTemperature;
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_AVERENVTEMPERATURE] = avgEnvTemperature;
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_MAXENVTEMPERATURE] = maxEnvTemperature;
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_MINENVTEMPERATURE] = minEnvTemperature;
    rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_FFTENVTEMPERATURE] = fftEnvTemperature;
    // rsk_input_data[rsk_input_index][RSK_INPUT_TYPE_STFFTENVTEMPERATURE] = fftEnvTemperature / avgEnvTemperature;

    rsk_input_index++;
    rsk_input_index %= RSK_INPUT_WINDOW_SIZE;
    if(rsk_input_index == 0){
        rsk_input_full = true;
    }
}



void rsk_modle_init(void)
{

    rsk_modle.interpreter = NULL;
    rsk_modle.model_data = rsk1_model_data;
    tflm_create(&rsk_modle);
    // xTaskCreatePinnedToCore(rsk_inference_task_handler,"rsk_task",1024*7,NULL,7,NULL,1);
}




