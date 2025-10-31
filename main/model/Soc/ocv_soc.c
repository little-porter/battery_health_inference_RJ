#include "ocv_soc.h"

static const char *TAG = "PRJ_OCV_SOC";

#define PRJ_OCV_SOC_LOG_ENABLE      0                     //soh log enable
#if PRJ_OCV_SOC_LOG_ENABLE
#define PRJ_OCV_SOC_PRINTF(x,...)           printf(x,##__VA_ARGS__)
#define PRJ_OCV_SOC_LOGI(format, ...)       ESP_LOGI(TAG,format, ##__VA_ARGS__)
#define PRJ_OCV_SOC_LOGW(format, ...)       ESP_LOGW(TAG,format, ##__VA_ARGS__)
#else
#define PRJ_OCV_SOC_PRINTF(x,...)          
#define PRJ_OCV_SOC_LOGI(format, ...)       
#define PRJ_OCV_SOC_LOGW(format, ...)       
#endif

#define PRJ_OCV_SOC_LOGE(format, ...)       ESP_LOGE(TAG,format, ##__VA_ARGS__)



#define OCV_SOC_TABLE_SIZE 21

float ocv_soc_table[OCV_SOC_TABLE_SIZE][2] ={
    {100,4.17185},{95,4.101625},{90,4.0401},{85,3.9841},
    {80,3.929375},{75,3.878475},{70,3.83045},{65,3.7871},
    {60,3.74915},{55,3.7061},{50,3.662275},{45,3.641775},
    {40,3.626975},{35,3.61305},{30,3.5995},{25,3.5805},
    {20,3.5475},{15,3.500625},{10,3.458725},{5,3.42715},
    {0,3.1799}
};

float ocv_soc_get(float openVoltage)
{
    int index = 0;
    float currentSoc;
    for(int i=0;i<OCV_SOC_TABLE_SIZE;i++){
        if(openVoltage>=ocv_soc_table[i][1]){
            break;
        }
        index++;
    }

    if(index==0){
        currentSoc = ocv_soc_table[index][0];
    }else if(index == OCV_SOC_TABLE_SIZE){
        currentSoc = ocv_soc_table[index-1][0];
    }
    else{
        currentSoc = (openVoltage - ocv_soc_table[index][1])*(ocv_soc_table[index-1][0]-ocv_soc_table[index][0])/(ocv_soc_table[index-1][1]-ocv_soc_table[index][1])+ocv_soc_table[index][0];
    }

    PRJ_OCV_SOC_LOGW("index:%d",index);
    return currentSoc;
}




