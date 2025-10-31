#include "uniformzation.h"



static char *TAG = "PRJ_UNIFORMIZATION";

#define PRJ_UNIFORMIZATION_LOG_ENABLE      0                     //uniformization log enable

#if PRJ_UNIFORMIZATION_LOG_ENABLE
#define PRJ_UNIFORMIZATION_PRINTF(x,...)           printf(x,##__VA_ARGS__)
#define PRJ_UNIFORMIZATION_LOGI(format, ...)       ESP_LOGI(TAG,format, ##__VA_ARGS__)
#define PRJ_UNIFORMIZATION_LOGW(format, ...)       ESP_LOGW(TAG,format, ##__VA_ARGS__)
#else
#define PRJ_UNIFORMIZATION_PRINTF(x,...)          
#define PRJ_UNIFORMIZATION_LOGI(format, ...)       
#define PRJ_UNIFORMIZATION_LOGW(format, ...)       
#endif

#define PRJ_UNIFORMIZATION_LOGE(format, ...)       ESP_LOGE(TAG,format, ##__VA_ARGS__)


void uniformization_interface(void *src_window,void *des_window,uint64_t row,uint64_t column,uint64_t now_index,uint32_t uniformizationFlag)
{
    float *max = (float *)heap_caps_calloc(1,column*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);       //最大值
    float *min = (float *)heap_caps_calloc(1,column*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);       //最小值
    float *diff = (float *)heap_caps_calloc(1,column*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);      //差值

    float (*data_window)[column] = (float *)src_window;
    float (*uniformization_data)[column] = (float *)des_window;

    if((NULL == data_window) || (des_window == NULL) || (max == NULL) || (min == NULL) || (diff == NULL)){
        PRJ_UNIFORMIZATION_LOGE("malloc error");
    }
    PRJ_UNIFORMIZATION_PRINTF("row = %d,column = %d,now_index = %d\n",(int)row,(int)column,(int)now_index);

    //初始化最大值、最小值
    for(int i=0;i<column;i++){
        max[i] = data_window[0][i];
        min[i] = data_window[0][i];
    }

    for(int i=0;i<column;i++){                      //遍历每类元素
        for(int j=0;j<row;j++){                     //遍历同一种元素数据
            if(data_window[j][i] > max[i]){         //更新最大值
                max[i] = data_window[j][i];
            }else{;}
            if(data_window[j][i] < min[i]){         //更新最小值
                min[i] = data_window[j][i];
            }else{;}
        }
        diff[i] = max[i] - min[i];
        PRJ_UNIFORMIZATION_PRINTF("max[%d] = %f,min[%d] = %f,diff[%d] = %f\n",i,max[i],i,min[i],i,diff[i]);
        PRJ_UNIFORMIZATION_PRINTF("\r\n");
    }

    //进行数据归一化
    for(int i=0;i<row;i++){
        uint16_t index_row = 0;
        if(now_index+i >= row){
            index_row =  i + now_index - row;
        }else{
            index_row = now_index + i;
        }

        for(int j=0;j<column;j++){
            if(uniformizationFlag&(1<<j)){
                uniformization_data[i][j] = data_window[index_row][j];
                // PRJ_UNIFORMIZATION_PRINTF("uniformizationFlag = %d, result = %d\r\n",(int)uniformizationFlag,(int)uniformizationFlag&(1<<j));
                // PRJ_UNIFORMIZATION_PRINTF("uniformization_data[%d][%d] = %f\r\n",i,j,uniformization_data[i][j]);
            }else{
               if(diff[j] != 0){
                    uniformization_data[i][j] = (data_window[index_row][j] - min[j])/diff[j];
                }else{
                    uniformization_data[i][j] = 0;
                }
            }
            
            if(i == row-1){
                PRJ_UNIFORMIZATION_PRINTF("uniformization_data[%d][%d] = %f, data_window[%d][%d] = %f\r\n",i,j,uniformization_data[i][j],index_row,j,data_window[index_row][j]);
            }
        }
    }

    //释放内存
    heap_caps_free(max);       
    heap_caps_free(min);
    heap_caps_free(diff);
}






