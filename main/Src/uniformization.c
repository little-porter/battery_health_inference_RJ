#include "uniformzation.h"



static char *TAG = "uniformization";


void uniformization_interface(void *src_window,void *des_window,uint64_t row,uint64_t column,uint64_t now_index)
{
    float *max = (float *)heap_caps_calloc(1,column*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);       //最大值
    float *min = (float *)heap_caps_calloc(1,column*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);       //最小值
    float *diff = (float *)heap_caps_calloc(1,column*sizeof(float),MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);      //差值

    float (*data_window)[column] = (float *)src_window;
    float (*uniformization_data)[column] = (float *)des_window;

    if((NULL == data_window) || (des_window == NULL) || (max == NULL) || (min == NULL) || (diff == NULL)){
        ESP_LOGE(TAG,"malloc error");
    }


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
    }

    //进行数据归一化
    for(int i=0;i<row;i++){
        uint16_t index_row = 0;
        if(now_index+i > row)
        {
            index_row = row + i - now_index;
        }
        else
        {
            index_row = now_index + i;
        }

        for(int j=0;j<column;j++){
            if(diff[j] != 0){
                uniformization_data[i][j] = (data_window[index_row][j] - min[j])/diff[j];
            }else{
                uniformization_data[i][j] = 0.999;
            }
        }
    }

    //释放内存
    heap_caps_free(max);       
    heap_caps_free(min);
    heap_caps_free(diff);
}






