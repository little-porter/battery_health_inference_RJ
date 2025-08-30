#ifndef __TFLM_H__
#define __TFLM_H__

#include "sys.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _tflm_module
{
    /* data */
    void *interpreter;                          //解释�?
    const unsigned char *model_data;            //模型数据
    uint32_t input_row;                         //输入�?    
    uint32_t input_col;                         //输入�?
    uint32_t result_num; 
}tflm_module_t;



void tflm_init(void);
void tflm_create(tflm_module_t *tflm);
void tflm_release(tflm_module_t *tflm);
void tflm_run(tflm_module_t *tflm,float *input_data,uint32_t input_num,float *output_data,uint32_t output_num);



#ifdef __cplusplus
}
#endif

#endif





