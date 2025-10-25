#ifndef __SOH_H__
#define __SOH_H__


#include "sys.h"

void soh_modle_init(void);
void soh_input_data_fill(float *data,uint16_t num,uint32_t index);
void soh_inference_task_handler(void *parameters);
float *soh_prediction_result_get(void);

extern bool sohPredFlag;

#endif


