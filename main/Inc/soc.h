#ifndef _SOC_H
#define _SOC_H

#include "sys.h"

void soc_modle_init(void);
void soc_input_data_fill(float *data,uint16_t num);
void soc_inference_task_handler(void *parameters);

#endif
