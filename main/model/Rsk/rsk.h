#ifndef __RSK_H__
#define __RSK_H__



#include "sys.h"

void rsk_modle_init(void);
void rsk_fft_input_data_fill(float *data,uint16_t num);

void rsk_inference_task_handler(void *parameters);

#endif


