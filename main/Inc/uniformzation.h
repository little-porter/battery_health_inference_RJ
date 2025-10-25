#ifndef __UNIFORMZATION_H__
#define __UNIFORMZATION_H__

#include "stdint.h"
#include "string.h"
#include "esp_heap_caps.h"

#include "sys.h"

void uniformization_interface(void *src_window,void *des_window,uint64_t row,uint64_t column,uint64_t now_index,uint32_t uniformizationFlag);


#endif

