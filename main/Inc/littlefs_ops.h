#pragma once
#ifndef __LITTLEFS_OPS_H__
#define __LITTLEFS_OPS_H__

#include "sys.h"

#include "esp_littlefs.h"

typedef struct
{
    char    *data;
    uint32_t size;
}littlefs_file_data_t;


void littlefs_ops_init(void);

void littlefs_test(void);

char *littlefs_ops_read_file(const char *file_name,littlefs_file_data_t *file_data);
bool littlefs_ops_write_file(const char *file_name, const char *file_data,uint32_t data_len);
bool littlefs_ops_write_file_append(const char *file_name, const char *file_data,uint32_t data_len);

bool littlefs_ops_remove_file(const char *file_name);

#endif








