#ifndef __CONFIGJSON_H__
#define __CONFIGJSON_H__


#include "sys.h"

void configJson_init(void);
uint16_t configJson_get_modbusAddr_by_serial(char *serial_str);
uint8_t configJson_get_modbusAddr_from_jsonData_by_serial(uint8_t *data,char *serial_str);

#endif
