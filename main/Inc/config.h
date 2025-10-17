#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "sys.h"
#include "project.h"


uint8_t device_get_modbus_id_form_configJSON(char *serial_str);

void config_msg_deal_handler(uint8_t *data,uint16_t length);

#endif
