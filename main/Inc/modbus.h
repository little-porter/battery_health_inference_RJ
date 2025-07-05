#ifndef __MODBUS_H__
#define __MODBUS_H__

#include <stdint.h>
#include <string.h>

void modbus_generate_crcTable(void);
void modbus_msg_deal_handler(uint8_t *data,uint16_t length);

void rs485_data_send(uint8_t *data,uint16_t len);

void modbus_reg_write(uint16_t addr,uint16_t *data,uint16_t num);

#endif
