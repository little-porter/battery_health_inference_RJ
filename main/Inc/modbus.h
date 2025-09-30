#ifndef __MODBUS_H__
#define __MODBUS_H__

#include <stdint.h>
#include <string.h>

void modbus_generate_crcTable(void);
uint16_t modbus_calculate_crc(uint8_t *data,uint32_t length);
uint16_t modbus_calculate_crc_ota(uint16_t cal_crc,uint8_t *data,uint32_t length);
void modbus_msg_deal_handler(uint8_t *data,uint16_t length);

void rs485_data_send(uint8_t *data,uint16_t len);

void modbus_reg_data_reverse(uint16_t *reg,uint16_t num);
void modbus_reg_write(uint16_t addr,uint16_t *data,uint16_t num);
void modbus_reg_write_no_reverse(uint16_t addr,uint16_t *data,uint16_t num);
void modbus_reg_read(uint16_t addr,uint16_t *data,uint16_t num);
void modbus_reg_read_no_reverse(uint16_t addr,uint16_t *data,uint16_t num);
// void modbus_reg_data_reverse(uint16_t *reg,uint16_t num);

#define DEVICE_ID  13

#endif
