#ifndef __IAP_H__
#define __IAP_H__

#include "sysEvent.h"

void iap_msg_deal_handler(uint8_t *data,uint16_t length);

#define iap_crc_calculate(a,b)	modbus_calculate_crc(a,b)

void iap_init(void);

#endif
