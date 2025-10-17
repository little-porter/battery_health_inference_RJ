#ifndef __NET_CONFIG_H__
#define __NET_CONFIG_H__

#include "sys.h"


void net_config_init(void);
typedef struct _device_config_t
{
    uint16_t modbus_addr;  
    uint16_t device_type;                       //设�?�类�?
    uint16_t device_addr;                       //modbus地址
    uint16_t cap;                               //电池额定容量  
    uint16_t reset_flag;                        //升级标志
    uint16_t low_power_flag;                    //设�?�地址  
    uint16_t balance_falg;
    char serial[20];                            //序列�?
    uint16_t h2_threshold;                      //H2报�?�阈�?
    uint16_t co_threshold;                      //CO报�?�阈�?
    uint16_t temp_threshold;                    //温度报�?�阈�?
    uint16_t temp_up_threshold;                 //温度升报警阈�?
    uint16_t voltage_high_threshold;            //电压高报警阈�?
    uint16_t voltage_low_threshold;             //电压低报警阈�?
    int16_t current_discharge_threshold;        //电流放电报�?�阈�?
    uint16_t current_charge_threshold;          //电流充电报�?�阈�?
}device_config_t;

extern device_config_t device_cfg;

#endif

