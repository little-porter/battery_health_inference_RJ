#include "alarm.h"

#define BATTERY_DATA_RRG    0x1000

#define ALARM_UNDERVOLTAGE  0x0001
#define ALARM_OVERVOLTAGE   0x0002
#define ALARM_OVERCHARGE    0x0004
#define ALARM_OVERDISCHARGE 0x0008
#define ALARM_OVERTEMP      0x0010
#define ALARM_SMK           0x0020
#define ALARM_H2            0x0040
#define ALARM_CO            0x00 0

void device_alarm_handler(void)
{
    int16_t voltage=0,current=0,temp=0;
    uint16_t h2=0,co=0,smk=0;

    uint16_t alarm = 0;

    modbus_reg_read(0x1003,(uint16_t *)&voltage,1);
    modbus_reg_read_no_reverse(0x1000,(uint16_t *)&current,1);
    modbus_reg_read(0x1005,(uint16_t *)&temp,1);
    modbus_reg_read(0x1007,&h2,1);
    modbus_reg_read(0x1006,&co,1);
    modbus_reg_read(0x1008,&smk,1);

    if(smk == 0x0001){
        alarm |= ALARM_SMK;
    }
    if((h2/100) > device_cfg.h2_threshold){
        alarm |= ALARM_H2;
    }  
    if((co/100) > device_cfg.co_threshold){
        alarm |= ALARM_CO;
    }
    if((temp/100) > device_cfg.temp_threshold){
        alarm |= ALARM_OVERTEMP;
    }
    if((current/1000) > device_cfg.current_charge_threshold){
        alarm |= ALARM_OVERCHARGE;
    }else if((current/1000) < (0-device_cfg.current_discharge_threshold)){
        alarm |= ALARM_OVERDISCHARGE;
    }
    if(abs(voltage/1000) > device_cfg.voltage_high_threshold){
        alarm |= ALARM_OVERVOLTAGE;
    }else if(abs(voltage/1000) < device_cfg.voltage_low_threshold){
        alarm |= ALARM_UNDERVOLTAGE;
    }
    ESP_LOGI(TAG, "氢气:%d ,一氧化�?:%d",(int)h2,(int)co);
    modbus_reg_write(0x200C,&alarm,1);
}







