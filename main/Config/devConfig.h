#ifndef __DEVCONFIG_H__
#define __DEVCONFIG_H__
#include "sys.h"

typedef struct _devConfig{
    uint16_t modbusAddr;                        //modbus地址
    uint16_t deviceType;                        //设备类型
    uint16_t deviceAddr;                        //设备地址
    uint16_t cap;                               //电池额定容量  
    uint16_t resetFlag;                         //复位标志
    uint16_t lowPowerFlag;                      //低功耗标志 
    uint16_t balanceFalg;                       //均衡标志    
    char serial[20];                            //序列号
    uint16_t h2Threshold;                       //H2报警阈值
    uint16_t coThreshold;                       //CO报警阈值
    uint16_t tempThreshold;                     //温度报警阈值
    uint16_t tempUpThreshold;                   //温度升报警阈值
    uint16_t voltageHighThreshold;              //电压高报警阈值
    uint16_t voltageLowThreshold;               //电压低报警阈值
    int16_t currentDischargeThreshold;          //电流放电报警阈值
    uint16_t currentChargeThreshold;            //电流充电报警阈值
    char unixTime[10];
}devConfig_t;
extern devConfig_t devCfg;

void devConfig_init(void);

#endif
