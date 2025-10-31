#include "devStatus.h"
#include "modbus.h"


#define DEVSTATUS_ALARM_STATUS_REG          0x200C
#define DEVSTATUS_FIRE_STATUS_REG           0x200E
#define DEVSTATUS_FAULT_STATUS_REG          0x5000
#define DEVSTATUS_RESULT_STATUS_REG         0x2000


#define DEVSTATUS_STATUS_REG                0x100B

static uint16_t devResult[12] = {0};

static uint16_t devStatus = 0;

#define DEVSTATUS_RESULT_BIT                0
#define DEVSTATUS_FAULT_BIT                 1

void devStatus_task_handler(void *pvParameters){
    while(1){
        uint16_t alarmStatus = 0, fireStatus = 0, faultStatus[6] = {0};
        uint16_t result[12] = {0};
        modbus_reg_read(DEVSTATUS_ALARM_STATUS_REG,&alarmStatus,1);
        modbus_reg_read(DEVSTATUS_FIRE_STATUS_REG,&fireStatus,1);
        modbus_reg_read(DEVSTATUS_FAULT_STATUS_REG,faultStatus,6);
        modbus_reg_read(DEVSTATUS_RESULT_STATUS_REG,result,12);
        if(alarmStatus != 0 || fireStatus != 0 || memcmp(devResult,result,12*2) != 0){
            devStatus |= (1 << DEVSTATUS_RESULT_BIT);
            memcpy(devResult,result,12*2);
        }else{
            devStatus &= ~(1 << DEVSTATUS_RESULT_BIT);
        }

        for(int i = 0;i < 6;i++){
            if(faultStatus[i] != 0){
                devStatus |= (1 << DEVSTATUS_FAULT_BIT);
                break;
            }else{
                devStatus &= ~(1 << DEVSTATUS_FAULT_BIT);
            }
        }

        modbus_reg_write(DEVSTATUS_STATUS_REG,&devStatus,1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



void devStatus_init(void){
    devStatus = 0;
    modbus_reg_write(DEVSTATUS_STATUS_REG,&devStatus,1);
    modbus_reg_read(DEVSTATUS_RESULT_STATUS_REG,devResult,12);

    xTaskCreatePinnedToCore(devStatus_task_handler,"devStatusTask",1024*5,NULL,6,NULL,0);
}


