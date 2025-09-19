#include "soc_estimate.h"
#include "ocv_soc.h"
#include "modbus.h"

#include "esp_timer.h"

#define VOLTAGE_REG      0x1003
#define CURRENT_REG      0x1000
#define CAPACITY_REG     0x0003

float g_socValue = 0;
float g_capacity = 0;

void soc_init_value_get(void)
{
    int16_t ocvVoltage = 0;
    float voltage = 0;
    uint16_t capacity;
    modbus_reg_read(VOLTAGE_REG,&ocvVoltage,1);
    modbus_reg_read(CAPACITY_REG,&capacity,1);
    voltage = abs(ocvVoltage)*1.0/1000;
    g_socValue = ocv_soc_get(voltage);
    g_capacity = capacity;
   
}


void soc_estimate_handler(void *parameter)
{
    float realCurrent = 0;
    int16_t current = 0; 
    float Q,dQ,dt;

    int64_t  startTime = 0, endTime = 0;
    startTime = endTime = esp_timer_get_time();
    // endTime = esp_timer_get_time();
    dt = (endTime - startTime)*1.0/1000000;
    Q = g_socValue*g_capacity;

    while (1)
    {
        endTime = esp_timer_get_time();
        dt = (endTime - startTime)*1.0/1000000;
        startTime = endTime;

        modbus_reg_read(CURRENT_REG,&current,1);
        realCurrent = current*1.0/1000;

        if(abs(realCurrent)>0.01){
            dQ = realCurrent*dt;
            Q += dQ;
            g_socValue = Q/g_capacity;
        }
        

        vTaskDelay(pdMS_TO_TICKS(1000));        /* code */
    }
    
}


void soc_init(void)
{
    soc_init_value_get();
    xTaskCreatePinnedToCore(soc_estimate_handler,"soc_est_task",1024*2,NULL,5,NULL,0);
}



