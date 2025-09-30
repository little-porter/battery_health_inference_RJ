#include "soc_estimate.h"
#include "ocv_soc.h"
#include "modbus.h"

#include "esp_timer.h"

#define VOLTAGE_REG      0x1003
#define CURRENT_REG      0x1000
#define CAPACITY_REG     0x0003

float g_socValue = 0;
float g_capacity = 0;

#define FILTER_SIZE 10
typedef struct _filter_t
{
    int16_t data[FILTER_SIZE];
    uint8_t fullFlag;
    uint8_t index;
}filter_t;

filter_t g_current_filter = {0};
filter_t g_voltage_filter = {0};

int16_t filter_data_calculate(filter_t *filter,int16_t newData)
{
    filter->data[filter->index] = newData;
    filter->index++;
    if(filter->index >= FILTER_SIZE){
        filter->index = 0;
        filter->fullFlag = 1;
    }

    int32_t sum = 0;
    uint16_t count = filter->fullFlag ? FILTER_SIZE : filter->index;
    int16_t max_data = filter->data[0];
    int16_t min_data = filter->data[0];
    for(int i=0;i<count;i++){
        if(filter->data[i] > max_data) max_data = filter->data[i];
        if(filter->data[i] < min_data) min_data = filter->data[i];
        sum += filter->data[i];
    }
    if(filter->fullFlag){
        sum = sum - max_data - min_data;
        return sum/(FILTER_SIZE-2);
    }else{
        return sum/count;
    }
}


void soc_init_value_get(void)
{
    int16_t ocvVoltage = 0;
    float voltage = 0;
    uint16_t capacity;
    modbus_reg_read(VOLTAGE_REG,(uint16_t *)&ocvVoltage,1);
    modbus_reg_read(CAPACITY_REG,&capacity,1);
    voltage = abs(ocvVoltage)*1.0/1000;
    g_socValue = ocv_soc_get(voltage);
    g_capacity = capacity;
   
}

void soc_calibrate(void)
{
    static int64_t keepSilentTime = 0,startTime = 0,endTime = 0;
    int16_t current = 0; 
    modbus_reg_read(CURRENT_REG,(uint16_t *)&current,1);
    current = filter_data_calculate(&g_current_filter,current);

    if(abs(current)<10){
        if(keepSilentTime == 0){
            startTime = esp_timer_get_time();
        }else{
            endTime = esp_timer_get_time();
            keepSilentTime = endTime - startTime;
            startTime = endTime;
        }

        if(keepSilentTime > 30*60*1000*1000){
            soc_init_value_get();
        }
    }else{
        keepSilentTime = 0;
        startTime = endTime = 0;
    }
    
}

void soc_discharge_calibrate(void)
{
    int16_t voltage = 0;
    modbus_reg_read(VOLTAGE_REG,(uint16_t *)&voltage,1);
    voltage = filter_data_calculate(&g_voltage_filter,voltage);
    if(abs(voltage)<3*1000){
        g_socValue = 0;
    }

}

void soc_charge_calibrate(void)
{
    int16_t voltage = 0,current = 0;
    modbus_reg_read(VOLTAGE_REG,(uint16_t *)&voltage,1);
    voltage = filter_data_calculate(&g_voltage_filter,voltage);
    modbus_reg_read(CURRENT_REG,(uint16_t *)&current,1);
    current = filter_data_calculate(&g_current_filter,current);
    if(abs(voltage)>4*1000 && abs(current)<10){
        g_socValue = 1.00;
    }

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

        modbus_reg_read(CURRENT_REG,(uint16_t *)&current,1);
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



