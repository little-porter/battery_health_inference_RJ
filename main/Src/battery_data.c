#include "battery_data.h"
#include "modbus.h"
#include "collect_device.h"
#include "ocv_soc.h"
#include "esp_timer.h"
#include <math.h>
#include "soc.h"
#include "soh.h"

static char *TAG = "PRJ_BatteryData";


#define BATTERY_CURRENT_REG   0x1000
#define BATTERY_VOLTAGE_REG   0x1003
#define BATTERY_INTRES_REG    0x1004
#define BATTERY_TEMP_REG      0x1005

// #define BATTERY_TEMP_REG      0x1005

#define BATTERY_SOC_REG         0x2000
#define BATTERY_SOC_PRED_1H_REG 0x2001
#define BATTERY_SOH_PRED_1H_REG 0x2002
#define BATTERY_SOH_REG         0x2003
#define BATTERY_SOH_PRED_1L_REG 0x2004

#define BATTERY_STATIC_VALUE   0.05


bool batteryValidFlag = false;      //电池数据有效标志

float batterySOC = 0;               //电池SOC
float batteryVoltage = 0;
float batteryCap = 2;

float socPred_1h = 0, socPred_2h = 0;


typedef struct _battery_data{
    float current;
    float voltage;
    float temperature;
    float soc;
    float soh;
    float internalRes;
    float chargeStatus;
    float incrementCap;
    float chargeTime;
    float cRate;
    float ICArea;
    float qVF;
}battery_data_t;

battery_data_t batteryData;

void battery_data_soc_initValue_get(void){
    int16_t ocvVoltage = 0;
    float voltage = 0;
    modbus_reg_read(BATTERY_VOLTAGE_REG,(uint16_t *)&ocvVoltage,1);
    voltage = (ocvVoltage)*1.0/1000;
    batteryData.soc = ocv_soc_get(fabsf(voltage));

    ESP_LOGE("SOC","voltage:%.3f,batterySOC:%.3f",voltage,batteryData.soc);
}

void battery_data_chargeTime_get(void){
    static float lastChargeStatus = 0;
    static int64_t lastTime = 0,nowTime = 0;

    nowTime = esp_timer_get_time();

    if(lastChargeStatus != batteryData.chargeStatus && batteryData.chargeStatus != 0){       //如果状态改变,且当前状态非静止，充放电时间归零
        batteryData.chargeTime = 0;
    }else{
        if(batteryData.chargeStatus == 0){
            batteryData.chargeTime = batteryData.chargeTime;
        }else{
            batteryData.chargeTime += (nowTime - lastTime)/1000/1000;
        }    
    }

    lastTime = nowTime;
    if(batteryData.chargeStatus != 0){                  //上一次状态只保存非静止状态
        lastChargeStatus = batteryData.chargeStatus;
    }
    
    ESP_LOGE(TAG,"状态:%f,chargeTime:%.3f",batteryData.chargeStatus,batteryData.chargeTime);
}

void battery_current_update(void){
    int16_t current = 0;
    modbus_reg_read_no_reverse(0x001E,(uint16_t *)&current,1);
    modbus_reg_write_no_reverse(BATTERY_CURRENT_REG,(uint16_t *)&current,1);
}

void battery_data_get(void){
    uint16_t newData[6] = {0};
    int16_t *pData = (int16_t *)newData;
    modbus_reg_read(BATTERY_CURRENT_REG,newData,6);

    batteryData.current = pData[0]*1.0/1000;
    batteryData.voltage = pData[3]*1.0/1000;
    batteryData.internalRes = newData[4]*1.0/100;
    batteryData.temperature = ((int16_t)newData[5])*1.0/100;

    if(batteryData.current > BATTERY_STATIC_VALUE){
        batteryData.chargeStatus = 1;
    }else if(batteryData.current < (0-BATTERY_STATIC_VALUE)){
        batteryData.chargeStatus = 2;
    }else{
        batteryData.chargeStatus = 0;
    }

    // batteryData.chargeStatus = (batteryData.current > BATTERY_STATIC_VALUE)?1:0;
    // batteryData.chargeStatus = (batteryData.current < -BATTERY_STATIC_VALUE)?2:batteryData.chargeStatus;

    battery_data_chargeTime_get();
}

void battery_data_soc_calibrate(void){
    //1、静止OCV校准
    static int64_t keepSilentTime = 0,startTime = 0,endTime = 0;
    static bool startFlag = true;

    if(fabsf(batteryData.current)<BATTERY_STATIC_VALUE){
        if(startFlag){
            startTime = esp_timer_get_time();
            startFlag = false;
            keepSilentTime = 0;
        }else{
            endTime = esp_timer_get_time();
            keepSilentTime += (endTime - startTime);
            startTime = endTime;
        }

        if(keepSilentTime > 60*1000*1000){
            batteryData.soc = ocv_soc_get(batteryData.voltage);
            ESP_LOGE("SOC","batterySOC:%.3f",batteryData.soc);
        }
    }else{
        startFlag = true;
        keepSilentTime = 0;
        startTime = endTime = 0;
    }
    
    //2、满充校准
    if(batteryData.voltage >= 4.18 && fabsf(batteryData.current)<BATTERY_STATIC_VALUE){
        batteryData.soc = 100.0;
        ESP_LOGE("SOC","current:%.3f, batterySOC:%.3f",batteryData.current,batteryData.soc);
    }else{;}

    //3、满放校准
    if(batteryData.voltage <= 3.0){
        batteryData.soc = 0.00;
        ESP_LOGE("SOC","batterySOC:%.3f",batteryData.soc);
    }else{;}
}

void battery_data_soc_calculate(void){ 
    static int64_t lastTime = 0,currentTime = 0,spanTime = 0;
    float time = 0,dq = 0;
    static bool startFlag = true;
    if(startFlag){
        spanTime = 0;
        currentTime = esp_timer_get_time();
        lastTime = esp_timer_get_time();
        startFlag = false;
    }else{
        currentTime = esp_timer_get_time();
        spanTime = currentTime - lastTime;
        lastTime = currentTime;
    }
    time = spanTime*1.0/(1000*1000);   //单位s
    
    if(time > 0){
        dq = batteryData.current*time/3600;   //单位Ah,安时积分
        batteryData.incrementCap += dq;
    }

    
    
    if(fabsf(batteryData.current) > BATTERY_STATIC_VALUE){           //电流大于0.01A才进行SOC计算
        batteryData.soc += (100*dq/batteryCap);                      //SOC计算
    }

    if(batteryData.soc > 100)   batteryData.soc = 100;
    if(batteryData.soc < 0.00)   batteryData.soc = 0.00;
    ESP_LOGE("SOC","dq:%.8f,time:%.06f, batterySOC:%.3f,current:%.3f",dq,time,batteryData.soc,batteryData.current);
}

void battery_data_soc_prediction(void){
    // float socPred_1h = 0, socPred_2h = 0;
    float chargeFinishTime = 0;

    if(batteryData.chargeStatus == 0){
        socPred_1h = socPred_2h = batteryData.soc;
    }else{ 
        socPred_1h = batteryData.soc + ((batteryData.current*1.0)/batteryCap*100);      //1h Soc预测
        socPred_2h = batteryData.soc + ((batteryData.current*2.0)/batteryCap*100);      //2h Soc预测

        if(batteryData.chargeStatus == 1){                  //充电时间计算
            chargeFinishTime = (2 - batteryData.soc*2/100)/batteryData.current;
        }else{                                                //放电时间计算
            chargeFinishTime = (batteryData.soc*2/100)/batteryData.current;
        }

        if(socPred_1h > 100)    socPred_1h = 100;
        if (socPred_1h < 0)     socPred_1h = 0;
        if(socPred_2h > 100)    socPred_2h = 100;
        if (socPred_2h < 0)     socPred_2h = 0;
    }

    ESP_LOGE(TAG,"socPred_1h:%.3f, socPred_2h:%.3f, chargeStatus:%f, chargeFinishTime:%.3f h",socPred_1h,socPred_2h,batteryData.chargeStatus,chargeFinishTime);
}

void battery_data_soh_calculate(void){
    static float lastVoltage = 0;
    static float dq = 0;
    static uint32_t index = 0;
    static int64_t lastTime = 0,currentTime = 0,spanTime = 0;
    if(batteryData.chargeStatus == 2){
        lastVoltage = 0;
        index = 0;
        dq = 0;
        return;
    }
    currentTime = esp_timer_get_time();
    spanTime = currentTime - lastTime;

    batteryData.cRate = batteryData.current/batteryCap;
    if(lastTime){
        dq += batteryData.current*spanTime/3600;
    }
    lastTime = currentTime; 

    if(fabsf(batteryData.voltage) > 3.0 && fabsf(batteryData.voltage) < 4.0){
        if(batteryData.voltage - lastVoltage > 0.01){       //电压上升0.01进行一次数据记录
            lastVoltage = batteryData.voltage;
            batteryData.qVF = dq/batteryCap;
            batteryData.ICArea = dq;
            float sohInput[6] = {batteryData.soh,batteryData.voltage,batteryData.current,batteryData.cRate,batteryData.ICArea,batteryData.qVF};
            soh_input_data_fill(sohInput,6,index);
            index++;
            ESP_LOGW("SOH","index:%d",(int)index);
            ESP_LOGE("SOH","soh:%.3f, voltage:%.3f, current:%.3f, cRate:%.3f, ICArea:%.3f, dq:%.3f",batteryData.soh,batteryData.voltage,batteryData.current,batteryData.cRate,batteryData.ICArea,batteryData.qVF);
        }else{;}
    }
}



void battery_data_write_result(void){
    uint16_t soc = batteryData.soc*100;
    modbus_reg_write(BATTERY_SOC_REG,(uint16_t *)&soc,1);
    uint16_t soh = batteryData.soh*100;
    modbus_reg_write(BATTERY_SOH_REG,(uint16_t *)&soh,1);

    uint16_t socPredResult[2] = {0};
    if(socPredFlag){
        float *socPred = soc_prediction_result_get();
        socPredResult[0] = socPred[0]*10000;
        socPredResult[1] = socPred[1]*10000;
        modbus_reg_write(BATTERY_SOC_PRED_1H_REG,(uint16_t *)socPredResult,2);
    }else{
        socPredResult[0] = socPred_1h*100;
        socPredResult[1] = socPred_2h*100;
        modbus_reg_write(BATTERY_SOC_PRED_1H_REG,(uint16_t *)socPredResult,2);
    }
    uint16_t sohPredResult = 0;
    if(sohPredFlag){
        float *sohPred = soh_prediction_result_get();
        sohPredResult = sohPred[0]*10000;
        modbus_reg_write(BATTERY_SOH_PRED_1L_REG,(uint16_t *)&sohPredResult,1);
    }else{
        float sohPred = (batteryData.soh*3000-1)/3000;
        sohPredResult = sohPred*100;
        modbus_reg_write(BATTERY_SOH_PRED_1L_REG,(uint16_t *)&sohPredResult,1);
    }
}

void battery_data_get_task_handler(void *pvParameters){
    uint32_t runTime = 0,offlineTime = 0;
    
    while(1){
        vTaskDelay(pdMS_TO_TICKS(1000));

        //变送器电更新
        battery_current_update();

        //采集底板离线检测
        if(collect_device_online_status_get() != true){
            offlineTime++;
            continue;
        }
    
        //获取新数据
        battery_data_get();

        //SOC校准
        battery_data_soc_calibrate();

        //实时SOC评估
        battery_data_soc_calculate();

        //SOC预测
        battery_data_soc_prediction();

        //SOH数据输入
        battery_data_soh_calculate();

        //更新结果
        battery_data_write_result();

        //模型数据输入
        if(runTime == 0){
            float socInput[5] = {batteryData.chargeStatus,batteryData.voltage,batteryData.chargeTime,batteryData.soc/100,batteryData.incrementCap};
            soc_input_data_fill(socInput,5);
            batteryValidFlag = true; 
            ESP_LOGW(TAG,"socInput chargeStatus:%f, voltage:%.3f, chargeTime:%.3f, soc:%.3f, incrementCap:%f",
                batteryData.chargeStatus,batteryData.voltage,batteryData.chargeTime,batteryData.soc/100,batteryData.incrementCap);
        }

        runTime++,runTime %= 180;
        //将数据有效标志位置位，用于模型输入 
    }   
}

void battery_inference_task_handler(void *pvParameters){ 
    while(1){
        soc_inference_task_handler(NULL);
        soh_inference_task_handler(NULL);
        vTaskDelay(pdMS_TO_TICKS(10*1000));
    }
}


void battery_data_init(void){
    batteryValidFlag = false;

    memset(&batteryData,0,sizeof(batteryData));
    batteryData.soh = 100;

    while(!collect_device_online_status_get()){    //等待采集底板上线
        vTaskDelay(pdMS_TO_TICKS(1000));
    } 
    
    // vTaskDelay(pdMS_TO_TICKS(10000));
    //获取初始状态SOC
    battery_data_soc_initValue_get();

    //创建电池数据更新任务
    xTaskCreatePinnedToCore(battery_data_get_task_handler,"batGetDataTask",1024*5,NULL,6,NULL,0);
    //创建推理任务
    xTaskCreatePinnedToCore(battery_inference_task_handler,"batInfTask",1024*5,NULL,6,NULL,1);
}





