#include "alarm.h"
#include "devConfig.h"
#include "modbus.h"
#include "bpa.h"

static const char *TAG = "PRJ_ALARM";
#define PRJ_ALARM_LOG_ENABLE                0                    //soh log enable
#if PRJ_ALARM_LOG_ENABLE
#define PRJ_ALARM_PRINTF(x,...)           printf(x,##__VA_ARGS__)
#define PRJ_ALARM_LOGI(format, ...)       ESP_LOGI(TAG,format, ##__VA_ARGS__)
#define PRJ_ALARM_LOGW(format, ...)       ESP_LOGW(TAG,format, ##__VA_ARGS__)
#else
#define PRJ_ALARM_PRINTF(x,...)          
#define PRJ_ALARM_LOGI(format, ...)       
#define PRJ_ALARM_LOGW(format, ...)       
#endif

#define PRJ_ALARM_LOGE(format, ...)       ESP_LOGE(TAG,format, ##__VA_ARGS__)


//1AH电量电解水析出0.418L氢气，0.209L氧气（理论值）  200Ah————>83.6L氢气,(H2O含量1.7~1.8公斤水含220L氢气)
/*****************************************************************************/
//火警相关信息
//氢气:测量范围0~300ppm
//阈值10ppm   fire:0.2;noFire:0.7;fault:0.1
//阈值100ppm  fire:0.5;noFire:0.4;fault:0.1
//阈值200ppm  fire:0.7;noFire:0.2;fault:0.1
//阈值250ppm  fire:0.8;noFire:0.1;fault:0.1

//一氧化碳：测量范围0~1000ppm
//阈值10ppm   fire:0.2;noFire:0.7;fault:0.1
//阈值100ppm  fire:0.3;noFire:0.6;fault:0.1
//阈值200ppm  fire:0.4;noFire:0.5;fault:0.1
//阈值400ppm  fire:0.7;noFire:0.2;fault:0.1
//阈值800ppm  fire:0.8;noFire:0.1;fault:0.1
//阈值1000ppm fire:0.9;noFire:0.1;fault:0.0

//烟雾:测量范围0/1
//阈值0   fire:0.0;noFire:0.7;fault:0.3
//阈值1   fire:0.7;noFire:0.0;fault:0.3

//温度:测量范围-40~125℃
//阈值60℃   fire:0.2;noFire:0.7;fault:0.1
//阈值80℃   fire:0.5;noFire:0.4;fault:0.1
//阈值100℃  fire:0.8;noFire:0.1;fault:0.1
//阈值120℃  fire:0.9;noFire:0.0;fault:0.1

//火警概率
//火警概率大于0.8：火警  ;冲突概率：大于50%：输出预警信息

/*****************************************************************************/


#define BATTERY_DATA_RRG        0x1000
#define BATTERY_CURRENT_RRG     0x1000
#define BATTERY_VOLTAGE_RRG     0x1003
#define BATTERY_TEMP_RRG        0x1005
#define BATTERY_CO_RRG          0x1006
#define BATTERY_H2_RRG          0x1007
#define BATTERY_SMK_RRG         0x1008

#define ALARM_UNDERVOLTAGE      0x0001
#define ALARM_OVERVOLTAGE       0x0002
#define ALARM_OVERCHARGE        0x0004
#define ALARM_OVERDISCHARGE     0x0008
#define ALARM_OVERTEMP          0x0010
#define ALARM_SMK               0x0020
#define ALARM_H2                0x0040
#define ALARM_CO                0x0080

#define ALARM_RESULT_REG        0x2011

typedef enum _alarm_input{
    ALARM_INPUT_VOLTAGE = 0,
    ALARM_INPUT_CURRENT,
    ALARM_INPUT_TEMP,
    ALARM_INPUT_H2,
    ALARM_INPUT_CO,
    ALARM_INPUT_SMK,
}alarm_input_t;

typedef enum _alarm_fire_out{
    ALARM_FIRE_OUT_WARNING = 0,
    ALARM_FIRE_OUT_FIRE = 1, 
}alarm_fire_out_t;

#define ALARM_INPUT_NUM 6
static float alarm_input_data[ALARM_INPUT_NUM] = {0};

bpa_t bpaH2,bpaCO,bpaSMK,bpaTEMP,bpaResult;
alarm_fire_out_t alarmFireOut = ALARM_FIRE_OUT_WARNING;

void alarm_bpa_init(void){
    bpa_unit_init(&bpaH2);
    bpa_unit_init(&bpaCO);
    bpa_unit_init(&bpaSMK);
    bpa_unit_init(&bpaTEMP);
    bpa_unit_init(&bpaResult);
}




void alarm_input_data_fill(float *data,uint16_t num){
    if(num < ALARM_INPUT_NUM){
        memcpy(alarm_input_data,data,num*sizeof(float));
    }else{
        memcpy(alarm_input_data,data,ALARM_INPUT_NUM*sizeof(float));
    }
}

void alarm_data_get(void){
    float alarmInput[ALARM_INPUT_NUM] = {0};
    int16_t voltage = 0, current = 0, temp = 0;
    uint16_t h2 = 0, co = 0, smk = 0;

    modbus_reg_read(BATTERY_CURRENT_RRG,(uint16_t *)&current,1);
    modbus_reg_read(BATTERY_VOLTAGE_RRG,(uint16_t *)&voltage,1);
    modbus_reg_read(BATTERY_TEMP_RRG,(uint16_t *)&temp,1);
    modbus_reg_read(BATTERY_CO_RRG,&co,1);
    modbus_reg_read(BATTERY_H2_RRG,&h2,1);
    modbus_reg_read(BATTERY_SMK_RRG,&smk,1);
    alarmInput[ALARM_INPUT_VOLTAGE] = voltage*1.0/1000;
    alarmInput[ALARM_INPUT_CURRENT] = current*1.0/1000;
    alarmInput[ALARM_INPUT_TEMP]    = temp*1.0/100;
    alarmInput[ALARM_INPUT_H2]      = h2*1.0/100;
    alarmInput[ALARM_INPUT_CO]      = co*1.0/100;
    alarmInput[ALARM_INPUT_SMK]     = smk;

    alarm_input_data_fill(alarmInput,ALARM_INPUT_NUM);
}

void alarm_fire_detection(void){ 
    //氢气探测可信度分配
    if(alarm_input_data[ALARM_INPUT_H2] < 10){
        bpaH2.mass[BPA_FOCAL_FIRE] = 0.0;
        bpaH2.mass[BPA_FOCAL_NOFIRE] = 0.9;
        bpaH2.mass[BPA_FOCAL_FAULT] = 0.1;
    }else if(alarm_input_data[ALARM_INPUT_H2] >= 10 && alarm_input_data[ALARM_INPUT_H2] < 100){
        bpaH2.mass[BPA_FOCAL_FIRE] = 0.2;
        bpaH2.mass[BPA_FOCAL_NOFIRE] = 0.7;
        bpaH2.mass[BPA_FOCAL_FAULT] = 0.1;
    }else if(alarm_input_data[ALARM_INPUT_H2] >= 100 && alarm_input_data[ALARM_INPUT_H2] < 200){
        bpaH2.mass[BPA_FOCAL_FIRE] = 0.5;
        bpaH2.mass[BPA_FOCAL_NOFIRE] = 0.4;
        bpaH2.mass[BPA_FOCAL_FAULT] = 0.1;
    }else if(alarm_input_data[ALARM_INPUT_H2] >= 200 && alarm_input_data[ALARM_INPUT_H2] < 250){
        bpaH2.mass[BPA_FOCAL_FIRE] = 0.7;
        bpaH2.mass[BPA_FOCAL_NOFIRE] = 0.2;
        bpaH2.mass[BPA_FOCAL_FAULT] = 0.1;
    }else{
        bpaH2.mass[BPA_FOCAL_FIRE] = 0.8;
        bpaH2.mass[BPA_FOCAL_NOFIRE] = 0.1;
        bpaH2.mass[BPA_FOCAL_FAULT] = 0.1;
    }
    //一氧化碳探测可信度分配
    if(alarm_input_data[ALARM_INPUT_CO] < 10){
        bpaCO.mass[BPA_FOCAL_FIRE] = 0.0;
        bpaCO.mass[BPA_FOCAL_NOFIRE] = 0.9;
        bpaCO.mass[BPA_FOCAL_FAULT] = 0.1;
    }else if(alarm_input_data[ALARM_INPUT_CO] >= 10 && alarm_input_data[ALARM_INPUT_CO] < 100){
        bpaCO.mass[BPA_FOCAL_FIRE] = 0.2;
        bpaCO.mass[BPA_FOCAL_NOFIRE] = 0.7;
        bpaCO.mass[BPA_FOCAL_FAULT] = 0.1;
    }else if(alarm_input_data[ALARM_INPUT_CO] >= 100 && alarm_input_data[ALARM_INPUT_CO] < 200){
        bpaCO.mass[BPA_FOCAL_FIRE] = 0.3;
        bpaCO.mass[BPA_FOCAL_NOFIRE] = 0.6;
        bpaCO.mass[BPA_FOCAL_FAULT] = 0.1;
    }else if(alarm_input_data[ALARM_INPUT_CO] >= 200 && alarm_input_data[ALARM_INPUT_CO] < 400){
        bpaCO.mass[BPA_FOCAL_FIRE] = 0.4;
        bpaCO.mass[BPA_FOCAL_NOFIRE] = 0.5;
        bpaCO.mass[BPA_FOCAL_FAULT] = 0.1;
    }else if(alarm_input_data[ALARM_INPUT_CO] >= 400 && alarm_input_data[ALARM_INPUT_CO] < 800){
        bpaCO.mass[BPA_FOCAL_FIRE] = 0.7;
        bpaCO.mass[BPA_FOCAL_NOFIRE] = 0.2;
        bpaCO.mass[BPA_FOCAL_FAULT] = 0.1;
    }else if(alarm_input_data[ALARM_INPUT_CO] >= 800 && alarm_input_data[ALARM_INPUT_CO] < 1000){
        bpaCO.mass[BPA_FOCAL_FIRE] = 0.8;
        bpaCO.mass[BPA_FOCAL_NOFIRE] = 0.1;
        bpaCO.mass[BPA_FOCAL_FAULT] = 0.1;
    }else{
        bpaCO.mass[BPA_FOCAL_FIRE] = 0.9;
        bpaCO.mass[BPA_FOCAL_NOFIRE] = 0.1;
        bpaCO.mass[BPA_FOCAL_FAULT] = 0.0;
    }

    //烟感探测可信度分配
    if(alarm_input_data[ALARM_INPUT_SMK] == 0){
        bpaSMK.mass[BPA_FOCAL_FIRE] = 0.0;
        bpaSMK.mass[BPA_FOCAL_NOFIRE] = 0.7;
        bpaSMK.mass[BPA_FOCAL_FAULT] = 0.3;
    }else{
        bpaSMK.mass[BPA_FOCAL_FIRE] = 0.7;
        bpaSMK.mass[BPA_FOCAL_NOFIRE] = 0.0;
        bpaSMK.mass[BPA_FOCAL_FAULT] = 0.0;
    }

    //温度探测可信度分配
    if(alarm_input_data[ALARM_INPUT_TEMP] < 60){
        bpaTEMP.mass[BPA_FOCAL_FIRE] = 0.0;
        bpaTEMP.mass[BPA_FOCAL_NOFIRE] = 0.9;
        bpaTEMP.mass[BPA_FOCAL_FAULT] = 0.1;
    }else if(alarm_input_data[ALARM_INPUT_TEMP] >= 60 && alarm_input_data[ALARM_INPUT_TEMP] < 80){
        bpaTEMP.mass[BPA_FOCAL_FIRE] = 0.2;
        bpaTEMP.mass[BPA_FOCAL_NOFIRE] = 0.7;
        bpaTEMP.mass[BPA_FOCAL_FAULT] = 0.1;
    }else if(alarm_input_data[ALARM_INPUT_TEMP] >= 80 && alarm_input_data[ALARM_INPUT_TEMP] < 100){
        bpaTEMP.mass[BPA_FOCAL_FIRE] = 0.5;
        bpaTEMP.mass[BPA_FOCAL_NOFIRE] = 0.4;
        bpaTEMP.mass[BPA_FOCAL_FAULT] = 0.1;
    }else if(alarm_input_data[ALARM_INPUT_TEMP] >= 100 && alarm_input_data[ALARM_INPUT_TEMP] < 120){
        bpaTEMP.mass[BPA_FOCAL_FIRE] = 0.8;
        bpaTEMP.mass[BPA_FOCAL_NOFIRE] = 0.1;
        bpaTEMP.mass[BPA_FOCAL_FAULT] = 0.1;
    }else{
        bpaTEMP.mass[BPA_FOCAL_FIRE] = 0.9;
        bpaTEMP.mass[BPA_FOCAL_NOFIRE] = 0.0;
        bpaTEMP.mass[BPA_FOCAL_FAULT] = 0.1;
    }

    bpa_combination_compute(&bpaH2,&bpaCO,&bpaResult);
    bpa_combination_compute(&bpaTEMP,&bpaResult,&bpaResult);
    double conflict = bpa_combination_compute(&bpaSMK,&bpaResult,&bpaResult);
    if(conflict < 0.5 && bpaResult.mass[BPA_FOCAL_FIRE] > 0.8){
        alarmFireOut = ALARM_FIRE_OUT_FIRE;
    }else{
        alarmFireOut = ALARM_FIRE_OUT_WARNING;
    }

    if(alarmFireOut == ALARM_FIRE_OUT_FIRE){
        //火警
    }else{
        //预警
    }
}

void alarm_informatin_processing(void){
    uint16_t alarm = 0;
    if(alarm_input_data[ALARM_INPUT_SMK] == 1){     //烟雾报警信息处理
        alarm |= ALARM_SMK;
    }else{;}

    if(alarm_input_data[ALARM_INPUT_H2] > devCfg.h2Threshold){
        alarm |= ALARM_H2;
    }else{;}

    if(alarm_input_data[ALARM_INPUT_CO] > devCfg.coThreshold){     //一氧化碳报警信息处理
        alarm |= ALARM_CO;
    }else{;}

    if(alarm_input_data[ALARM_INPUT_TEMP] > devCfg.tempThreshold){     //温度报警信息处理
        alarm |= ALARM_OVERTEMP;
    }else{;}

    if(alarm_input_data[ALARM_INPUT_VOLTAGE] > devCfg.voltageHighThreshold){     //过压报警信息处理
        alarm |= ALARM_OVERVOLTAGE;
    }else if(alarm_input_data[ALARM_INPUT_VOLTAGE] < devCfg.voltageLowThreshold){     //欠压报警信息处理
        alarm |= ALARM_UNDERVOLTAGE;
    }else{;}


    PRJ_ALARM_LOGI("烟雾:%f ,氢气:%f ,一氧化碳:%f ,温度:%f ,电压:%f ,电流:%f ",
            alarm_input_data[ALARM_INPUT_SMK],alarm_input_data[ALARM_INPUT_H2],alarm_input_data[ALARM_INPUT_CO],
            alarm_input_data[ALARM_INPUT_TEMP],alarm_input_data[ALARM_INPUT_VOLTAGE],alarm_input_data[ALARM_INPUT_CURRENT]);

    modbus_reg_write(ALARM_RESULT_REG,&alarm,1);
}


void alarm_task_handler(void *arg){
    while(1){
        alarm_data_get();
        alarm_fire_detection();
        alarm_informatin_processing();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void alarm_init(void){
    for(int i = 0;i < ALARM_INPUT_NUM;i++){
        alarm_input_data[i] = 0;
    }
    alarm_bpa_init();

    //创建报警处理任务
    xTaskCreatePinnedToCore(alarm_task_handler,"alarmTask",1024*5,NULL,6,NULL,0);
}





