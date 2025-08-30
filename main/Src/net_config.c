#include "net_config.h"
#include "esp_mac.h"
#include "littlefs_ops.h"
#include "cJSON.h"
#include "modbus.h"
#include "sysEvent.h"

static char *TAG = "net_config";

#define LITTLEFS_PATH     "/littlefs"

char *file = "/littlefs/NetConfig.json";
char *device_file = "device_config.text";

#define REG_MODBUS_ADDR     0x0000
#define REG_MAC_ADDR        0x300C
#define REG_SERIAL_ADDR     0x0007

typedef struct _device_config_t
{
    uint16_t modbus_addr;  
    uint16_t device_type;                       //设备类型
    uint16_t device_addr;                       //modbus地址
    uint16_t cap;                               //电池额定容量  
    uint16_t reset_flag;                        //升级标志
    uint16_t low_power_flag;                    //设备地址  
    uint16_t balance_falg;
    char serial[20];                            //序列号
    uint16_t h2_threshold;                      //H2报警阈值
    uint16_t co_threshold;                      //CO报警阈值
    uint16_t temp_threshold;                    //温度报警阈值
    uint16_t temp_up_threshold;                 //温度升报警阈值
    uint16_t voltage_high_threshold;            //电压高报警阈值
    uint16_t voltage_low_threshold;             //电压低报警阈值
    int16_t current_discharge_threshold;        //电流放电报警阈值
    uint16_t current_charge_threshold;          //电流充电报警阈值
    char mac[20];                               //MAC地址 
}device_config_t;

device_config_t device_cfg;

void device_config_task_handler(void *pvParameters);
void net_config_save(void);
void net_config_mac_read(void)
{ 
    uint8_t mac[8] = {0};
    char mac_str[20] = {0};
    char *p = mac_str;
    size_t remaining = sizeof(mac_str);
    esp_efuse_mac_get_default(mac);
    // printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("MAC: %s\n", mac);
    for(int i = 0; i < 6; i++){
        int n = 0;
        if(i == 5){
            n = snprintf(p,remaining,"%02x",mac[i]);
        }else{
            n = snprintf(p,remaining,"%02x-",mac[i]);
        }
        
        if (n < 0 || (size_t)n >= remaining) { // 检查错误或缓冲区不足
            break;
        }
        p += n;
    }
    printf("MAC: %s\n", mac_str);
    strncpy(device_cfg.mac,mac_str,sizeof(device_cfg.mac));
}



void net_config_init(void)
{
    xTaskCreatePinnedToCore(device_config_task_handler,"devcfg_task",1024*3,NULL,6,NULL,0);     //创建设备系统配置任务

    littlefs_file_data_t config_file;
    net_config_mac_read();
    // littlefs_ops_write_file(device_file,(const char*)&device_cfg,sizeof(device_cfg));
    bool ret = littlefs_ops_read_file(device_file,&config_file);
    if(ret == false) {
        net_config_mac_read();
        littlefs_ops_write_file(device_file,(const char*)&device_cfg,sizeof(device_cfg));
        ESP_LOGI(TAG, "read device config fail");
        return;
    }
    if(config_file.size == sizeof(device_cfg)){
        memcpy(&device_cfg,config_file.data,sizeof(device_cfg));
        printf("MAC: %s\n", device_cfg.mac);
        modbus_reg_write_no_reverse(REG_MAC_ADDR,(uint16_t *)device_cfg.mac,(sizeof(device_cfg.mac)+1)/2);
        modbus_reg_write_no_reverse(REG_SERIAL_ADDR,(uint16_t *)device_cfg.serial,(sizeof(device_cfg.serial)+1)/2);
        ESP_LOGI(TAG, "read device config data success");
    }else{
        modbus_reg_read_no_reverse(REG_MODBUS_ADDR,(uint16_t *)&device_cfg,(sizeof(device_cfg)+1)/2);
        modbus_reg_write_no_reverse(REG_MAC_ADDR,(uint16_t *)device_cfg.mac,(sizeof(device_cfg.mac)+1)/2);
        littlefs_ops_write_file(device_file,(const char*)&device_cfg,sizeof(device_cfg));
        ESP_LOGI(TAG, "read device config data error");
    }
    
    
    heap_caps_free(config_file.data);
}

void net_config_save(void)
{ 
    char mac[18] = {0};
    char serial[18] = {0};
    uint32_t event = 0;
    sysEvent_get(sys_cfg_event_group,&event);
    if(event&SYS_CFG_SAVE_EVENT_BIT){
        // modbus_reg_read_no_reverse(REG_MAC_ADDR,(uint16_t*)device_cfg.mac,(sizeof(device_cfg.mac)+1)/2);
        // modbus_reg_read_no_reverse(REG_SERIAL_ADDR,(uint16_t *)device_cfg.serial,(sizeof(device_cfg.serial)+1)/2);
        modbus_reg_read(REG_MODBUS_ADDR,(uint16_t *)&device_cfg,(sizeof(device_cfg)+1)/2);
        littlefs_ops_write_file(device_file,(const char*)&device_cfg,sizeof(device_cfg));
        ESP_LOGI(TAG, "reset config success, mac:%s, serial:%s",mac,serial);
        sysEvent_clear(sys_cfg_event_group,SYS_CFG_SAVE_EVENT_BIT);
    }
}

#define ALARM_UNDERVOLTAGE  0x0001
#define ALARM_OVERVOLTAGE   0x0002
#define ALARM_OVERCHARGE    0x0004
#define ALARM_OVERDISCHARGE 0x0008
#define ALARM_OVERTEMP      0x0010
#define ALARM_SMK           0x0020
#define ALARM_H2            0x0040
#define ALARM_CO            0x0080

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
    ESP_LOGI(TAG, "氢气:%d ,一氧化碳:%d",(int)h2,(int)co);
    modbus_reg_write(0x200C,&alarm,1);
}

void device_config_task_handler(void *pvParameters)
{
    while(1){
        net_config_save();
        device_alarm_handler();
        vTaskDelay(pdMS_TO_TICKS(1000));
    } 
}





