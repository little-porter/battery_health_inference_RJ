#include "net_config.h"
#include "esp_mac.h"
#include "littlefs_ops.h"
#include "modbus.h"
#include "sysEvent.h"
#include "string.h"
#include "config.h"

static char *TAG = "net_config";

#define LITTLEFS_PATH     "/littlefs"

char *device_file = "device_config.text";

#define REG_MODBUS_ADDR     0x0000
#define REG_MAC_ADDR        0x300C
#define REG_SERIAL_ADDR     0x0007

    
device_config_t device_cfg;
char device_mac[20];                               //MAC地址 

#define DEFAULT_MODBUS_ADDR     0x01
static char *default_serial = "101-202510090001";


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
        
        if (n < 0 || (size_t)n >= remaining) { // 检查错�?或缓冲区不足
            break;
        }
        p += n;
    }
    printf("MAC: %s\n", mac_str);
    strncpy(device_mac,mac_str,sizeof(device_mac));
    modbus_reg_write_no_reverse(REG_MAC_ADDR,(uint16_t *)device_mac,(sizeof(device_mac)+1)/2);
}


void net_config_save_default(void)
{
    device_cfg.modbus_addr = DEFAULT_MODBUS_ADDR;
    strncpy(device_cfg.serial,default_serial,sizeof(device_cfg.serial));
    littlefs_ops_write_file(device_file,(const char*)&device_cfg,sizeof(device_cfg));
}

void write_device_config_info_to_modbus_reg(void)
{
     //将配置信息写入MODBUS寄存器
    modbus_reg_write(REG_MODBUS_ADDR,(uint16_t *)&device_cfg,7);                                                    //功能配置信息，数据大端在前，反转
    modbus_reg_write_no_reverse(REG_SERIAL_ADDR,(uint16_t *)device_cfg.serial,(sizeof(device_cfg.serial)+1)/2);     //设备序列号，数据小端在前，不反转
    modbus_reg_write(0x0011,(uint16_t *)&device_cfg.h2_threshold,14);                                               //阈值配置信息，数据大端在前，反转 
}

void net_config_init(void)
{

    littlefs_file_data_t config_file;
    //读取设备MAC地址
    net_config_mac_read();
    // littlefs_ops_write_file(device_file,(const char*)&device_cfg,sizeof(device_cfg));
    bool ret = littlefs_ops_read_file(device_file,&config_file);
    
    if(ret == false) {
        net_config_save_default();                                  //打开文件失败，创建文件并写入默认数据
        ESP_LOGE(TAG, "read device_config.text fail");
        goto OPENFILE_ERR;
    }else{;}
                                       
    if(config_file.size != sizeof(device_cfg)){
        net_config_save_default();                                  //文件数据错误，创建文件并写入默认数据
        ESP_LOGE(TAG, "device_config.text size error");
        goto FILESIZE_ERR;
    }else{
        memcpy(&device_cfg,config_file.data,sizeof(device_cfg));    //更新设备配置数据
    }

    //通过序列号初始化modbus地址
    uint16_t modbus_id = (uint16_t)device_get_modbus_id_form_configJSON(device_cfg.serial);
    if(modbus_id == 0){
        net_config_save_default();                                  //序列号匹配错误，创建文件并写入默认数据
        ESP_LOGE(TAG, "serial is not find modbus id");
    }else{;}

    
    printf("MAC: %s\r\n", device_cfg.serial);
    ESP_LOGI(TAG, "read device config data success");
    
FILESIZE_ERR:
    heap_caps_free(config_file.data);
OPENFILE_ERR:
    write_device_config_info_to_modbus_reg();                       //将设备配置信息写入modbus寄存器    

    xTaskCreatePinnedToCore(device_config_task_handler,"devcfg_task",1024*3,NULL,6,NULL,0);                         //创建设备系统配置任务
}

void net_config_data_reverse(uint16_t *data,uint16_t num)
{
    uint8_t temp = 0;
    uint8_t *pData = (uint8_t *)data;
    for(int i=0;i<num/2;i++){
        temp = pData[2*i];
        pData[2*i] = pData[2*i+1];
        pData[2*i+1] = temp;
    }
}

void net_config_save(void)
{ 
    char config_data[200] = {0};
    char *pSerial = &config_data[14];
    modbus_reg_read(REG_MODBUS_ADDR,(uint16_t *)config_data,(sizeof(device_cfg)+1)/2);

    net_config_data_reverse((uint16_t *)pSerial,sizeof(device_cfg.serial));

    if(0 != memcmp(pSerial,device_cfg.serial,20)){
        uint16_t modbus_id = (uint16_t)device_get_modbus_id_form_configJSON(pSerial);               //通过序列号初始化modbus地址
        if(modbus_id != 0){                                                                         //序列号匹配成功,更改modbus地址
            device_cfg.modbus_addr = modbus_id;
            memcpy(config_data,&modbus_id,2);
            modbus_reg_write(REG_MODBUS_ADDR,(uint16_t *)&modbus_id,1);    
        }
    }

    if(0 != memcmp(&device_cfg,config_data,48)){
        memcpy(&device_cfg,config_data,48);
        littlefs_ops_write_file(device_file,(const char*)&device_cfg,sizeof(device_cfg));
        ESP_LOGI(TAG, "reset config success, mac:%s, serial:%s",device_mac,device_cfg.serial);
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
    ESP_LOGI(TAG, "氢气:%d ,一氧化�?:%d",(int)h2,(int)co);
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





