#include "devConfig.h"
#include "esp_mac.h"
#include "modbus.h"
#include "configJson.h"


static char *TAG = "PRJ_devConfig";

static char *devConfigFlie = "/littlefs/devConfig.text";

#define DEFAULT_MODBUS_ADDR     0x01
static char *defaultSerial = "101-xxxxxxxxxx";

#define DEVCONFIG_START_REG         0x0000
#define DEVCONFIG_MAC_REG           0x300C
#define DEVCONFIG_SERIAL_REG        0x0007

#define DEVCONFIG_TIME_STAMP_REG    0x0019

devConfig_t devCfg;
static char devMac[20] = {0};

void devConfig_mac_read(void)
{ 
    uint8_t mac[8] = {0};
    char mac_str[20] = {0};
    char *p = mac_str;
    size_t remaining = sizeof(mac_str);
    esp_efuse_mac_get_default(mac);
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
    strncpy(devMac,mac_str,sizeof(devMac));
    modbus_reg_write_no_reverse(DEVCONFIG_MAC_REG,(uint16_t *)devMac,(sizeof(devMac)+1)/2);
}


void devConfig_default_init(void){
    devCfg.modbusAddr = DEFAULT_MODBUS_ADDR;
    memset(&devCfg.unixTime,0,sizeof(devCfg.unixTime));
    strncpy(devCfg.serial,defaultSerial,sizeof(devCfg.serial));
}


void devConfig_save(void){
    FILE* file = fopen(devConfigFlie, "w");
    if(file == NULL){
        ESP_LOGE(TAG, "Failed to open devConfigFlie for writing!");
        return;
    }else{;}

    size_t write_num = fwrite(&devCfg, 1, sizeof(devCfg), file);
    if(write_num != sizeof(devCfg)){
        ESP_LOGE(TAG, "Failed to write devConfig file!");
        fclose(file);
        return;
    }else{;}

    fclose(file);
}


bool devConfig_file_read(void){
    bool ret = false;
    FILE* file = fopen(devConfigFlie, "rb");
    if(file == NULL){
        ESP_LOGE(TAG, "Failed to open devConfigFlie!");
        return ret;
    }else{;}

    fseek(file, 0L, SEEK_END);          //point to file end
    size_t size = ftell(file);
    rewind(file);
    if(size != sizeof(devConfig_t)){
        ESP_LOGE(TAG, "devConfig file size error!");
        fclose(file);
        return ret;
    }else{;}

	uint8_t *data = heap_caps_malloc(size, MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    size_t read_num = fread(data, 1, size, file);
    if(read_num != size){
        ESP_LOGE(TAG, "Failed to read devConfig file!");
        heap_caps_free(data);
        fclose(file);
        return ret;
    }else{;}

    memcpy(&devCfg,data,sizeof(devCfg));    //更新设备配置数据
    heap_caps_free(data);
    fclose(file);
    ret = true;
    return ret;
}

void devConfig_modbus_id_get_from_configJson(void){
    //判断序列号
    if(devCfg.serial[0] == '\0'){
        ESP_LOGE(TAG, "devCfg.serial is empty!");
        return;
    }

    //通过序列号初始化modbus地址
    uint16_t modbus_id = (uint16_t)configJson_get_modbusAddr_by_serial(devCfg.serial);

    if(modbus_id != devCfg.modbusAddr){
        devCfg.modbusAddr = modbus_id;
        devConfig_save();                                  
    }else{;}

}

void devConfig_write_to_modbus_reg(void){
    //将配置信息写入MODBUS寄存器
    modbus_reg_write(DEVCONFIG_START_REG,(uint16_t *)&devCfg,7);                                                    //功能配置信息，数据大端在前，反转
    modbus_reg_write_no_reverse(DEVCONFIG_SERIAL_REG,(uint16_t *)devCfg.serial,(sizeof(devCfg.serial)+1)/2);     //设备序列号，数据小端在前，不反转
    modbus_reg_write(0x0011,(uint16_t *)&devCfg.h2Threshold,14);                                               //阈值配置信息，数据大端在前，反转 
}

void devConfig_data_reverse(uint16_t *data,uint16_t num){
    uint8_t temp = 0;
    uint8_t *pData = (uint8_t *)data;
    for(int i=0;i<num/2;i++){
        temp = pData[2*i];
        pData[2*i] = pData[2*i+1];
        pData[2*i+1] = temp;
    }
}



void devConfig_check(void)
{ 
    char config_data[200] = {0};
    char *pSerial = &config_data[14];
    modbus_reg_read(DEVCONFIG_START_REG,(uint16_t *)config_data,(sizeof(devCfg)+1)/2);

    devConfig_data_reverse((uint16_t *)pSerial,sizeof(devCfg.serial));

    if(0 != memcmp(pSerial,devCfg.serial,20)){      //序列号改变                                 
        uint16_t modbus_id = (uint16_t)configJson_get_modbusAddr_by_serial(pSerial);                //通过序列号初始化modbus地址
        if(modbus_id != 0){                                                                         //序列号匹配成功,更改modbus地址
            devCfg.modbusAddr = modbus_id;
            memcpy(config_data,&modbus_id,2);
            modbus_reg_write(DEVCONFIG_START_REG,(uint16_t *)&modbus_id,1);    
        }
    }

    if(0 != memcmp(&devCfg,config_data,48)){
        memcpy(&devCfg,config_data,48);
        devConfig_save();
        ESP_LOGI(TAG, "reset config success, mac:%s, serial:%s",devMac,devCfg.serial);
    }
}



void devConfig_monitor_task_handler(void *parameters){
    while (1){
        devConfig_check();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}


uint64_t byte_reverse_uint64(uint64_t value) {
    uint64_t result = 0;
    
    result |= (value & 0x00000000000000FFULL) << 56;
    result |= (value & 0x000000000000FF00ULL) << 40;
    result |= (value & 0x0000000000FF0000ULL) << 24;
    result |= (value & 0x00000000FF000000ULL) << 8;
    result |= (value & 0x000000FF00000000ULL) >> 8;
    result |= (value & 0x0000FF0000000000ULL) >> 24;
    result |= (value & 0x00FF000000000000ULL) >> 40;
    result |= (value & 0xFF00000000000000ULL) >> 56;
    
    return result;
}

void devConfig_systemTimeStamp_task_handler(void *parameters){
    while (1){
        uint64_t time = 0,tempTime = 0;;
        modbus_reg_read_no_reverse(DEVCONFIG_TIME_STAMP_REG,(uint16_t *)&time,sizeof(time)/2);   
        // uint8_t *pTime = (uint8_t *)&time;
        // for(int i=0;i<8;i++){
        //     tempTime |= (pTime[i] << (8*i));
        // }
        // tempTime++;
        // pTime = (uint8_t *)&tempTime;
        // for(int i=0;i<8;i++){
        //     time |= (pTime[i] << (8*i));
        // }
        time = byte_reverse_uint64(time);
        time++;
        time = byte_reverse_uint64(time);
        modbus_reg_write_no_reverse(DEVCONFIG_TIME_STAMP_REG,(uint16_t *)&time,sizeof(time)/2);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void devConfig_init(void){
    //读取设备MAC地址
    devConfig_mac_read();

    //读取设备配置文件
    if(devConfig_file_read() == false){
        devConfig_default_init();
        devConfig_save();
    }
    
    //获取modbusAddress
    devConfig_modbus_id_get_from_configJson();
    
    //将设备配置信息写入modbus寄存器    
    devConfig_write_to_modbus_reg();                       

    //create config monitor task
    xTaskCreatePinnedToCore(devConfig_monitor_task_handler,"devCfgTask",1024*3,NULL,6,NULL,0);                         //创建设备系统配置任务

    //create system time stamp task
    xTaskCreatePinnedToCore(devConfig_systemTimeStamp_task_handler,"timeStampTask",1024*3,NULL,9,NULL,0);
}






