#include "config.h"
#include "cJSON.h"
#include "esp_littlefs.h"

static char *TAG = "PRJ_config";
char *file_config = "/littlefs/config.json";

#define MODBUS_SERIAL_REG     0x0007
#define MODBUS_DEVICE_ID_REG    0x0000

uint8_t *data_fifo = NULL;
uint32_t load_size = 0;
bool  finishFlag = false;
uint32_t upgrade_size = 0;
uint16_t upgrade_crc = 0;


uint8_t device_get_modbus_id_form_data(uint8_t *data,char *serial_str)
{
    uint8_t modbus_id = 0;
    cJSON *root = cJSON_Parse((char *)data);
    if(root){
        cJSON *device = cJSON_GetObjectItemCaseSensitive(root, "devices");
        if(device && cJSON_IsArray(device)){
            for(int i = 0; i < cJSON_GetArraySize(device); i++){
                cJSON *item = cJSON_GetArrayItem(device, i);
                if (item && cJSON_IsObject(item)){
                    cJSON *serial = cJSON_GetObjectItemCaseSensitive(item, "devid");
                    ESP_LOGE(TAG, "Get device id form config: %s", serial->valuestring);
                    if(serial && cJSON_IsString(serial)){
                        if(0 == strcmp(serial->valuestring,serial_str)){                                    //比较设备序列号，获取设备ID
                            modbus_id = cJSON_GetObjectItemCaseSensitive(item, "adr")->valueint;
                            ESP_LOGE(TAG, "Get device id form config: %d", modbus_id);
                            break;
                        }else{
                            continue;
                        }
                    }else{
                        ESP_LOGE(TAG, "Failed to parse JSON serial");
                    }
                }else{
                    ESP_LOGE(TAG, "Failed to parse JSON item is not object");
                }     
            }   
        }else{
            ESP_LOGE(TAG, "Failed to parse JSON device");
        }

        cJSON_Delete(root);                                                                             // 释放JSON对象
    }else{
        ESP_LOGE(TAG, "Failed to parse JSON root");
    }

    return modbus_id;
}


uint8_t device_get_modbus_id_form_configJSON(char *serial_str)
{
    uint8_t modbus_id = 0;
    ESP_LOGI(TAG, "Read configJSON file");

    FILE* file = fopen(file_config, "r");
    if(file == NULL){
        ESP_LOGE(TAG, "Failed to open file for reading");
        goto OPENFILE_ERR;
    }
    fseek(file, 0L, SEEK_END);                                      // 将文件指针移动到文件末尾
    size_t size = ftell(file);                                      // 获取文件大小
    rewind(file);                                                   // 将文件指针重新定位到文件开头 
    uint8_t *data = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if(data == NULL){
        ESP_LOGE(TAG, "Failed to allocate memory");
        goto MALLOC_ERR;
    }
    memset(data, 0, size);
    size_t read_num = fread(data, 1, size, file);
    if(read_num != size){
        ESP_LOGE(TAG, "Failed to read file");
        goto READFILE_ERR;
    }

    modbus_id = device_get_modbus_id_form_data(data,serial_str);

READFILE_ERR:
    heap_caps_free(data);
MALLOC_ERR:
    fclose(file);
OPENFILE_ERR:
    return modbus_id;
}

void configJSON_upgrade(void)
{
    if(data_fifo == NULL)       return;
    FILE *file = fopen(file_config, "w");
    if(file == NULL){
        ESP_LOGE(TAG, "Failed to open file to save bin");
        return;
    }

    size_t size = fwrite(data_fifo, 1, upgrade_size, file);
	ESP_LOGI(TAG, "iap write size:%d",size);

    fclose(file);
}


void configJSON_upgrade_task_handler(void *pvParameters){
    while (1)
    {
        if(finishFlag){
            uint16_t cal_crc = 0;
            // uint16_t cal_crc = modbus_calculate_crc(data_fifo,upgrade_size);
            if(cal_crc == upgrade_crc){             //数据更新
                configJSON_upgrade();
                char serial_str[20];
                // modbus_reg_read_no_reverse(MODBUS_SERIAL_REG,serial_str,sizeof(serial_str)/2);
                uint16_t modbus_id = device_get_modbus_id_form_data(data_fifo,serial_str);
            }
            heap_caps_free(data_fifo);
            data_fifo = NULL;
        }
        vTaskDelay(1000);
    }
    
}


void config_msg_deal_handler(uint8_t *data,uint16_t length)
{
	if(data[2] != 0xff)  return;
	uint16_t data_num = data[6] | data[7]<<8;
	if(data[3] == 0x00){
		upgrade_size = (data[8]<<0) | (data[9]<<8) | (data[10]<<16) |(data[11]<<24) ;
        data_fifo = heap_caps_malloc(upgrade_size, MALLOC_CAP_SPIRAM);
        load_size = 0;
        finishFlag = false;
		return;
	}else if(data[3] == 0x11){
		finishFlag = true;
		upgrade_crc = (data[8]<<0) | (data[9]<<8);
		return;
	}else if(data[3] == 0x10){
	}else{
		return;
	}	
	
	if(data_num <= 1024){
        memcpy(data_fifo+load_size,&data[8],data_num);  //将数据写入FIFO
        load_size += data_num;
	}else{;}
}











