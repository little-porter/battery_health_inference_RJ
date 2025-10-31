#include "configJson.h"
#include "modbus.h"
#include "cJSON.h"

static char* TAG = "PRJ_CONFIGJSON";
#define PRJ_CONFIGJSON_LOG_ENABLE      1                     //soh log enable
#if PRJ_CONFIGJSON_LOG_ENABLE
#define PRJ_CONFIGJSON_PRINTF(x,...)           printf(x,##__VA_ARGS__)
#define PRJ_CONFIGJSON_LOGI(format, ...)       ESP_LOGI(TAG,format, ##__VA_ARGS__)
#define PRJ_CONFIGJSON_LOGW(format, ...)       ESP_LOGW(TAG,format, ##__VA_ARGS__)
#else
#define PRJ_CONFIGJSON_PRINTF(x,...)          
#define PRJ_CONFIGJSON_LOGI(format, ...)       
#define PRJ_CONFIGJSON_LOGW(format, ...)       
#endif

#define PRJ_CONFIGJSON_LOGE(format, ...)       ESP_LOGE(TAG,format, ##__VA_ARGS__)



static char* file_configJson = "/littlefs/config.json";
static char* file_configJson_cache = "/littlefs/configCache.json";

#define CONFIGJSON_MODBUS_ID_REG    0x0000
#define CONFIGJSON_SERIAL_REG       0x0007


#define CONFIGJSON_FIFO_SIZE		1024*4
#define CONFIGJSON_MAX_SIZE		    0xA000          //40kb
#define CONFIGJSON_MAX_TIMEOUT		(10)

#define configJson_crc_calculate(a,b)	modbus_calculate_crc(a,b)

typedef struct configJson_fifo{
    uint8_t data[CONFIGJSON_FIFO_SIZE];
    uint16_t pos;
    uint16_t tail;
}configJson_fifo_t;

typedef enum _configJson_update_flag{
    CONFIGJSON_UPDATE_FLAG_IDLE = 0,
    CONFIGJSON_UPDATE_FLAG_START,
    CONFIGJSON_UPDATE_FLAG_FINISH,
}configJson_update_flag_t;

typedef enum _configJson_updtate_status{
    CONFIGJSON_UPDATE_STATUS_FINISH = 0,
    CONFIGJSON_UPDATE_STATUS_UPGRADING = 1,
    CONFIGJSON_UPDATE_STATUS_BIN_OVER,
    // CONFIGJSON_UPDATE_STATUS_BIN_ERROR,
    CONFIGJSON_UPDATE_STATUS_CRC_ERROR,
    CONFIGJSON_UPDATE_STATUS_TIMEOUT,
}configJson_update_status_t;

typedef enum _configJson_process{
    CONFIGJSON_PROCESS_IDLE = 0,
    CONFIGJSON_PROCESS_UPGRADE,
    CONFIGJSON_PROCESS_CRC,
    CONFIGJSON_PROCESS_FINISH,
    CONFIGJSON_PROCESS_ERROR,
}configJson_process_t;

typedef struct configJson_info{
    uint32_t update_size;
    uint16_t update_crc;
    uint32_t bin_size;
    uint16_t bin_crc;

    configJson_update_flag_t    flag;
    configJson_update_status_t  status;
    configJson_process_t        process;

    FILE *cacheFile;
    TimerHandle_t timeoutTimer;
    uint16_t timeout;
}configJson_info_t;

configJson_fifo_t configJson_fifo;
configJson_info_t configJson_info;


uint8_t configJson_get_modbusAddr_from_jsonData_by_serial(uint8_t *data,char *serial_str)
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
                    PRJ_CONFIGJSON_LOGI("Get device id form config: %s", serial->valuestring);
                    if(serial && cJSON_IsString(serial)){
                        if(0 == strcmp(serial->valuestring,serial_str)){                                    //比较设备序列号，获取设备ID
                            modbus_id = atoi(cJSON_GetObjectItemCaseSensitive(item, "adr")->valuestring);
                            PRJ_CONFIGJSON_LOGW("Get device id form config: %d", modbus_id);
                            break;
                        }else{
                            continue;
                        }
                    }else{
                        PRJ_CONFIGJSON_LOGE("Failed to parse JSON serial");
                    }
                }else{
                    PRJ_CONFIGJSON_LOGE("Failed to parse JSON item is not object");
                }     
            }   
        }else{
            PRJ_CONFIGJSON_LOGE("Failed to parse JSON device");
        }

        cJSON_Delete(root);                                                                             // 释放JSON对象
    }else{
        PRJ_CONFIGJSON_LOGE("Failed to parse JSON root");
    }

    return modbus_id;
}

uint16_t configJson_get_modbusAddr_by_serial(char *serial_str)
{
    uint8_t modbus_id = 0;
    PRJ_CONFIGJSON_LOGI("Read configJSON file");

    FILE* file = fopen(file_configJson, "r");
    if(file == NULL){
        PRJ_CONFIGJSON_LOGE("Failed to open configJSON file!");
        return modbus_id;
    }else{;}

    fseek(file, 0L, SEEK_END);          //point to file end
    size_t size = ftell(file);
    rewind(file);

    uint8_t *data = heap_caps_malloc(size, MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    size_t read_num = fread(data, 1, size, file);
    if(read_num != size){
        PRJ_CONFIGJSON_LOGE("Failed to read configJSON file!");
        heap_caps_free(data);
        fclose(file);
        return modbus_id;
    }else{;}

    modbus_id = configJson_get_modbusAddr_from_jsonData_by_serial(data,serial_str);

    heap_caps_free(data);
    fclose(file);
    return modbus_id;
}


uint16_t configJson_read_fifo_bytes(void)
{
	uint16_t data_num;
	if(configJson_fifo.pos < configJson_fifo.tail){
		data_num = configJson_fifo.tail - configJson_fifo.pos;
	}else if(configJson_fifo.pos > configJson_fifo.tail){
		data_num = CONFIGJSON_FIFO_SIZE - configJson_fifo.pos + configJson_fifo.tail;
	}else{
		data_num = 0;
	}
	return data_num;
}

uint16_t configJson_read_data_from_fifo(uint8_t *pdata,uint16_t num,uint16_t timeout)
{
	uint16_t data_num;
	data_num = configJson_read_fifo_bytes();
	
	if(data_num > num){
		data_num = num;
	}else{
		data_num = data_num;
	}
	
	if((configJson_fifo.pos+data_num) < CONFIGJSON_FIFO_SIZE){
		memcpy(pdata,&configJson_fifo.data[configJson_fifo.pos],data_num);
	}else{
		memcpy(pdata,&configJson_fifo.data[configJson_fifo.pos],CONFIGJSON_FIFO_SIZE-configJson_fifo.pos);
		memcpy(pdata+CONFIGJSON_FIFO_SIZE-configJson_fifo.pos,&configJson_fifo.data[0],data_num-(CONFIGJSON_FIFO_SIZE-configJson_fifo.pos));
	}
	
	configJson_fifo.pos = configJson_fifo.pos + data_num;
	configJson_fifo.pos %= CONFIGJSON_FIFO_SIZE;
	
	return data_num;		//返回读取数量
}

void configJson_write_data_to_fifo(uint8_t *pdata,uint16_t num,uint16_t timeout)
{
	if((configJson_fifo.tail+num) < CONFIGJSON_FIFO_SIZE){
		memcpy(&configJson_fifo.data[configJson_fifo.tail],pdata,num);
	}else{
		memcpy(&configJson_fifo.data[configJson_fifo.tail],pdata,CONFIGJSON_FIFO_SIZE-configJson_fifo.tail);
		memcpy(&configJson_fifo.data[0],pdata+CONFIGJSON_FIFO_SIZE-configJson_fifo.tail,num - (CONFIGJSON_FIFO_SIZE-configJson_fifo.tail));
	}
	
	configJson_fifo.tail = configJson_fifo.tail + num;
	configJson_fifo.tail %= CONFIGJSON_FIFO_SIZE;
}

void configJson_cache_update_start(void)
{
    configJson_info.cacheFile = fopen(file_configJson_cache, "w");
    if(configJson_info.cacheFile == NULL){
        PRJ_CONFIGJSON_LOGE("Failed to open file to save configJson");
        return;
    }
}

void configJson_cache_update_data_write(uint8_t *data,uint16_t length)
{
    size_t size = fwrite(data, 1, length, configJson_info.cacheFile);
    configJson_info.bin_size += size;
	PRJ_CONFIGJSON_LOGI("configJson write size:%d",(int)configJson_info.bin_size);
}

void configJson_cache_update_end(void)
{
    if(configJson_info.cacheFile != NULL){
         fclose(configJson_info.cacheFile);
		 configJson_info.cacheFile = NULL;
    }
}

bool configJson_cache_update(void)
{
	bool status = true;
	uint8_t read_data[1024] = {0};
	uint16_t read_bytes = 0;

	read_bytes = configJson_read_data_from_fifo(read_data,1024,100);
	if((read_bytes != 0) && ((configJson_info.bin_size+read_bytes) <= CONFIGJSON_MAX_SIZE)){
        configJson_cache_update_data_write(read_data,read_bytes);
        configJson_info.timeout = CONFIGJSON_MAX_TIMEOUT;
    }else if((configJson_info.bin_size+read_bytes) > CONFIGJSON_MAX_SIZE){
		status = false;
	}else{;}

	return status;
}

bool configJson_cache_crc(void){
    bool status = true;
    FILE* file = fopen(file_configJson_cache, "rb");
    if(file == NULL){
        PRJ_CONFIGJSON_LOGE("Failed to open file for upgrade,not found bin file");
        return false;
    }
    fseek(file, 0L, SEEK_END);          //point to file end
    size_t size = ftell(file);
    rewind(file);

	uint8_t *data = heap_caps_malloc(size, MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    size_t read_num = fread(data, 1, size, file);


	configJson_info.bin_crc = configJson_crc_calculate(data,configJson_info.bin_size);
	PRJ_CONFIGJSON_LOGI("bin crc is:%04x,cal_crc is:%04x",configJson_info.update_crc,configJson_info.bin_crc);
	
	if(configJson_info.bin_crc == configJson_info.update_crc){
		configJson_info.status = CONFIGJSON_PROCESS_FINISH;
		// modbus_reg_write(REG_IAP_START_FLAG,&configJson_info.status,1);
		status = true;
	}else{
		configJson_info.status = CONFIGJSON_PROCESS_ERROR;
		status = false;
	}

	heap_caps_free(data);
	fclose(file);
	return status;
}


bool configJson_update(void){
    bool status = false;
	FILE* file_cache = fopen(file_configJson_cache, "rb");
	FILE* file_bin = fopen(file_configJson, "w");
	if(file_cache == NULL || file_bin == NULL){
		PRJ_CONFIGJSON_LOGE("Failed to open cache file bin file");
		return status;
	}

	fseek(file_cache, 0L, SEEK_END);  // point to file end
    size_t size = ftell(file_cache);
    rewind(file_cache);
	
	uint8_t *data = heap_caps_malloc(size, MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    size_t read_num = fread(data, 1, size, file_cache);
	
	fwrite(data, 1, read_num, file_bin);

	heap_caps_free(data);
	fclose(file_cache);
	fclose(file_bin);
	return status;
}

bool congfigJson_update_timeout_check(void)
{
	bool status = false;
	if(configJson_info.timeout == 0){
		configJson_info.status = CONFIGJSON_UPDATE_STATUS_TIMEOUT;
		status = true;
		return status;
	}else{
		return status;
	}
	
}

void configJson_update_modbusAddr(void){
    char serialStr[20] = {0};
    modbus_reg_read_no_reverse(CONFIGJSON_SERIAL_REG,(uint16_t *)serialStr,sizeof(serialStr)/2);
    uint16_t modbus_id = configJson_get_modbusAddr_by_serial(serialStr);
    PRJ_CONFIGJSON_LOGW("configJson update modbus id to : %d", modbus_id);
    modbus_reg_write(CONFIGJSON_MODBUS_ID_REG,&modbus_id,1);
}


void configJson_timer_callback(TimerHandle_t xTimer)
{
    if(configJson_info.timeout > 0) configJson_info.timeout--;
}

void configJSON_update_task_handler(void *pvParameters){
    while (1)
    {
        bool status = false;
        switch (configJson_info.process){
            case CONFIGJSON_PROCESS_IDLE:
                if(configJson_info.flag == CONFIGJSON_UPDATE_FLAG_START){
                    if(configJson_info.update_size > CONFIGJSON_MAX_SIZE){
                        configJson_info.status = CONFIGJSON_UPDATE_STATUS_BIN_OVER;
                        configJson_info.process = CONFIGJSON_PROCESS_ERROR;
                    }else{
                        //fifo初始化
                        configJson_fifo.pos = 0;
                        configJson_fifo.tail = 0;
                        //信息初始化
                        configJson_info.bin_size = 0;
                        configJson_info.bin_crc = 0;
                        configJson_info.process = CONFIGJSON_PROCESS_UPGRADE;
                        configJson_cache_update_start();
                        configJson_info.timeout = CONFIGJSON_MAX_TIMEOUT;
                    }
                }
                break;
            case CONFIGJSON_PROCESS_UPGRADE:
                //file cache update
                status = configJson_cache_update();                 
                if(!status){                            //return false is bin size over
                    configJson_info.status = CONFIGJSON_UPDATE_STATUS_BIN_OVER;
                    configJson_info.process = CONFIGJSON_PROCESS_ERROR;
                }
                if(configJson_info.flag == CONFIGJSON_UPDATE_FLAG_FINISH){   //finish flag
                    configJson_cache_update_end();
                    configJson_info.process = CONFIGJSON_PROCESS_CRC;
                }
                //timeout check
                status = congfigJson_update_timeout_check();
                if(status){                 //timeout
                    configJson_info.process = CONFIGJSON_PROCESS_ERROR;
                }else{;}
                break;
            case CONFIGJSON_PROCESS_CRC:
                status = configJson_cache_crc();
                if(status){
                    configJson_info.process = CONFIGJSON_PROCESS_FINISH;
                }else{
                    configJson_info.process = CONFIGJSON_PROCESS_ERROR;
                }
                break;
            case CONFIGJSON_PROCESS_FINISH:
                configJson_update();
                configJson_update_modbusAddr();
                configJson_info.process = CONFIGJSON_PROCESS_IDLE;
                break;
            case CONFIGJSON_PROCESS_ERROR:
                configJson_cache_update_end();
                configJson_info.process = CONFIGJSON_PROCESS_IDLE;
                break;
            default:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
}


void configJson_init(void){
    //info init
    configJson_info.update_size = 0;
    configJson_info.update_crc = 0;
    configJson_info.timeout = CONFIGJSON_MAX_TIMEOUT;
    configJson_info.flag = CONFIGJSON_UPDATE_FLAG_IDLE;
    configJson_info.status = CONFIGJSON_UPDATE_STATUS_FINISH;
    configJson_info.process = CONFIGJSON_PROCESS_IDLE;
    configJson_info.bin_size = 0;
    configJson_info.bin_crc = 0;
    configJson_info.cacheFile = NULL;

    //fifo init
    configJson_fifo.pos = 0;
    configJson_fifo.tail = 0;
    memset(configJson_fifo.data,0,CONFIGJSON_FIFO_SIZE);

    //create timeout check timer
    configJson_info.timeoutTimer = xTimerCreate("confJsonTimer", pdMS_TO_TICKS(1000), pdTRUE, NULL, configJson_timer_callback);
	xTimerStart(configJson_info.timeoutTimer, 0);

    //create configJson update task
    xTaskCreatePinnedToCore(configJSON_update_task_handler, "confJsonTask", 1024*5, NULL, 6, NULL, 0);
}




void config_msg_deal_handler(uint8_t *data,uint16_t length)
{
	if(data[2] != 0xff) {
        PRJ_CONFIGJSON_LOGE("config json msg type error: %02x", data[2]);
        return;
    }
	uint16_t data_num = data[6] | data[7]<<8;
	if(data[3] == 0x00){
		configJson_info.update_size = (data[8]<<0) | (data[9]<<8) | (data[10]<<16) |(data[11]<<24) ;
        configJson_info.flag = CONFIGJSON_UPDATE_FLAG_START;
        PRJ_CONFIGJSON_LOGE("config json update size: %d", (int)configJson_info.update_size);
		return;
	}else if(data[3] == 0x11){
		configJson_info.update_crc = (data[8]<<0) | (data[9]<<8);
        configJson_info.flag = CONFIGJSON_UPDATE_FLAG_FINISH;
        PRJ_CONFIGJSON_LOGE("config json update crc: %04x", (int)configJson_info.update_crc);
		return;
	}else if(data[3] == 0x10){
	}else{
		return;
	}	
	
    PRJ_CONFIGJSON_LOGE("config json data num: %d", data_num);
	if(data_num <= 1024){
        configJson_write_data_to_fifo(&data[8],data_num,500);  //将数据写入FIFO
        configJson_info.flag = CONFIGJSON_UPDATE_FLAG_IDLE;
	}else{;}
}





