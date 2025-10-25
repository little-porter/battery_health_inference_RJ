#include "iap.h"
#include "string.h"
#include "modbus.h"

static const char *TAG = "PRJ_IAP";

static const char *collect_bin_file = "/littlefs/battery_info_collect.bin";
static const char *collect_cache_bin_file = "/littlefs/battery_info_collect_cache.bin";
const char *app_name = "batteryInfoCollectSoftware";

#define FACTORY_APP		0
#define IAP_APP			1
#define APP_TYPE		FACTORY_APP
#define APP_MAX_SIZE			0xA000				//40kb
#define REG_IAP_START_FLAG 		0x200F
#define REG_IAP_VERSION_YEAR 	0x3000
#define REG_IAP_VERSION_AA 		0x3003
#define REG_IAP_VERSION_BB 		0x3004
#define REG_IAP_VERSION_CC 		0x3005
#define IAP_OUT_TIME_MS	   		10000

#define IAP_INFO_CRC			0xAA55


typedef void (*pAppFunction)(void);

#define IAP_FIFO_SIZE		4096
typedef struct _iap_fifo
{
	uint8_t data[IAP_FIFO_SIZE];
	uint16_t pos;
	uint16_t tail;
}iap_fifo_t;

iap_fifo_t iap_fifo;


typedef enum _iap_process
{
	IAP_PROCESS_IDLE = 0,
	IAP_PROCESS_HEAD_CHECK,
	IAP_PROCESS_UPGRADE,
	IAP_PROCESS_FINISH,
	IAP_PROCESS_ERROR,
}iap_process_t;

typedef enum _iap_flag
{
	IAP_FLAG_IDLE = 0,
	IAP_FLAG_START = 1,
	IAP_FLAG_FINISH = 2,
}iap_falg_t;

typedef enum _iap_status{ 
	IAP_STATUS_UPGRADING = 0,
	IAP_STATUS_FINISH,
	IAP_STATUS_BIN_OVER,
	IAP_STATUS_BIN_ERROR,
	IAP_STATUS_VERSION_ERROR,
	IAP_STATUS_CRC_ERROR,
	IAP_STATUS_TIMEOUT,
}iap_status_t;


typedef enum _iap_upgrade_falg
{
	IAP_UPGRADE_FLAG_NO = 0,
	IAP_UPGRADE_FLAG_YES,
}iap_upgrade_flag_t;


typedef struct _version
{
    uint16_t year,month,day;
    uint16_t aa,bb,cc;
}version_t;
typedef struct _app_info
{
	version_t version;
	char name[100];
	char describe[100];
}app_info_t;

typedef struct _iap_info
{
	version_t run_app_version;
	version_t cache_version;
    FILE* file;
	
	iap_process_t process;
	uint32_t upgrade_size;
	uint16_t upgrade_crc;
	iap_falg_t    iap_flag;
	uint16_t  iap_status;
	
	uint32_t bin_size;
	uint16_t bin_crc;
	uint16_t info_crc;
}iap_info_t;

iap_info_t iap_info;

#define  IAP_TIMEOUT		10
uint16_t   iap_delay_time = IAP_TIMEOUT;					//IAP���̳�ʱʱ��
uint8_t    app_is_executable = 0;

bool iapBinFileUpdate = true;

#define IAP_INFO_OFFSET		0x300

void iap_task_handler(void *param);

void iap_get_bin_version(version_t *bin_version)
{
    FILE* file = fopen(collect_bin_file, "rb");
    if(file == NULL){
        ESP_LOGE(TAG, "Failed to open file for upgrade,not found bin file");
        return;
    }
    uint8_t *data = heap_caps_malloc(1024,MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    size_t bytes_read = fread(data, 1, 1024, file);
    version_t *version = (version_t *)(data+IAP_INFO_OFFSET);
    memcpy(bin_version,version,sizeof(version_t));

    ESP_LOGI(TAG, "collect_device_bin_soft_version is:%d-%d-%d  %d.%d.%d",
            (int)bin_version->year,(int)bin_version->month,(int)bin_version->day,
            (int)bin_version->aa,(int)bin_version->bb,(int)bin_version->cc);

    heap_caps_free(data);
    fclose(file);
}

void iap_get_cache_bin_version(version_t *cache_version)
{
    FILE* file = fopen(collect_cache_bin_file, "rb");
    if(file == NULL){
        ESP_LOGE(TAG, "Failed to open file for upgrade,not found bin file");
        return;
    }
    uint8_t *data = heap_caps_malloc(1024,MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    size_t bytes_read = fread(data, 1, 1024, file);
    version_t *version = (version_t *)(data+IAP_INFO_OFFSET);
    memcpy(cache_version,version,sizeof(version_t));

    ESP_LOGI(TAG, "collect_device_bin_soft_version is:%d-%d-%d  %d.%d.%d",
            (int)cache_version->year,(int)cache_version->month,(int)cache_version->day,
            (int)cache_version->aa,(int)cache_version->bb,(int)cache_version->cc);

    heap_caps_free(data);
    fclose(file);
}

void iap_info_init(void)
{
	iap_get_bin_version(&iap_info.run_app_version);
	iap_get_cache_bin_version(&iap_info.cache_version);
	iap_info.process = IAP_PROCESS_IDLE;
    iap_info.upgrade_size = 0;
    iap_info.upgrade_crc = 0;
    iap_info.iap_flag = IAP_FLAG_IDLE;
    iap_info.bin_size = 0;
    iap_info.bin_crc = 0;
	iap_info.info_crc = IAP_INFO_CRC;
}

TimerHandle_t iap_timer;
void iap_out_times_dec(void)
{
	if(iap_delay_time > 0){
		iap_delay_time--;
	}
}
void iap_timer_callback(TimerHandle_t xTimer)
{
	iap_out_times_dec();
}
void iap_init(void)
{
	iap_info_init();
	xTaskCreatePinnedToCore(iap_task_handler, "iap_task", 1024*5, NULL, 6, NULL, 0);

	/*创建超时检测定时器*/
	iap_timer = xTimerCreate("iap_timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, iap_timer_callback);
	xTimerStart(iap_timer, 0);
}

void iap_start(void)
{
    iap_info.file = fopen(collect_cache_bin_file, "w");
    if(iap_info.file == NULL){
        ESP_LOGE(TAG, "Failed to open file to save bin");
        return;
    }
}

void iap_bin_write(uint8_t *data,uint16_t length)
{
    size_t size = fwrite(data, 1, length, iap_info.file);
    iap_info.bin_size += size;
	ESP_LOGI(TAG, "iap write size:%d",(int)iap_info.bin_size);
}

void iap_bin_end(void)
{
    if(iap_info.file != NULL){
         fclose(iap_info.file);
		 iap_info.file = NULL;
    }
}

void iap_cache_block_erase(void)
{
	
}

uint16_t iap_read_bytes(void)
{
	uint16_t data_num;
	if(iap_fifo.pos < iap_fifo.tail){
		data_num = iap_fifo.tail - iap_fifo.pos;
	}else if(iap_fifo.pos > iap_fifo.tail){
		data_num = IAP_FIFO_SIZE - iap_fifo.pos + iap_fifo.tail;
	}else{
		data_num = 0;
	}
	return data_num;
}

uint16_t iap_read_data(uint8_t *pdata,uint16_t num,uint16_t timeout)
{
	uint16_t data_num;
	if(iap_fifo.pos < iap_fifo.tail){
		data_num = iap_fifo.tail - iap_fifo.pos;
	}else if(iap_fifo.pos > iap_fifo.tail){
		data_num = IAP_FIFO_SIZE - iap_fifo.pos + iap_fifo.tail;
	}else{
		data_num = 0;
	}
	
	if(data_num > num){
		data_num = num;
	}else{
		data_num = data_num;
	}
	
	if((iap_fifo.pos+data_num) < IAP_FIFO_SIZE){
		memcpy(pdata,&iap_fifo.data[iap_fifo.pos],data_num);
	}else{
		memcpy(pdata,&iap_fifo.data[iap_fifo.pos],IAP_FIFO_SIZE-iap_fifo.pos);
		memcpy(pdata+IAP_FIFO_SIZE-iap_fifo.pos,&iap_fifo.data[0],data_num-(IAP_FIFO_SIZE-iap_fifo.pos));
	}
	
	iap_fifo.pos = iap_fifo.pos + data_num;
	iap_fifo.pos %= IAP_FIFO_SIZE;
	
	return data_num;		//返回读取数量
}

void iap_write_data(uint8_t *pdata,uint16_t num,uint16_t timeout)
{
	if((iap_fifo.tail+num) < IAP_FIFO_SIZE){
		memcpy(&iap_fifo.data[iap_fifo.tail],pdata,num);
	}else{
		memcpy(&iap_fifo.data[iap_fifo.tail],pdata,IAP_FIFO_SIZE-iap_fifo.tail);
		memcpy(&iap_fifo.data[0],pdata+IAP_FIFO_SIZE-iap_fifo.tail,num - (IAP_FIFO_SIZE-iap_fifo.tail));
	}
	
	iap_fifo.tail = iap_fifo.tail + num;
	iap_fifo.tail %= IAP_FIFO_SIZE;
}

bool iap_app_name_check(uint8_t *pdata,uint16_t num)
{
	app_info_t *new_app_info = (app_info_t *)(pdata + IAP_INFO_OFFSET);
	ESP_LOGI(TAG, "new_app_info->version is:%d-%d-%d  %d.%d.%d",(int)new_app_info->version.year,(int)new_app_info->version.month,(int)new_app_info->version.day,
		(int)new_app_info->version.aa,(int)new_app_info->version.bb,(int)new_app_info->version.cc);
	ESP_LOGI(TAG, "new_app_info->name len:%d",strlen(new_app_info->name));
	ESP_LOGI(TAG, "run_app_info->name len:%d",strlen(app_name));

	if(0 == strcmp(new_app_info->name,app_name)){
		return true;
	}else{
		iap_info.iap_status = IAP_STATUS_BIN_ERROR;
		return false;
	}
}
bool iap_app_version_check(uint8_t *pdata,uint16_t num)
{
	version_t *run_app_version = &iap_info.run_app_version;
	version_t *new_app_version = (version_t *)(pdata + IAP_INFO_OFFSET);
	
	ESP_LOGI(TAG, "run app version is:%d-%d-%d  %d.%d.%d",(int)run_app_version->year,(int)run_app_version->month,(int)run_app_version->day,
		(int)run_app_version->aa,(int)run_app_version->bb,(int)run_app_version->cc);
	ESP_LOGI(TAG, "new app version is:%d-%d-%d  %d.%d.%d",(int)new_app_version->year,(int)new_app_version->month,(int)new_app_version->day,
		(int)new_app_version->aa,(int)new_app_version->bb,(int)new_app_version->cc);


	if((run_app_version->aa == 0xFFFF) && (run_app_version->bb == 0xFFFF) && (run_app_version->cc == 0xFFFF)){
		return true;
	}else{;}
	
	if(0 > memcmp(run_app_version,new_app_version,6*2)){
		return true;	
	}else{
		iap_info.iap_status = IAP_STATUS_VERSION_ERROR;
		return false;
	}
	
}

bool iap_start_check(void)
{
	bool status = true;
	if(iap_info.iap_flag == IAP_FLAG_START){
		return  status;
	}else{
		status = false;
		return status;
	}
}

bool iap_bin_size_check(void)
{
	bool status = true;
	if(iap_info.upgrade_size > APP_MAX_SIZE){
		iap_info.iap_status = IAP_STATUS_BIN_OVER;
		status = false;
		return status;
	}else{
		iap_info.iap_status = IAP_STATUS_UPGRADING;
		modbus_reg_write(REG_IAP_START_FLAG,&iap_info.iap_status,1);
		return  status;
	}
}

bool iap_head_check(void)
{
	bool status = true;
	uint16_t read_bytes = 0;
	uint8_t read_data[1024] = {0};
	if(iap_read_bytes() >= 1024){
		read_bytes = iap_read_data(read_data,1024,100);
		if(false == iap_app_name_check(read_data,1024)){
			status = false;
			return status;
		}else{;}
			
		if(false == iap_app_version_check(read_data,1024)){
			status = false;
			return status;
		}else{;}
		
		iap_bin_write(read_data,read_bytes);
		iap_delay_time = IAP_TIMEOUT;
		return true;
	}else{;}
		
	status = false;
	return status;
	
}

bool iap_finish_check(void)
{
	bool status = false;
	if(iap_fifo.pos == iap_fifo.tail){
		if(iap_info.iap_flag == IAP_FLAG_FINISH){
			status = true;
		}else{
			status = false;
		}
	}else{;}
	return status;
}

bool iap_timeout_check(void)
{
	bool status = false;
	if(iap_delay_time == 0){
		iap_info.iap_status = IAP_STATUS_TIMEOUT;
		status = true;
		return status;
	}else{
		return status;
	}
	
}



bool iap_upgrade(void)
{
	bool status = true;
	uint8_t read_data[1024] = {0};
	uint16_t read_bytes = 0;

	read_bytes = iap_read_data(read_data,1024,100);
	if(read_bytes == 0)  return status;
	if((iap_info.bin_size+read_bytes) < APP_MAX_SIZE){
		iap_bin_write(read_data,read_bytes);
	}else{
		status = false;
	}
	
	iap_delay_time = IAP_TIMEOUT;
	return status;
}

bool iap_bin_crc(void)
{
	bool status = true;
    FILE* file = fopen(collect_cache_bin_file, "rb");
    if(file == NULL){
        ESP_LOGE(TAG, "Failed to open file for upgrade,not found bin file");
        return false;
    }
    fseek(file, 0L, SEEK_END);  // ��λ���Ƶ��ļ���β
    size_t size = ftell(file);
    rewind(file);

	uint8_t *data = heap_caps_malloc(size, MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    size_t read_num = fread(data, 1, size, file);


	iap_info.bin_crc = iap_crc_calculate(data,iap_info.bin_size);
	ESP_LOGI(TAG, "bin crc is:%04x,cal_crc is:%04x",iap_info.upgrade_crc,iap_info.bin_crc);
	
	if(iap_info.bin_crc == iap_info.upgrade_crc){
		iap_info.iap_status = IAP_STATUS_FINISH;
		modbus_reg_write(REG_IAP_START_FLAG,&iap_info.iap_status,1);
		status = true;
	}else{
		iap_info.iap_status = IAP_STATUS_CRC_ERROR;
		status = false;
	}
	heap_caps_free(data);
	fclose(file);
	return status;
}

bool iap_app_update(void)
{
	bool status = false;
	FILE* file_cache = fopen(collect_cache_bin_file, "rb");
	FILE* file_bin = fopen(collect_bin_file, "w");
	if(file_cache == NULL || file_bin == NULL){
		ESP_LOGE(TAG, "Failed to open cache file bin file");
		return status;
	}

	fseek(file_cache, 0L, SEEK_END);  // ��λ���Ƶ��ļ���β
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

void iap_upgrade_process(void *param)
{
	switch(iap_info.process)
	{
		case IAP_PROCESS_IDLE:
			if(iap_start_check()){
				if(iap_bin_size_check()){
					iap_info.process = IAP_PROCESS_HEAD_CHECK;
					iap_info.bin_size = 0;
					iap_info.bin_crc = 0;
					iap_fifo.pos = 0;
					iap_fifo.tail = 0;
					iap_delay_time = IAP_TIMEOUT;
					iap_start();
				}else{
					iap_info.process = IAP_PROCESS_ERROR;
				}
			}else{;}
			break;
		case IAP_PROCESS_HEAD_CHECK:
			if(iap_head_check()){
				iap_info.process = IAP_PROCESS_UPGRADE;
			}else{;}
			if(iap_timeout_check()){
				iap_info.process = IAP_PROCESS_ERROR;
			}else{;}				
			break;
		case IAP_PROCESS_UPGRADE:
			iap_upgrade();
			if(iap_finish_check()){
				iap_info.process = IAP_PROCESS_FINISH;
				iap_bin_end();
			}else{;}
				
			if(iap_timeout_check()){
				iap_info.process = IAP_PROCESS_ERROR;
			}else{;}
			break;
		case IAP_PROCESS_FINISH:
			if(iap_bin_crc()){
				iap_app_update();
				iapBinFileUpdate = true;
				iap_get_bin_version(&iap_info.run_app_version);
				ESP_LOGI(TAG, "iap upgrade success,error code is:%d",iap_info.iap_status);
			}else{
				iap_info.process = IAP_PROCESS_ERROR;
			}
		case IAP_PROCESS_ERROR:
			iap_info.process = IAP_PROCESS_IDLE;
			iap_info.bin_size = 0;
			iap_info.bin_crc = 0;
			iap_fifo.pos = 0;
			iap_fifo.tail = 0;
			iap_bin_end();
			modbus_reg_write(REG_IAP_START_FLAG,&iap_info.iap_status,1);
			ESP_LOGI(TAG, "iap upgrade error,error code is:%d",iap_info.iap_status);
			break;
		default:
			iap_info.bin_size = 0;
			iap_info.bin_crc = 0;
			iap_info.process = IAP_PROCESS_IDLE;
			iap_bin_end();
			break;	
	}
}

void iap_task_handler(void *param)
{
	while(1){
		iap_upgrade_process(param);
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}


void iap_msg_deal_handler(uint8_t *data,uint16_t length)
{
	if(data[2] != 0x02)  return;
	uint16_t data_num = data[6] | data[7]<<8;
	if(data[3] == 0x00){
		iap_info.iap_flag = IAP_FLAG_START;
		iap_info.upgrade_size = (data[8]<<0) | (data[9]<<8) | (data[10]<<16) |(data[11]<<24) ;
		return;
	}else if(data[3] == 0x11){
		iap_info.iap_flag = IAP_FLAG_FINISH;
		iap_info.upgrade_crc = (data[8]<<0) | (data[9]<<8);
		return;
	}else if(data[3] == 0x10){
		iap_info.iap_flag = IAP_FLAG_IDLE;
	}else{
		return;
	}	
	
	if(data_num <= 1024){
		iap_write_data(&data[8],data_num,500);		//将文件发送到iap
	}else{;}
}







