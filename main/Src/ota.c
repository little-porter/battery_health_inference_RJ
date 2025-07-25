#include "ota.h"
#include "littlefs_ops.h"
#include "modbus.h"

#include "esp_ota_ops.h"
#include "esp_app_format.h"


static const char *TAG = "PRJ_OTA";

const char *inference_file = "电池健康推理软件.bin";
const char *collect_file = "battery_info_collect.bin";
const char *ota_info_file = "软件升级信息";

const char *soft_name = "电池信息采集软件";

typedef enum{ERROR=0,SUCCESS=!ERROR}ErrorStatus;
#define APP_INFO_SIZE 1024

typedef void (*pAppFunction)(void);




typedef enum _ota_process
{
	OTA_PROCESS_IDLE = 0,
	OTA_PROCESS_UPGRADE = 1,
	OTA_PROCESS_FINISH = 2,
	OTA_PROCESS_BIN_LOAD = 3,
	OTA_PROCESS_ERROR = 4,
}ota_process_t;

typedef enum _ota_flag
{
	OTA_FLAG_IDLE = 0,
	OTA_FLAG_START = 1,
	OTA_FLAG_UPGRADING = 2,
	OTA_FLAG_FINISH = 3,
}ota_falg_t;

typedef enum _ota_upgrade_flag
{
	OTA_NO_UPGRADE = 0,
	OTA_UPGRADE = 1,
}ota_upgrade_flag_t;

typedef struct _app_info
{
	char version[100];
	char name[100];
	char reserve[APP_INFO_SIZE-100-100];
}app_info_t;


typedef struct _app_version
{
	uint8_t aa;
	uint8_t bb;
	uint8_t cc;
	uint8_t day;
	uint8_t month;
	uint16_t year;
}app_version_t;


typedef struct _ota_info
{
	ota_upgrade_flag_t upgrade_flag;
	ota_process_t process;
	ota_falg_t    ota_flag;
	uint32_t bin_size;
	uint16_t bin_crc;
	uint16_t info_crc;
}ota_info_t;

ota_info_t ota_info;
ota_fifo_t inf_ota_fifo;
uint16_t   ota_delay_time = 10000;
uint8_t    app_is_executable = 0;


QueueHandle_t inf_data_queue;
QueueHandle_t col_data_queue;


#define OTA_EVENT_START_BIT  		1<<0
#define OTA_EVENT_UPGREADING_BIT   	1<<1
#define OTA_EVENT_FINISH_BIT 		1<<2
#define OTA_EVENT_ERROR_BIT 		1<<3

EventGroupHandle_t inf_event;

ErrorStatus ota_bin_load(void *des,void *src)
{
	//加载bin文件到OTA分区1
	littlefs_file_data_t file;
	littlefs_ops_read_file(collect_file,&file);
	return SUCCESS;
}

ErrorStatus ota_bin_crc(void)
{
	ErrorStatus status = SUCCESS;
	uint16_t crc = 0;
	littlefs_file_data_t file;
	littlefs_ops_read_file(inference_file,&file);
	
	if(file.data == NULL) return ERROR;
//	iap_calculate_crc(data,iap_info.bin_size);
	crc = modbus_calculate_crc((uint8_t *)file.data,file.size);
	printf("crc:%04x\r\n",crc);

	if(crc == ota_info.bin_crc){
		status = SUCCESS;
	}else{
		status = ERROR;
	}
	
	heap_caps_free(file.data);
	return status;
}

void ota_crc_set(uint16_t crc)
{
	ota_info.bin_crc = crc;
}


void ota_info_save(void)
{
	littlefs_ops_write_file(ota_info_file,(char *)&ota_info,sizeof(ota_info_t));
}


void ota_info_init(void)
{
	ota_info.upgrade_flag = OTA_NO_UPGRADE;
	ota_info.process = OTA_PROCESS_IDLE;
	ota_info.ota_flag = OTA_FLAG_IDLE;
	ota_info.bin_size = 0;
	ota_info.bin_crc = 0;
	uint16_t size = sizeof(ota_upgrade_flag_t) + sizeof(ota_process_t) + sizeof(ota_falg_t) + sizeof(uint32_t)  + sizeof(uint16_t);
	ota_info.info_crc = modbus_calculate_crc((uint8_t *)&ota_info,size);
	ota_info_save();
}


void ota_read_info(void)
{
	littlefs_file_data_t file;
	littlefs_ops_read_file(ota_info_file,&file);
	if(file.data == NULL){
		ota_info_init();
		return;
	}

	memcpy(&ota_info,file.data,file.size);
	uint16_t size = sizeof(ota_upgrade_flag_t) + sizeof(ota_process_t) + sizeof(ota_falg_t) + sizeof(uint32_t)  + sizeof(uint16_t);
	uint16_t crc = modbus_calculate_crc((uint8_t *)&file.data,size);
	if(crc != ota_info.info_crc){							//ota区域数据校验不通过
		ota_info_init();									//初始化ota数据
	}
	
}


void ota_task_callback(void *param);
void ota_init(void)
{
	inf_data_queue = xQueueCreate(10,sizeof(ota_fifo_t));
	inf_event = xEventGroupCreate();
	// ota_read_info();

	xTaskCreatePinnedToCore(ota_task_callback,"ota_task",1024*7,NULL,5,NULL,0);

}

void iap_flag_idle(void)
{
	xEventGroupSetBits(inf_event,OTA_EVENT_START_BIT);
}


void iap_flag_start(void)
{
	xEventGroupSetBits(inf_event,OTA_EVENT_START_BIT);
}

void iap_flag_finish(void)
{
	xEventGroupSetBits(inf_event,OTA_EVENT_FINISH_BIT);
}

void iap_flag_upgrading(void)
{
	xEventGroupSetBits(inf_event,OTA_EVENT_UPGREADING_BIT);
}

void ota_push_msg_to_queue(uint8_t *pdata,uint16_t num,uint32_t timeout)
{
	// ota_fifo_t msg;
	// memcpy(&msg.data,pdata,num);
	// msg.length = num;
	// xQueueSend(inf_data_queue,&msg,timeout);
}

void ota_data_write_to_fifo(ota_fifo_t *fifo,uint8_t *pdata,uint16_t num)
{
	if((fifo->tail+num) < OTA_FIFO_SIZE){
		memcpy(&fifo->data[fifo->tail],pdata,num);
	}else{
		memcpy(&fifo->data[fifo->tail],pdata,OTA_FIFO_SIZE-fifo->tail);
		memcpy(&fifo->data[0],pdata+OTA_FIFO_SIZE-fifo->tail,num - (OTA_FIFO_SIZE-fifo->tail));
	}
	
	fifo->tail = fifo->tail + num;
	fifo->tail %= OTA_FIFO_SIZE;
}

uint16_t ota_data_read_form_fifo(ota_fifo_t *fifo,uint8_t *pdata,uint16_t num,uint16_t timeout)
{
	uint16_t data_num;
	if(fifo->pos < fifo->tail){
		data_num = fifo->tail - fifo->pos;
	}else if(fifo->pos > fifo->tail){
		data_num = OTA_FIFO_SIZE - fifo->pos + fifo->tail;
	}else{
		data_num = 0;
	}
	
	if(data_num > num){
		data_num = num;
	}else{
		data_num = data_num;
	}
	
	if((fifo->pos+data_num) < OTA_FIFO_SIZE){
		memcpy(pdata,&fifo->data[fifo->pos],data_num);
	}else{
		memcpy(pdata,&fifo->data[fifo->pos],OTA_FIFO_SIZE-fifo->pos);
		memcpy(pdata+OTA_FIFO_SIZE-fifo->pos,&fifo->data[0],data_num-(OTA_FIFO_SIZE-fifo->pos));
	}
	
	fifo->pos = fifo->pos + data_num;
	fifo->pos %= OTA_FIFO_SIZE;
	
	return data_num;		//返回读取数量
}



ErrorStatus iap_idle_process(void)
{
	ErrorStatus status = SUCCESS;
	if(ota_info.ota_flag == OTA_FLAG_START){
		status = SUCCESS;
	}else{
		status = ERROR;
	}
	return status;
}



ErrorStatus ota_upgrade_process(void)
{
	ErrorStatus status = SUCCESS;
	static uint16_t count = 0;
	uint8_t read_data[1024];
	uint16_t read_num = 0;
	read_num = ota_data_read_form_fifo(&inf_ota_fifo,read_data,1024,10);
	vTaskDelay(10);
	if(read_num <= 0) return status;

	count++;
	littlefs_ops_write_file_append(inference_file,(char *)read_data,read_num);
	printf("write %d\r\n",count);
	
	return status;
}

void ota_task_callback(void *param)
{
	ErrorStatus	status = SUCCESS;
	while (inf_data_queue == NULL);
	
	while(1)
	{
		switch (ota_info.process)
		{
		case OTA_PROCESS_IDLE:
			/* code */
			if(xEventGroupWaitBits(inf_event,OTA_EVENT_START_BIT,true,true,portMAX_DELAY)&OTA_EVENT_START_BIT){
				ota_info.process = OTA_PROCESS_UPGRADE;
				printf("ota start!\r\n");
				littlefs_ops_remove_file(inference_file);
			}else{;}
			break;
		case OTA_PROCESS_UPGRADE:
			ota_upgrade_process();
			if(xEventGroupWaitBits(inf_event,OTA_EVENT_FINISH_BIT,true,true,pdMS_TO_TICKS(5))&OTA_EVENT_FINISH_BIT){
				ota_info.process = OTA_PROCESS_FINISH;
			}else{;}
			break;
		case OTA_PROCESS_FINISH:
			if(ota_bin_crc()){
				printf("ota finish,bin crc success!\r\n");
				ota_info.process = OTA_PROCESS_IDLE;
			}else{
				printf("ota finish,bin crc fail!\r\n");
				ota_info.process = OTA_PROCESS_ERROR;
			}
			break;
		case OTA_PROCESS_ERROR:

			break;
		default:
			break;
		}
		
	}
}

// 将版本字符串解析为整数，例如 "1.2.3" -> 0x010203
unsigned int ota_parse_version(const char *version_str) {
    unsigned int major = 0, minor = 0, patch = 0;
    sscanf(version_str, "%u.%u.%u", &major, &minor, &patch);
    return (major << 16) | (minor << 8) | patch;
}

// 比较两个版本字符串
int ota_compare_versions(const char *v1, const char *v2) {
    unsigned int ver1 = ota_parse_version(v1);
    unsigned int ver2 = ota_parse_version(v2);

    if (ver1 < ver2) return -1;   // v1 < v2
    if (ver1 > ver2) return 1;    // v1 > v2
    return 0;                     // equal
}
void iap_task(void)
{
	// if(app_is_executable){
	// 	if(iap_delay_time == 0){
	// 		iap_jump_app();
	// 	}else{
	// 		iap_task_callback(NULL);
	// 	}
	// }else{
	// 	iap_task_callback(NULL);
	// }
	ota_bin_crc();
	int size = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t);
	esp_app_desc_t runnimg_app_info;

	esp_partition_t *running_partition = esp_ota_get_running_partition();

	if(esp_ota_get_partition_description(running_partition,&runnimg_app_info) == ESP_OK){
		ESP_LOGI(TAG,"running_app_info.version:%s\r\n",runnimg_app_info.version);
		ESP_LOGI(TAG,"running_app_info.project_name:%s\r\n",runnimg_app_info.project_name);
	}

	printf("size:%d\r\n",size);
}








