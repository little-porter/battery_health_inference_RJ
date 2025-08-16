#include "ota.h"
#include "littlefs_ops.h"
#include "esp_partition.h"
#include "esp_app_format.h"
#include "esp_ota_ops.h"

#include "modbus.h"

#define REG_OTA_START_FLAG 0x0003
#define REG_OTA_VERSION_AA 0x3003
#define REG_OTA_VERSION_BB 0x3004
#define REG_OTA_VERSION_CC 0x3005
#define OTA_OUT_TIME_MS	   10000

static const char *TAG = "PRJ_OTA";
static const char *app_name = "battery_inference_ota";

static const char *ota_flie = "ota_info.text";

typedef enum _ota_run_app{
	OTA_RUN_FACTORY = 0,
	OTA_RUN_APP1 	= 1,
	OTA_RUN_APP2 	= 2,
}ota_run_app_t;

typedef enum _ota_process{
	OTA_PROCESS_IDLE = 0,
	OTA_PROCESS_START,
	OTA_PROCESS_UPGRADING,
	OTA_PROCESS_FINISH,
}ota_process_t;

typedef enum _ota_status{ 
	OTA_STATUS_UPGRADING = 0,
	OTA_STATUS_FINISH = 1,
	OTA_STATUS_BIN_ERROR,
	OTA_STATUS_VERSION_ERROR,
	OTA_STATUS_CRC_ERROR,
	OTA_STATUS_TIMEOUT,
}ota_status_t;

typedef struct _ota_info{ 
	uint32_t app1_size;
	uint32_t app2_size;
	uint16_t app1_crc;
	uint16_t app2_crc;
	uint32_t upgrade_size;
	uint16_t upgrade_crc;
	ota_run_app_t run_app;
	ota_process_t process;
	uint16_t  status;
	esp_ota_handle_t ota_handle;
	esp_partition_t *run_partition;
	esp_partition_t *ota_partition;
	uint32_t out_times;
} ota_info_t;

ota_info_t ota_info;

#define OTA_EVENT_START			(1<<0)
#define OTA_EVENT_UPGRADING		(1<<1)
#define OTA_EVENT_FINISH		(1<<2)

EventGroupHandle_t ota_event_group;


ota_fifo_t app_fifo;
void ota_info_init(void)
{
	ota_info.app1_size = 0;
	ota_info.app2_size = 0;
	ota_info.app1_crc = 0;
	ota_info.app2_crc = 0;
	ota_info.run_app = OTA_RUN_FACTORY;
}

void ota_info_save(void)
{
	littlefs_ops_write_file(ota_flie,(char *)&ota_info,sizeof(ota_info));
}

void ota_info_load(void)
{
	if(true != littlefs_ops_read_file(ota_flie,(littlefs_file_data_t *)&ota_info)){
		ota_info_init();
		ota_info_save();
	}

}

void app_info_check(void)
{
	esp_partition_t *run_partition = NULL;
	esp_app_desc_t run_app_info;
	if(ota_info.run_app == OTA_RUN_APP1){
		// run_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_APP_FACTORY,"app1");
		if(run_partition){
			if(esp_ota_get_partition_description(run_partition,&run_app_info) == ESP_OK){
				ESP_LOGI(TAG,"run_app_info.version:%s\r\n",run_app_info.version);
				ESP_LOGI(TAG,"run_app_info.project_name:%s\r\n",run_app_info.project_name);
				if(0 != strcmp(run_app_info.project_name,app_name)){						//判断运行APP软件名称
					 
				}
			}
		}
	}else if(ota_info.run_app == OTA_RUN_APP2){
		// run_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_APP_FACTORY,"app2");
		if(run_partition){
			if(esp_ota_get_partition_description(run_partition,&run_app_info) == ESP_OK){
				ESP_LOGI(TAG,"run_app_info.version:%s\r\n",run_app_info.version);
				ESP_LOGI(TAG,"run_app_info.project_name:%s\r\n",run_app_info.project_name);
			}
		}
	}else{;}
}

void ota_data_write_to_fifo(uint8_t *pdata,uint16_t num)
{
	if((app_fifo.tail+num) < OTA_FIFO_SIZE){
		memcpy(&app_fifo.data[app_fifo.tail],pdata,num);
	}else{
		memcpy(&app_fifo.data[app_fifo.tail],pdata,OTA_FIFO_SIZE-app_fifo.tail);
		memcpy(&app_fifo.data[0],pdata+OTA_FIFO_SIZE-app_fifo.tail,num - (OTA_FIFO_SIZE-app_fifo.tail));
	}
	
	app_fifo.tail = app_fifo.tail + num;
	app_fifo.tail %= OTA_FIFO_SIZE;
}

uint16_t ota_data_read_form_fifo(ota_fifo_t *fifo,uint8_t *pdata,uint16_t num)
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

uint16_t ota_fifo_data_bytes_read(ota_fifo_t *fifo)
{
	uint16_t data_num;
	if(fifo->pos < fifo->tail){
		data_num = fifo->tail - fifo->pos;
	}else if(fifo->pos > fifo->tail){
		data_num = OTA_FIFO_SIZE - fifo->pos + fifo->tail;
	}else{
		data_num = 0;
	}
	return data_num;
}

bool ota_info_head_check(void)
{
	bool ret = false;
	int size = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t);
	uint32_t read_bytes = 0;
	/*获取校验头 */
	read_bytes = ota_fifo_data_bytes_read(&app_fifo);
	vTaskDelay(pdMS_TO_TICKS(10));
	if(read_bytes < size){
		ret = false;			/*校验头不全*/
	} else{
		ret = true;				/*校验头全了*/
	}
	return ret;
	
}


bool ota_info_check(void)
{
	bool ret = false;
	uint32_t read_bytes = 0;
	int size = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t);
	/*校验头全，申请校验头保存区域*/
	uint8_t *data = heap_caps_calloc(1,size,MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
	read_bytes = ota_data_read_form_fifo(&app_fifo,data,size);		//读取数据

	esp_app_desc_t new_app_info;
	if(read_bytes == size){ 
		memcpy(&new_app_info, data + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t), sizeof(esp_app_desc_t));
		if(strcmp(new_app_info.project_name, app_name) != 0){
			heap_caps_free(data);
			return ret;
		}	

		/*校验成功，开始写入*/
		// ota_info.ota_partition = esp_ota_get_next_update_partition(NULL);
		// ESP_ERROR_CHECK(esp_ota_begin(ota_info.ota_partition, OTA_SIZE_UNKNOWN, &ota_info.ota_handle));
		ESP_ERROR_CHECK(esp_ota_write(ota_info.ota_handle, data, size));
		ota_info.app1_size += read_bytes;
		ESP_LOGI(TAG, "ota_write_data: %d", (int)ota_info.app1_size);
		ret = true;
		heap_caps_free(data);
		return ret;
	}
	heap_caps_free(data);
	return ret;
}

bool ota_process_upgrading(void)
{
	bool ret = false;
	uint8_t read_data[1024] = {0};
	uint16_t read_bytes = 0;
	read_bytes = ota_data_read_form_fifo(&app_fifo,read_data,1024);		//读取数据
	if(read_bytes){
		// ESP_ERROR_CHECK(esp_ota_write(ota_info.ota_handle, read_data, read_bytes));
		esp_ota_write(ota_info.ota_handle, read_data, read_bytes);
		ota_info.app1_size += read_bytes;
		ret = true;
		ESP_LOGI(TAG, "ota_write_data: %d", (int)ota_info.app1_size);
	}else{;}
	vTaskDelay(pdMS_TO_TICKS(10));
	return ret;
}

bool ota_data_crc(void)
{
	bool ret = true;
	uint16_t crc = ota_info.upgrade_crc;
	uint16_t cal_crc  = 0xFFFF;
	uint16_t cal_num = 0;
	// uint8_t *read_data = (uint8_t *)heap_caps_calloc(1, ota_info.app1_size, MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
	// if(ESP_OK == esp_partition_read(ota_info.ota_partition, 0, read_data, ota_info.app1_size)){
	// 	cal_crc = modbus_calculate_crc(read_data,ota_info.app1_size);
	// }
	// heap_caps_free(read_data);
	uint8_t read_data[1024] = {0};
	
	for(uint32_t offset = 0; (offset + 1024) <= ota_info.app1_size; offset += 1024){
		memset(read_data, 0, 1024);
		if(ESP_OK == esp_partition_read(ota_info.ota_partition, offset, read_data, 1024)){
			cal_crc = modbus_calculate_crc_ota(cal_crc,(uint8_t *)&read_data[0],(uint32_t)1024);
			cal_num++;
		}
		
	}
	if(ota_info.app1_size%1024 > 0){
		memset(read_data, 0, 1024);
		if(ESP_OK == esp_partition_read(ota_info.ota_partition, (ota_info.app1_size/1024)*1024, read_data, ota_info.app1_size%1024)){
			cal_crc = modbus_calculate_crc_ota(cal_crc,(uint8_t *)&read_data[0],(uint32_t)(ota_info.app1_size%1024));
			cal_num++;
		}	
	}

	printf("crc:%x,cal_crc:%x,cal_num:%d\r\n",crc,cal_crc,cal_num);
	if(ota_info.upgrade_size != ota_info.app1_size){
		ret = false;
	}else{;}

	// if(crc != cal_crc){
	// 	ret = false;
	// }else{;}

	return ret;
}

void ota_process_end(void)
{
	if(!ota_info.ota_handle){
		esp_ota_abort(ota_info.ota_handle);
		esp_ota_end(ota_info.ota_handle);
	}else{;}
}

bool ota_out_times_check(void)
{
	if(ota_info.out_times == 0){
		return true;
	}else{return false;}
}
void ota_out_times_dec(void)
{
	if(ota_info.out_times > 0){
		ota_info.out_times = ota_info.out_times - 1000;
	}
}

void ota_out_times_reload(void)
{
	ota_info.out_times = OTA_OUT_TIME_MS;
}

void ota_task_handler(void *param)
{
	ESP_LOGI(TAG,"ota_task_running...");
	while(1){
		switch (ota_info.process)
		{
		case OTA_PROCESS_IDLE:
			if (xEventGroupWaitBits(ota_event_group,OTA_EVENT_START,pdFALSE,pdTRUE,10)&OTA_EVENT_START){
				xEventGroupClearBits(ota_event_group,0x00FFFFFFUL);
				ota_info.process = OTA_PROCESS_START;
				ota_info.app1_size = 0;
				ota_info.app1_crc = 0;
				ota_info.status = OTA_STATUS_UPGRADING;
				modbus_reg_write(REG_OTA_START_FLAG,&ota_info.status,1);
				app_fifo.pos = 0;
				app_fifo.tail = 0;
				ota_out_times_reload();
				ESP_ERROR_CHECK(esp_ota_begin(ota_info.ota_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_info.ota_handle));
				ESP_LOGI(TAG,"ota process start...");
			}else{;}
			break;
		case OTA_PROCESS_START:
			if(ota_info_head_check()){
				if(ota_info_check()){
					ota_info.process = OTA_PROCESS_UPGRADING;
					ota_out_times_reload();
					ESP_LOGI(TAG,"ota head check success...");
				}else{
					ota_process_end();
					ota_info.process = OTA_PROCESS_IDLE;
					ota_info.status = OTA_STATUS_BIN_ERROR;
					modbus_reg_write(REG_OTA_START_FLAG,&ota_info.status,1);
					ESP_LOGI(TAG,"ota head check fail...");
				}
			}else{
				if(ota_out_times_check()){
					ota_process_end();
					ota_info.process = OTA_PROCESS_IDLE;
					ota_info.status = OTA_STATUS_TIMEOUT;
					modbus_reg_write(REG_OTA_START_FLAG,&ota_info.status,1);
					ESP_LOGI(TAG,"ota out time...");
				}
			}
			break;
		case OTA_PROCESS_UPGRADING:
			if(ota_info.app1_size + 1024 > 3*1024*1024){					//文件大于3M，退出ota
				ota_process_end();
				ota_info.process = OTA_PROCESS_IDLE;
				ota_info.status = OTA_STATUS_BIN_ERROR;
				modbus_reg_write(REG_OTA_START_FLAG,&ota_info.status,1);
				ESP_LOGI(TAG,"ota out time...");
			}else{;}

			if(ota_process_upgrading()){				//升级数据处理	
				ota_out_times_reload();
			}else{
				if(ota_out_times_check()){
					ota_process_end();
					ota_info.process = OTA_PROCESS_IDLE;
					ota_info.status = OTA_STATUS_TIMEOUT;
					modbus_reg_write(REG_OTA_START_FLAG,&ota_info.status,1);
					ESP_LOGI(TAG,"ota out time...");
				}
			}
			/*结束判断*/
			if ((xEventGroupWaitBits(ota_event_group,OTA_EVENT_FINISH,pdFALSE,pdTRUE,10)&OTA_EVENT_FINISH) && (0 == ota_fifo_data_bytes_read(&app_fifo))){
				xEventGroupClearBits(ota_event_group,0x00FFFFFFUL);
				ota_info.process = OTA_PROCESS_FINISH;
				ESP_LOGI(TAG,"ota process finish...");
				ota_process_end();
			}else{;}
			break;
		case OTA_PROCESS_FINISH: 
 			if(ota_data_crc()){
				esp_ota_set_boot_partition(ota_info.ota_partition);
				ESP_LOGI(TAG,"ota success ,restart system...");
				ota_info.status = OTA_STATUS_FINISH;
				modbus_reg_write(REG_OTA_START_FLAG,&ota_info.status,1);
				esp_restart();
			}else{
				ota_info.process = OTA_PROCESS_IDLE;
				ota_info.status = OTA_STATUS_CRC_ERROR;
				modbus_reg_write(REG_OTA_START_FLAG,&ota_info.status,1);
				ESP_LOGI(TAG,"ota crc check fail...");
			}
			break;
		default:
			ota_info.process = OTA_PROCESS_IDLE;
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

void ota_read_running_app_version(void)
{
	esp_partition_t *running_partition = esp_ota_get_running_partition();
	esp_app_desc_t running_app_info;
	if(esp_ota_get_partition_description(running_partition,&running_app_info) == ESP_OK){
		ESP_LOGI(TAG,"running_app_info.version:%s\r\n",running_app_info.version);
		ESP_LOGI(TAG,"running_app_info.project_name:%s\r\n",running_app_info.project_name);
	}
	uint32_t version = ota_parse_version(running_app_info.version);
	uint16_t version_aa = (version >> 16) & 0xff;
	uint16_t version_bb = (version >> 8) & 0xff;
	uint16_t version_cc = (version >> 0) & 0xff;
	modbus_reg_write(REG_OTA_VERSION_AA,&version_aa,1);
	modbus_reg_write(REG_OTA_VERSION_BB,&version_bb,1);
	modbus_reg_write(REG_OTA_VERSION_CC,&version_cc,1);
}



TimerHandle_t ota_timer;
void ota_timer_callback(TimerHandle_t xTimer)
{
	ota_out_times_dec();
}

void ota_init(void)
{
	/*创建升级事件 */
	ota_event_group = xEventGroupCreate();
	ota_info.ota_partition = esp_ota_get_next_update_partition(NULL);
	/*加载ota信息*/
	ota_info_load();
	ota_info.process = OTA_PROCESS_IDLE;
	/*保存版本信息到modbus寄存器*/
	ota_read_running_app_version();
	/*状态信息保存到modbus寄存器*/
	ota_info.status = OTA_STATUS_FINISH;
	modbus_reg_write(REG_OTA_START_FLAG,&ota_info.status,1);
	/*创建超时检测定时器*/
	ota_timer = xTimerCreate("ota_timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, ota_timer_callback);
	xTimerStart(ota_timer, 0);

	if(OTA_RUN_FACTORY == ota_info.run_app){	//创建更新任务
		xTaskCreatePinnedToCore(ota_task_handler, "ota_task", 1024*6, NULL, 7, NULL, 0);
	}else{										/*校验app信息*/
		app_info_check();
	}	
	
}




void ota_data_deal_handler(uint8_t *pdata,uint16_t num)
{
	uint16_t data_num = (pdata[6]<<0) | (pdata[7]<<8);
    uint8_t  frm_flag = pdata[3];
    uint8_t  frm_type = pdata[2];
    if(frm_type != 0x01 )        return;

    if(frm_flag == 0x00){       //升级开始标志
        xEventGroupSetBits(ota_event_group,OTA_EVENT_START);
		ota_info.upgrade_size = (pdata[8]<<0) | (pdata[9]<<8) | (pdata[10]<<16) |(pdata[11]<<24) ;
        return;
    }else if(frm_flag == 0x10){ //升级数据
        // iap_flag_upgrading();
    }else if(frm_flag == 0x11){ //升级结束标志
        xEventGroupSetBits(ota_event_group,OTA_EVENT_FINISH);
        ota_info.upgrade_crc = (pdata[8]<<0) | (pdata[9]<<8);
        // ESP_LOGI("OTA_CRC", "OTA_CRC:0x%x",(pdata[9]<<8) | pdata[8]);
        return;
    }else{return;}

	if(data_num <= 1024){
		ota_data_write_to_fifo(&pdata[8],data_num);
	}else{;}
    
    // printf("OTA_PUSH_DATA: %d\n",data_num);
}



