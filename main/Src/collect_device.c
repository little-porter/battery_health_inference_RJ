#include "collect_device.h"
#include "modbus.h"
#include "sysEvent.h"

static const char *TAG = "PRJ_COLLECT";

static const char *collect_bin_file = "/littlefs/battery_info_collect.bin";
static const char *collect_cache_bin_file = "/littlefs/battery_info_collect_cache.bin";

#define COLLECT_UART_PORT     UART_NUM_1
#define COLLECT_UART_BAUD     115200
#define COLLECT_UART_TX_PIN   GPIO_NUM_43
#define COLLECT_UART_RX_PIN   GPIO_NUM_44
// #define COLLECT_EN_PIN        GPIO_NUM_45

#define COLLECT_DEVICE_ADDR         0x01
#define COLLECT_DEVICE_BROADCAST_ADDR    0xFF
#define COLLECT_DEVICE_ADDR_IDX     0
#define COLLECT_DEVICE_FUNC_IDX     1
#define REG_NUM_IDX                 2
#define REG_DATA_IDX                4


#define REG_COLLECT_VERSION_YEAR    0x3006
#define REG_COLLECT_VERSION_MONTH   0x3007
#define REG_COLLECT_VERSION_DAY     0x3008
#define REG_COLLECT_VERSION_AA      0x3009
#define REG_COLLECT_VERSION_BB      0x300A
#define REG_COLLECT_VERSION_CC      0x300B


#define COLLECT_INFO_OFFSET				0x300

QueueHandle_t collect_device_queue = NULL;

typedef enum _collect_process_t
{
    COLLECT_PROCESS_INIT = 0,                   //初�?�化，获取版�?号，判断�?否需要更�?
    COLLECT_PROCESS_GET_VERSION,                //获取版本�?
    COLLECT_PROCESS_READ_DATA,                  //读取数据
    COLLECT_PROCESS_UPGRADE,                    //�?件升�?
    COLLECT_PROCESS_SET_CALIBRATION,
    COLLECT_PROCESS_READ_CALIBRATION ,
}collect_process_t;

collect_process_t collect_process = COLLECT_PROCESS_INIT;

#define  CALIBRATE_TABLE_LEN        8
uint16_t collect_calibrate_table[CALIBRATE_TABLE_LEN];

typedef struct _version
{
    uint16_t year,month,day;
    uint16_t aa,bb,cc;
}version_t;

version_t collect_version;

typedef enum _collect_upgrade_flag
{
    COLLECT_UPGRADE_FLAG_START = 0,
    COLLECT_UPGRADING,
    COLLECT_UPGRADE_FLAG_END,
}collect_upgrade_flag_t;


typedef struct _iap_msg
{
    uint8_t dev_addr;
    uint8_t cmd;
    uint8_t dev_type;
    uint8_t frm_flag;
    uint16_t frm_num;
    uint16_t data_len;
    uint8_t payload[0];
}iap_msg_t;

#define  collect_data_reverse(pdata,num)      modbus_reg_data_reverse(pdata,num)


void collect_send_task_handler(void *pvParameters);
void collect_recive_task_handler(void *pvParameters);

void collect_device_bin_soft_version_get(void)
{
    FILE* file = fopen(collect_bin_file, "rb");
    if(file == NULL){
        ESP_LOGE(TAG, "Failed to open file for upgrade,not found bin file");
        return;
    }
    uint8_t *data = heap_caps_malloc(1024,MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    size_t bytes_read = fread(data, 1, 1024, file);
    version_t *version = (version_t *)(data+COLLECT_INFO_OFFSET);
    memcpy(&collect_version,version,sizeof(version_t));

    ESP_LOGI(TAG, "collect_device_bin_soft_version is:%d-%d-%d  %d.%d.%d",
            (int)collect_version.year,(int)collect_version.month,(int)collect_version.day,
            (int)collect_version.aa,(int)collect_version.bb,(int)collect_version.cc);

    heap_caps_free(data);
    fclose(file);
}
void collect_device_uart_config(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = COLLECT_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,       
    };
    uart_param_config(COLLECT_UART_PORT, &uart_cfg);

    uart_set_pin(COLLECT_UART_PORT, COLLECT_UART_TX_PIN, COLLECT_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(COLLECT_UART_PORT, 1024*2, 1024*2, 10, &collect_device_queue, 0);
}


void collect_device_init(void)
{
    collect_device_uart_config();
    collect_process = COLLECT_PROCESS_INIT;
    collect_device_bin_soft_version_get();
    xTaskCreatePinnedToCore(collect_send_task_handler,"rs485_send_task",1024*5,NULL,5,NULL,0);
    xTaskCreatePinnedToCore(collect_recive_task_handler,"rs485_recive_task",1024*5,NULL,5,NULL,0);
}

typedef union _u_float_u8
{
    /* data */
    float f_data;
    uint8_t u8_data[4];
}u_float_u8_t;


void collect_device_init_read(void)
{ 
    printf("[collect_device] collect_device_init_read\r\n");
    uint8_t data[8] = {0x01, 0x04, 0x40, 0x01, 0x00, 0x08, 0x00, 0x01};
    uint16_t len = 8;
    uint16_t crc = modbus_calculate_crc(data,len-2);
    data[len-2] = crc&0xff;
    data[len-1] = crc>>8;

    uart_write_bytes(COLLECT_UART_PORT,data,len);
    uart_wait_tx_done(COLLECT_UART_PORT,portMAX_DELAY);
}
void collect_device_read_data(void)
{
    printf("[collect_device] collect_device_read_data\r\n");
    uint8_t data[8] = {0x01, 0x03, 0x10, 0x00, 0x00, 0x0B, 0x00, 0xCD};
    uint16_t len = 8;

    uart_write_bytes(COLLECT_UART_PORT,data,len);
    uart_wait_tx_done(COLLECT_UART_PORT,portMAX_DELAY);
}

void collect_device_set_calibration(void)
{ 
    printf("[collect_device] collect_device_set_calibration\r\n");
    uint8_t data[26] = {0x01, 0x10, 0x40, 0x00, 0x00, 0x09, 0x00, 0x01,\
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
                        0x00, 0x0D};
    uint16_t len = 26;
    uint16_t data_reg[8] = {0};
    modbus_reg_read_no_reverse(0x4001,data_reg,8);
    memcpy(data+8,data_reg,8*2);
    uint16_t crc = modbus_calculate_crc(data,len-2);
    data[len-2] = crc&0xff;
    data[len-1] = crc>>8;

    uart_write_bytes(COLLECT_UART_PORT,data,len);
    uart_wait_tx_done(COLLECT_UART_PORT,portMAX_DELAY);
}

void collect_device_read_calibration(void)
{ 
    printf("[collect_device] collect_device_read_calibration\r\n");
    uint8_t data[8] = {0x01, 0x04, 0x40, 0x01, 0x00, 0x08, 0x00, 0x01};
    uint16_t len = 8;
    uint16_t crc = modbus_calculate_crc(data,len-2);
    data[len-2] = crc&0xff;
    data[len-1] = crc>>8;

    uart_write_bytes(COLLECT_UART_PORT,data,len);
    uart_wait_tx_done(COLLECT_UART_PORT,portMAX_DELAY);
}

void collect_devicie_version_get(void)
{
    printf("[collect_device] collect_devicie_version_get\r\n");
    uint8_t data[8] = {0x01, 0x03, 0x30, 0x00, 0x00, 0x06, 0x00, 0x01};
    uint16_t len = 8;
    uint16_t crc = modbus_calculate_crc(data,len-2);
    data[len-2] = crc&0xff;
    data[len-1] = crc>>8;

    uart_write_bytes(COLLECT_UART_PORT,data,len);
    uart_wait_tx_done(COLLECT_UART_PORT,portMAX_DELAY);
}

void collect_device_upgrade_data_send(const char *data,uint32_t len,uint16_t frm_num,collect_upgrade_flag_t flag)
{
    uint32_t size = sizeof(iap_msg_t)+len+2;
    iap_msg_t *iap_msg = (iap_msg_t *)heap_caps_malloc(size,MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    uint16_t crc = 0;
    iap_msg->dev_addr = COLLECT_DEVICE_ADDR;
    iap_msg->cmd = 0xff;
    iap_msg->dev_type = 0x02;
    if(flag == COLLECT_UPGRADE_FLAG_START){
        iap_msg->frm_flag = 0x00;
    }else if(flag == COLLECT_UPGRADE_FLAG_END){
        iap_msg->frm_flag = 0x11;
    }else{
        iap_msg->frm_flag = 0x10;
    }
    iap_msg->frm_num = frm_num>>8 | ((frm_num&0xff)<<8);
    iap_msg->data_len = len;
    memcpy(iap_msg->payload,data,len);
    crc = modbus_calculate_crc((uint8_t *)iap_msg,size-2);
    iap_msg->payload[len] = crc&0xff;
    iap_msg->payload[len+1] = crc>>8;

    uart_write_bytes(COLLECT_UART_PORT,iap_msg,size);
    uart_wait_tx_done(COLLECT_UART_PORT,portMAX_DELAY);

    heap_caps_free(iap_msg);
}



void collect_device_upgrade(void)
{
    FILE* file = fopen(collect_bin_file, "rb");
    if(file == NULL){
        ESP_LOGE(TAG, "Failed to open file for upgrade,not found bin file");
        goto OPEN_FAIL;
    }
    fseek(file, 0L, SEEK_END);  // 将文件指针移动到文件�?�?
    size_t bin_size = ftell(file);
    rewind(file);
    uint16_t frm_num = 0;
    uint16_t cal_crc = 0xFFFF;
    size_t send_size = 0;
    if(bin_size == 0){
        goto OPEN_FAIL;
    }

    ESP_LOGI(TAG, "need to send File size: %d bytes", bin_size);

    collect_device_upgrade_data_send((const char*)&bin_size,sizeof(size_t),frm_num++,COLLECT_UPGRADE_FLAG_START);
    vTaskDelay(pdMS_TO_TICKS(300));

    uint8_t *data = heap_caps_malloc(1024,MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    size_t bytes_read = 0;

    
    while((bytes_read = fread(data, 1, 1024, file)) > 0){
        //发送升级数�?
        collect_device_upgrade_data_send((const char*)data,bytes_read,frm_num++,COLLECT_UPGRADING);
        cal_crc = modbus_calculate_crc_ota(cal_crc,data,bytes_read);
        send_size+=bytes_read;
        ESP_LOGI(TAG, "send File size: %d bytes", send_size);
        vTaskDelay(pdMS_TO_TICKS(300));
    }

    ESP_LOGI(TAG, "send File crc: %04x", cal_crc);
    collect_device_upgrade_data_send((const char*)&cal_crc,sizeof(uint16_t),frm_num++,COLLECT_UPGRADE_FLAG_END);

    heap_caps_free(data);
    fclose(file);

OPEN_FAIL:
    collect_process = COLLECT_PROCESS_READ_DATA;
}


void collect_send_task_handler(void *pvParameters)
{
    uint16_t reply = 0;
    while(1)
    {
        uint32_t collect_event = 0;
        sysEvent_get(collect_dev_event_group,&collect_event);
        if(collect_event&COLLECT_DEV_CALIB_EVENT_BIT){
            sysEvent_clear(collect_dev_event_group,COLLECT_DEV_CALIB_EVENT_BIT);
            collect_process = COLLECT_PROCESS_SET_CALIBRATION;
        }else if(collect_event&COLLECT_DEV_UPGRADE_EVENT_BIT){
            sysEvent_clear(collect_dev_event_group,COLLECT_DEV_UPGRADE_EVENT_BIT);
            collect_process = COLLECT_PROCESS_GET_VERSION;
        }else{;}


        switch (collect_process)
        {
        case COLLECT_PROCESS_INIT:
            collect_device_init_read();
            break;
        case COLLECT_PROCESS_GET_VERSION:
            collect_devicie_version_get();
            break;
        case COLLECT_PROCESS_UPGRADE: 
            collect_device_upgrade();
            break;
        case COLLECT_PROCESS_READ_DATA:
            collect_device_read_data();
            break;
        case COLLECT_PROCESS_SET_CALIBRATION:
            collect_device_set_calibration();
            break;
        case COLLECT_PROCESS_READ_CALIBRATION:
            collect_device_read_calibration();
            break;
        default:
            break;
        }   

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


void collect_device_init_msg_deal(uint8_t *data,uint16_t length)
{
    uint16_t crc=0,cal_crc=0;
    if(data[COLLECT_DEVICE_ADDR_IDX] != COLLECT_DEVICE_ADDR) return;
    crc = data[length-2] | data[length-1]<<8;
    cal_crc = modbus_calculate_crc(data,length-2);
    if(crc != cal_crc) return;

    printf("init calibration read crc = %04x , cal_crc = %04x\r\n",crc,cal_crc);

    uint16_t num = data[REG_NUM_IDX]<<8 | data[REG_NUM_IDX+1];
    
    uint16_t *data_ptr = (uint16_t *)&data[REG_DATA_IDX];

    modbus_reg_write_no_reverse(0x4001,data_ptr,num/2);

    collect_process = COLLECT_PROCESS_GET_VERSION;
}

void collect_device_data_msg_deal(uint8_t *data,uint16_t length)
{
    uint16_t crc=0,cal_crc=0;
    if(data[COLLECT_DEVICE_ADDR_IDX] != COLLECT_DEVICE_ADDR) return;
    crc = data[length-2] | data[length-1]<<8;
    cal_crc = modbus_calculate_crc(data,length-2);
    if(crc != cal_crc) return;

    // printf("data crc = %04x , cal_crc = %04x\r\n",crc,cal_crc);
   
    uint16_t num = data[REG_NUM_IDX]<<8 | data[REG_NUM_IDX+1];
    
    uint16_t *data_ptr = (uint16_t *)&data[REG_DATA_IDX];
    modbus_reg_write_no_reverse((uint16_t)0x1000,&data_ptr[0],1);
    modbus_reg_write_no_reverse((uint16_t)0x1002,&data_ptr[2],1);
    modbus_reg_write_no_reverse((uint16_t)0x1005,&data_ptr[1],1);
    modbus_reg_write_no_reverse((uint16_t)0x1006,&data_ptr[6],1);
    modbus_reg_write_no_reverse((uint16_t)0x1007,&data_ptr[7],1);
    modbus_reg_write_no_reverse((uint16_t)0x1008,&data_ptr[8],1);
    modbus_reg_write_no_reverse((uint16_t)0x1009,&data_ptr[5],1);
    modbus_reg_write_no_reverse((uint16_t)0x100A,&data_ptr[9],1);

    // modbus_reg_write(0x1000,&data[REG_DATA_IDX],num/2);
}


void collect_device_calibration_msg_deal(uint8_t *data,uint16_t length)
{
    uint16_t crc=0,cal_crc=0;
    if(data[COLLECT_DEVICE_ADDR_IDX] != COLLECT_DEVICE_ADDR) return;
    crc = data[length-2] | data[length-1]<<8;
    cal_crc = modbus_calculate_crc(data,length-2);
    if(crc != cal_crc) return;

    printf("calibration read crc = %04x , cal_crc = %04x\r\n",crc,cal_crc);

    uint16_t num = data[REG_NUM_IDX]<<8 | data[REG_NUM_IDX+1];
    
    uint16_t *data_ptr = (uint16_t *)&data[REG_DATA_IDX];

    modbus_reg_write_no_reverse(0x4001,data_ptr,num/2);

    collect_process = COLLECT_PROCESS_READ_DATA;
}

void collect_device_version_msg_deal(uint8_t *data,uint16_t length)
{
    version_t version;
    uint16_t *data_ptr = (uint16_t *)&data[REG_DATA_IDX];
    version.year = data_ptr[0];
    version.month = data_ptr[1];
    version.day = data_ptr[2];
    version.aa = data_ptr[3];
    version.bb = data_ptr[4];
    version.cc = data_ptr[5];
    collect_data_reverse((uint16_t *)&version,sizeof(version_t)/2);

    ESP_LOGI(TAG, "collect_device_run_soft_version is:%d-%d-%d  %d.%d.%d",
            (int)version.year,(int)version.month,(int)version.day,
            (int)version.aa,(int)version.bb,(int)version.cc);

    modbus_reg_write_no_reverse(REG_COLLECT_VERSION_YEAR,data_ptr,6);


    if(0 > memcmp(&version,&collect_version,sizeof(version_t))){
        collect_process = COLLECT_PROCESS_UPGRADE;
    } else{
        collect_process = COLLECT_PROCESS_READ_DATA;
    } 
}


void collect_msg_deal(uint8_t *data,uint16_t length)
{
    switch (collect_process)
    {
    case COLLECT_PROCESS_INIT:
        collect_device_init_msg_deal(data,length);
        break;
    case COLLECT_PROCESS_GET_VERSION:
        collect_device_version_msg_deal(data,length);
        break;
    case COLLECT_PROCESS_READ_DATA:
        collect_device_data_msg_deal(data,length);
        break;
    case COLLECT_PROCESS_READ_CALIBRATION:
        collect_device_calibration_msg_deal(data,length);
        break;
    case COLLECT_PROCESS_UPGRADE:
        break;
    default:
        break;
    }
}

void collect_recive_task_handler(void *pvParameters)
{
    uint8_t rx_data[200] = {0};
    uint16_t len = 0;
    size_t buffered_size = 0;

    while(1)
    {
        uart_event_t event;
        int rx_bytes = 0;
        if(pdTRUE == xQueueReceive(collect_device_queue,&event,portMAX_DELAY))
        {
            switch (event.type)
            {
                case UART_DATA:
                    if(event.timeout_flag == true){
                        uart_get_buffered_data_len(COLLECT_UART_PORT,&buffered_size);
                        while (buffered_size){
                            if(buffered_size > 200){
                                rx_bytes = uart_read_bytes(COLLECT_UART_PORT, rx_data, 200, 10 / portTICK_PERIOD_MS);
                                if(rx_bytes == 0) break;
                                collect_msg_deal(rx_data,rx_bytes);
                                buffered_size -= rx_bytes;
                            }else{
                                rx_bytes = uart_read_bytes(COLLECT_UART_PORT, rx_data, buffered_size, 10 / portTICK_PERIOD_MS);
                                if(rx_bytes == 0) break;
                                collect_msg_deal(rx_data,rx_bytes);
                                buffered_size -= rx_bytes;
                            }
                        }
                        
                    }
                   
                    
                    // modbus_msg_deal_handler(rx_data,rx_bytes);
                    // rx_data[rx_bytes] = 0x00;
                    // ESP_LOGI(TAG,"*********************************************");
                    // ESP_LOGI(TAG,"rs485 接收数量�?%d\r\n",rx_bytes);
                    break;
                default:
                    break;
            }
        }
    }
}












