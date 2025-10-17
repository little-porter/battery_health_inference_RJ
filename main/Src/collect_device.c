#include "collect_device.h"
#include "modbus.h"
#include "sysEvent.h"

static const char *TAG = "PRJ_COLLECT";

/************************************************************************************
*电池信息采集软件bin文件名称定义
************************************************************************************/
static const char *collect_bin_file = "/littlefs/battery_info_collect.bin";                 //电池信息采集软件bin文件存储文件
static const char *collect_cache_bin_file = "/littlefs/battery_info_collect_cache.bin";     //bin文件缓存文件

#define COLLECT_INFO_OFFSET				0x300                                               //软件信息在bin文件中的偏移
/************************************************************************************
*底板串口信息定义
************************************************************************************/
#define COLLECT_UART_PORT     UART_NUM_1                //使用串口1
#define COLLECT_UART_BAUD     9600                      //串口波特率(不能大于9600，隔离电路决定)
#define COLLECT_UART_TX_PIN   GPIO_NUM_43               //GPIO43
#define COLLECT_UART_RX_PIN   GPIO_NUM_44               //GPIO44
// #define COLLECT_EN_PIN        GPIO_NUM_45
QueueHandle_t collect_device_queue = NULL;              //串口事件队列

#define  COLLECT_DEVICE_DATA_SIZE      200              //底板数据帧最大长度
#define  COLLECT_DEVICE_FIFO_SIZE      100              //底板数据缓存FIFO深度

typedef struct _collect_data
{
    uint8_t data[COLLECT_DEVICE_DATA_SIZE];
    uint16_t len;
}collect_data_t;                                        //数据帧结构体
                                      
typedef struct _collect_fifo
{
    collect_data_t data[COLLECT_DEVICE_FIFO_SIZE];
    uint16_t pos;
    uint16_t tail;
}collect_fifo_t;                                       //数据缓存FIFO结构体   
collect_fifo_t collect_rx_fifo;                        //底板数据接收FIFO

/************************************************************************************
*底板数据帧相关宏定义
************************************************************************************/
#define COLLECT_DEVICE_ADDR                 0x01        //底板设备地址
#define COLLECT_DEVICE_BROADCAST_ADDR       0xFF        //广播地址
#define COLLECT_DEVICE_FUNC_READ            0x03        //读寄存器功能码
#define COLLECT_DEVICE_FUNC_WRITE           0x10        //写寄存器功能码
#define COLLECT_DEVICE_ADDR_IDX             0           //设备地址索引        
#define COLLECT_DEVICE_FUNC_IDX             1           //功能码索引 
#define REG_NUM_IDX                         2           //寄存器数量索引
#define REG_DATA_IDX                        4           //寄存器数据索引 

#define COLLECT_DEVICE_RESPOND_TIME         200         //底板响应时间
/************************************************************************************
*modbus寄存器地址宏定义
************************************************************************************/
#define REG_COLLECT_VERSION_YEAR    0x3006              //软件版本年寄存器
#define REG_COLLECT_VERSION_MONTH   0x3007              //软件版本月寄存器
#define REG_COLLECT_VERSION_DAY     0x3008              //软件版本日寄存器
#define REG_COLLECT_VERSION_AA      0x3009              //软件版本AA寄存器
#define REG_COLLECT_VERSION_BB      0x300A              //软件版本BB寄存器
#define REG_COLLECT_VERSION_CC      0x300B              //软件版本CC寄存器

/************************************************************************************
*采集底板信息
************************************************************************************/
typedef enum _collect_process_t
{
    COLLECT_PROCESS_INIT = 0,                           //初始化过程
    COLLECT_PROCESS_GET_VERSION,                        //获取版本信息
    COLLECT_PROCESS_READ_DATA,                          //读取数据
    COLLECT_PROCESS_UPGRADE,                            //软件升级
    COLLECT_PROCESS_SET_CALIBRATION,
    COLLECT_PROCESS_READ_CALIBRATION ,
    COLLECT_PROCESS_SET_BALANCE,
    COLLECT_PROCESS_SET_VOLTAGE_CALIBRATION,
    COLLECT_PROCESS_SET_CO_CALIBRATION,
    COLLECT_PROCESS_SET_H2_CALIBRATION,
    COLLECT_PROCESS_SET_BATTERY_TEMP_CALIBRATION,
    COLLECT_PROCESS_SET_ENV_INFO_CALIBRATION,
    COLLECT_PROCESS_SET_H2_MAP,
}collect_process_t;                                     //采集底板信息交互过程    

typedef struct _version
{
    uint16_t year,month,day;
    uint16_t aa,bb,cc;
}version_t;                                             //软件版本结构体  

version_t collect_version;                              //采集底板软件版本

typedef enum _collect_upgrade_flag
{
    COLLECT_UPGRADE_FLAG_START = 0,
    COLLECT_UPGRADING,
    COLLECT_UPGRADE_FLAG_END,
}collect_upgrade_flag_t;                                //采集底板软件升级过程标志


typedef struct _iap_msg
{
    uint8_t dev_addr;
    uint8_t cmd;
    uint8_t dev_type;
    uint8_t frm_flag;
    uint16_t frm_num;
    uint16_t data_len;
    uint8_t payload[0];
}iap_msg_t;                                             //采集底板iap消息结构体      

#define  collect_data_reverse(pdata,num)      modbus_reg_data_reverse(pdata,num)    //数据反转宏定义

typedef struct _collect_device
{
    uint16_t modebus_addr;
    uint16_t devcie_type;
    uint16_t device_addr;
    uint16_t capacity;
    uint16_t balance;
    uint16_t upgrade_flag;
}collect_device_t;                                              //采集底板设备数据结构体  

collect_process_t collect_process = COLLECT_PROCESS_INIT;       //采集底板交互状态

#define  CALIBRATE_TABLE_VOLTAGE_ADDR   0
#define  CALIBRATE_TABLE_CO_ADDR        4
#define  CALIBRATE_TABLE_H2_ADDR        16  
#define  CALIBRATE_TABLE_BATTERY_TEMP_ADDR   28
#define  CALIBRATE_TABLE_ENV_TEMP_ADDR       32
#define  CALIBRATE_TABLE_ENC_HUMIDITY_ADDR   36   
#define  CALIBRATE_TABLE_LEN        50                          //校准表长度
#define  MAP_TABLE_CO_ADDR          48
#define  MAP_TABLE_H2_ADDR          4
#define  MAP_TABLE_ZERO_ADDR        0
#define  MAP_TABLE_LEN              100
uint16_t collect_calibrate_table[CALIBRATE_TABLE_LEN];          //校准表

uint16_t collect_map[MAP_TABLE_LEN];
collect_device_t collect_device;                                //采集底板设备
bool colDevOnlineStatus = false;                                //采集底板在线标志
uint16_t colDevKeepOnlineTime = 0;                             //采集底板离线时间(单位:秒)


bool collect_device_online_status_get(void)
{
    return colDevOnlineStatus;
}



/************************************************************************************
*采集底板接收fifo相关函数定义
************************************************************************************/
uint16_t collect_device_rx_fifo_data_num(void)
{
    uint16_t data_num = 0;
    if(collect_rx_fifo.tail == collect_rx_fifo.pos){
        return data_num;
    }else if(collect_rx_fifo.tail > collect_rx_fifo.pos){
        data_num = collect_rx_fifo.tail - collect_rx_fifo.pos;
    }else{
        data_num = collect_rx_fifo.tail + COLLECT_DEVICE_FIFO_SIZE - collect_rx_fifo.pos;
    }
    return data_num;
}

void collect_device_clear_rx_fifo(void)
{
    collect_rx_fifo.tail = 0;
    collect_rx_fifo.pos = 0;
}

void collect_device_push_data_to_rx_fifo(uint8_t *data,uint16_t len)
{
    uint16_t write_size = 0;
    if(len > COLLECT_DEVICE_DATA_SIZE){
        write_size = COLLECT_DEVICE_DATA_SIZE;
    }else{
        write_size = len;
    }

    memcpy(collect_rx_fifo.data[collect_rx_fifo.tail].data,data,write_size);
    collect_rx_fifo.data[collect_rx_fifo.tail].len = write_size;
    collect_rx_fifo.tail++;
    collect_rx_fifo.tail %= COLLECT_DEVICE_FIFO_SIZE;
}

uint16_t collect_device_pull_data_from_rx_fifo(uint8_t *data,uint16_t max_len)
{
    uint16_t read_size = 0;
    if(collect_rx_fifo.pos == collect_rx_fifo.tail)  return 0;
    if(max_len < collect_rx_fifo.data[collect_rx_fifo.pos].len){
        read_size = max_len;
    }else{
        read_size = collect_rx_fifo.data[collect_rx_fifo.pos].len;
    }
    memcpy(data,collect_rx_fifo.data[collect_rx_fifo.pos].data,read_size);
    collect_rx_fifo.pos++;
    collect_rx_fifo.pos %= COLLECT_DEVICE_FIFO_SIZE;
    return read_size;
}
/************************************************************************************
*采集底板�?件升级相关函数定�?
************************************************************************************/
void collect_device_read_upgrade_flag(void)
{
    printf("[collect_device] collect device upgrade flag read\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = 0x04;
    uint16_t read_reg_addr = 0x2000,read_reg_num = 1;
    uint8_t data[8] = {0x00};
    uint16_t len = 8;
    data[0] = addr;
    data[1] = cmd;
    data[2] = read_reg_addr>>8;
    data[3] = read_reg_addr&0xff;
    data[4] = read_reg_num>>8;
    data[5] = read_reg_num&0xff;
    uint16_t crc = modbus_calculate_crc(data,len-2);
    data[len-2] = crc&0xff;
    data[len-1] = crc>>8;
    uart_write_bytes(COLLECT_UART_PORT,data,len);
    uart_wait_tx_done(COLLECT_UART_PORT,portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t read_data[100] = {0};
    uint16_t cal_crc = 0;
    uint16_t read_len = collect_device_pull_data_from_rx_fifo(read_data, sizeof(read_data));
    if(read_len>0){
        if(read_len > 6){
            cal_crc = modbus_calculate_crc(read_data,read_len-2);
            crc = read_data[read_len-2] | (read_data[read_len-1]<<8);
        }
        
        if((cal_crc == crc) && (read_data[0] == addr) && (read_data[1] == cmd) && (read_reg_num*2 == (read_data[2]<<8 | read_data[3]))){
            modbus_reg_write_no_reverse(0x200f,&read_data[4],1);            //升级标志
            collect_device.upgrade_flag = read_data[4]<<8|read_data[5];     //升级标志   
            if(collect_device.upgrade_flag == 0x01){
                ESP_LOGE(TAG, "collect device upgrade success");
            }else{
                ESP_LOGE(TAG, "collect device upgrade fail,error code = %d",collect_device.upgrade_flag);
            }
        }else{
            printf("respond msg error,error code = %d\r\n",read_data[1]);
        }
        
        printf("respond crc = %04x , cal_crc = %04x\r\n",crc,cal_crc);
        // printf("collect device upgrade flag read success.\r\n",crc,cal_crc);
    }else{
        ESP_LOGE(TAG, "collect device upgrade flag read fail.");
    }
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
    fseek(file, 0L, SEEK_END);          // 将文件指针移动到文件�?�?
    size_t bin_size = ftell(file);      // 获取文件大小
    rewind(file);                       // 将文件指针移动到文件开�?
    if(bin_size == 0){
        goto OPEN_FAIL;
    }else{;}

    uint8_t *data = heap_caps_malloc(1024,MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    size_t bytes_read = 0;
    size_t send_size = 0;
    uint16_t cal_crc = 0xFFFF;
    uint16_t frm_num = 0;

    //发送升级起始帧(包含bin文件大小)
    collect_device_upgrade_data_send((const char*)&bin_size,sizeof(size_t),frm_num++,COLLECT_UPGRADE_FLAG_START);
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "total File size: %d bytes", bin_size);
    
    //发送升级数�?
    while((bytes_read = fread(data, 1, 1024, file)) > 0){
        collect_device_upgrade_data_send((const char*)data,bytes_read,frm_num++,COLLECT_UPGRADING);
        cal_crc = modbus_calculate_crc_ota(cal_crc,data,bytes_read);
        send_size+=bytes_read;
        vTaskDelay(pdMS_TO_TICKS(300));
        ESP_LOGI(TAG, "sent File size: %d bytes...", send_size);
    }

    //发送升级结束帧(包含bin文件crc)
    collect_device_upgrade_data_send((const char*)&cal_crc,sizeof(uint16_t),frm_num++,COLLECT_UPGRADE_FLAG_END);
    vTaskDelay(pdMS_TO_TICKS(300));
    ESP_LOGI(TAG, "File crc: %04x", cal_crc);

    // collect_device_read_upgrade_flag();             //读取升级标志
    heap_caps_free(data);
    fclose(file);

OPEN_FAIL:
}


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


typedef struct _collect_device_msg_t{
    uint8_t dev_addr;
    uint8_t cmd;
    uint16_t reg_addr;
    uint16_t reg_num;
    uint8_t payload[0];
} collect_device_msg_t;

void collect_device_reg_read(uint16_t reg_addr,uint16_t reg_num)
{
    uint16_t msg_size = sizeof(collect_device_msg_t)+2;
    collect_device_msg_t *msg = (collect_device_msg_t *)heap_caps_malloc(msg_size,MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    
    msg->dev_addr = COLLECT_DEVICE_ADDR;
    msg->cmd = COLLECT_DEVICE_FUNC_READ;
    msg->reg_addr = (reg_addr>>8)|((reg_addr&0xff)<<8);
    msg->reg_num = (reg_num>>8)|((reg_num&0xff)<<8);
    uint16_t crc = modbus_calculate_crc((uint8_t *)msg,msg_size-2);
    msg->payload[0] = crc&0xff;
    msg->payload[1] = crc>>8;
    uart_write_bytes(COLLECT_UART_PORT,msg,msg_size);
    uart_wait_tx_done(COLLECT_UART_PORT,portMAX_DELAY);

    heap_caps_free(msg);
}


void collect_device_reg_write(uint16_t reg_addr,uint16_t reg_num,uint16_t *data)
{ 
    uint16_t msg_size = sizeof(collect_device_msg_t)+2+reg_num*2;
    collect_device_msg_t *msg = (collect_device_msg_t *)heap_caps_malloc(msg_size,MALLOC_CAP_8BIT|MALLOC_CAP_SPIRAM);
    
    msg->dev_addr = COLLECT_DEVICE_ADDR;
    msg->cmd = COLLECT_DEVICE_FUNC_WRITE;
    msg->reg_addr = (reg_addr>>8)|((reg_addr&0xff)<<8);
    msg->reg_num = (reg_num>>8)|((reg_num&0xff)<<8);
    memcpy(msg->payload,data,reg_num*2);
    uint16_t crc = modbus_calculate_crc((uint8_t *)msg,msg_size-2);
    msg->payload[reg_num*2] = crc&0xff;
    msg->payload[reg_num*2+1] = crc>>8;
    uart_write_bytes(COLLECT_UART_PORT,msg,msg_size);
    uart_wait_tx_done(COLLECT_UART_PORT,portMAX_DELAY);

    heap_caps_free(msg);
}

void collect_device_read_reg_respond_anlysis(uint16_t reg_addr,uint16_t reg_num,uint16_t *data)
{
    if(reg_addr>=0x4000 && reg_addr<0x4029){
        ESP_LOGI(TAG, "collect device read calibration success");
        modbus_reg_write_no_reverse(reg_addr,data,reg_num);
        memcpy(&collect_calibrate_table[reg_addr-0x4001],data,reg_num*2);
    }else if(reg_addr>=0x4029 && reg_addr<0x5000){
        ESP_LOGI(TAG, "collect device read map success");
        modbus_reg_write_no_reverse(reg_addr,data,reg_num);
        memcpy(&collect_map[reg_addr-0x4029],data,reg_num*2);
    }else if(reg_addr>=0x1000 && reg_addr<0x2000){
        ESP_LOGI(TAG, "collect device read data success");                                      //复位底板在线时间
        colDevOnlineStatus = true;
        colDevKeepOnlineTime = 10;
        modbus_reg_write_no_reverse(reg_addr,data,reg_num);
        // memcpy(collect_calibrate_table,data,reg_num*2);
    }else if(reg_addr>=0x2000 && reg_addr<0x3000){
        ESP_LOGI(TAG, "collect device read upgrade flag success");
        modbus_reg_write_no_reverse(reg_addr,data,reg_num);
        collect_device.upgrade_flag = (*data&0xFF)<<8|(*data)>>8;     //升级标志   
    }else if(reg_addr>=0x3000 && reg_addr<0x4000){
        ESP_LOGI(TAG, "collect device read version success");
        modbus_reg_write_no_reverse(REG_COLLECT_VERSION_YEAR,data,reg_num);
        version_t version;
        uint16_t *data_ptr = data;
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
    }else{
        ESP_LOGI(TAG, "collect device read reg addr not anlysis,error addr = %04x",reg_addr);
    }
}

void collect_device_write_reg_respond_anlysis(uint16_t reg_addr,uint16_t reg_num)
{
    if(reg_addr < 0x1000 || (reg_addr >= 0x4000 && reg_addr < 0x5000)){
        ESP_LOGI(TAG, "collect device write reg addr success,reg_addr = %04x,reg_num = %d",reg_addr,reg_num);
    }else{
        ESP_LOGE(TAG, "collect device write reg invalid,reg_addr = %04x,reg_num = %d",reg_addr,reg_num);
    }
}

void collect_device_respond_msg_anlysis(uint8_t cmd,uint16_t reg_addr,uint16_t reg_num)
{
    uint8_t read_data[200] = {0};
    uint16_t cal_crc = 0xFFFF,crc = 0;
    uint16_t read_len = collect_device_pull_data_from_rx_fifo(read_data, sizeof(read_data));
    if(read_len>0){
        ESP_LOGI(TAG, "collect device respond,ops reg_addr = %04x,reg_num = %d",reg_addr,reg_num);
        if(read_len > 2){
            cal_crc = modbus_calculate_crc(read_data,read_len-2);
            crc = read_data[read_len-2] | (read_data[read_len-1]<<8);
        }
        if(cal_crc != crc){ 
            ESP_LOGE(TAG, "collect device respond msg  crc error,crc = %d,cal_crc = %d",crc,cal_crc);
            return;
        }else if(read_data[0] != COLLECT_DEVICE_ADDR){
            ESP_LOGE(TAG, "collect device respond addr error,error adrr = %d",read_data[0]);
            return;
        }else if(read_data[1] != cmd){
            if(read_data[1] == cmd+0x80){
                ESP_LOGE(TAG, "collect device error respond,error cmd = %d,error code = %d",read_data[1],read_data[2]); 
            }else{
                ESP_LOGE(TAG, "collect device respond cmd error,error cmd = %d",read_data[1]);
            }
            return;
        }else{;}
        
        if(cmd == COLLECT_DEVICE_FUNC_READ){
            if(reg_num*2 != (read_data[2]<<8 | read_data[3])){
                ESP_LOGE(TAG, "collect device respond read reg num error,read num = %d,data_num = %d",reg_num*2,(read_data[2]<<8 | read_data[3]));
            }else{
                collect_device_read_reg_respond_anlysis(reg_addr,reg_num,&read_data[4]);
            }
            
        }else if(cmd == COLLECT_DEVICE_FUNC_WRITE){
            if(reg_num != (read_data[4]<<8 | read_data[5])){
                ESP_LOGE(TAG, "collect device respond write reg num error,reg_num = %d,write num = %d",reg_num,(read_data[4]<<8 | read_data[5]));
            }else{
                collect_device_write_reg_respond_anlysis(reg_addr,reg_num);
            }
        }else{
            ESP_LOGE(TAG, "collect device ops cmd error,error ops cmd = %d",cmd);
        }
    }else{
        ESP_LOGE(TAG, "collect device not respond,ops reg_addr = %04x,reg_num = %d",reg_addr,reg_num);
    }
}

void collect_device_read_calibration(void)
{
    printf("[collect_device] Read Calibration\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    uint16_t read_reg_addr = 0x4001,read_reg_num = 80;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(500));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);

    printf("[collect_device] Read H2 Map\r\n");
    addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    read_reg_addr = 0x4029,read_reg_num = 49;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(500));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}

void collect_device_read_data(void)
{
    printf("[collect_device] Read Data\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    uint16_t read_reg_addr = 0x1003,read_reg_num = 17;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}

void collect_devicie_read_version(void)
{
    printf("[collect_device] Read Version\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    uint16_t read_reg_addr = 0x3000,read_reg_num = 6;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}
void collect_device_read_config_param(void)
{ 
    printf("[collect_device] Read Config Param\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    uint16_t read_reg_addr = 0x0000,read_reg_num = 5;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}



void collect_device_balance_check(void)
{
    uint16_t balance = 0;
    modbus_reg_read(0x0006,&balance,1);
    if(collect_device.balance != balance){
        sysEvent_set(collect_dev_event_group,COLLECT_DEV_BALANCE_EVENT_BIT);
        collect_device.balance = balance;
    }

}

void collect_device_voltage_calibration_check(void)
{
    uint16_t calibration[4] = {0};
    modbus_reg_read_no_reverse(0x4001,&calibration,4);
    if(0 != memcmp(calibration,&collect_calibrate_table[CALIBRATE_TABLE_VOLTAGE_ADDR],sizeof(calibration))){
        sysEvent_set(collect_dev_event_group,COLLECT_DEV_VOLTAGE_CALIB_EVENT_BIT);
        // modbus_reg_write_no_reverse(0x4001,&collect_calibrate_table[0],4);
        memcpy(&collect_calibrate_table[CALIBRATE_TABLE_VOLTAGE_ADDR],calibration,sizeof(uint16_t)*4);
    }

}
void collect_device_co_calibration_check(void)
{
    uint16_t calibration[12] = {0};
    uint16_t size = sizeof(calibration);
    modbus_reg_read_no_reverse(0x4005,&calibration,12);
    if(0 != memcmp(calibration,&collect_calibrate_table[CALIBRATE_TABLE_CO_ADDR],sizeof(calibration))){
        sysEvent_set(collect_dev_event_group,COLLECT_DEV_CO_CALIB_EVENT_BIT);
        // modbus_reg_write_no_reverse(0x4005,&collect_calibrate_table[4],4);
        memcpy(&collect_calibrate_table[CALIBRATE_TABLE_CO_ADDR],calibration,sizeof(calibration));
    }

}

void collect_device_h2_calibration_check(void)
{
    uint16_t calibration[12] = {0};
    modbus_reg_read_no_reverse(0x4011,&calibration,12);
    if(0 != memcmp(calibration,&collect_calibrate_table[CALIBRATE_TABLE_H2_ADDR],sizeof(calibration))){
        sysEvent_set(collect_dev_event_group,COLLECT_DEV_H2_CALIB_EVENT_BIT);
        // modbus_reg_write_no_reverse(0x4009,&collect_calibrate_table[8],4);
        memcpy(&collect_calibrate_table[CALIBRATE_TABLE_H2_ADDR],calibration,sizeof(calibration));
    }else{;}

}

void collect_device_battery_temperature_calibration_check(void)
{
    uint16_t calibration[4] = {0};
    modbus_reg_read_no_reverse(0x401D,&calibration,sizeof(calibration)/2);
    if(0 != memcmp(calibration,&collect_calibrate_table[CALIBRATE_TABLE_BATTERY_TEMP_ADDR],sizeof(calibration))){
        sysEvent_set(collect_dev_event_group,COLLECT_DEV_BATTERY_TEMP_CALIB_EVENT_BIT);
        memcpy(&collect_calibrate_table[CALIBRATE_TABLE_BATTERY_TEMP_ADDR],calibration,sizeof(calibration));
    }else{;}
}


void collect_device_env_info_calibration_check(void)
{
    uint16_t calibration[8] = {0};
    modbus_reg_read_no_reverse(0x4021,&calibration,sizeof(calibration)/2);
    if(0 != memcmp(calibration,&collect_calibrate_table[CALIBRATE_TABLE_ENV_TEMP_ADDR],sizeof(calibration))){
        sysEvent_set(collect_dev_event_group,COLLECT_DEV_ENV_INFO_CALIB_EVENT_BIT);
        memcpy(&collect_calibrate_table[CALIBRATE_TABLE_ENV_TEMP_ADDR],calibration,sizeof(calibration));
    }else{;}
}

void collect_device_h2_map_check(void)
{
    uint16_t map[50] = {0};
    // printf("[collect_device] H2 MAP SIZE is %d\r\n",sizeof(map));
    modbus_reg_read_no_reverse(0x4029,&map,sizeof(map)/2);
    if(0 != memcmp(map,&collect_map[MAP_TABLE_ZERO_ADDR],sizeof(map))){
        sysEvent_set(collect_dev_event_group,COLLECT_DEV_H2_MAP_SET_EVENT_BIT);
        memcpy(&collect_map[MAP_TABLE_ZERO_ADDR],map,sizeof(map));
    }else{;}
}
    
void collect_device_current_data_updata(void)
{
    int16_t current = 0;
    modbus_reg_read_no_reverse(0x001E,(uint16_t *)&current,1);
    modbus_reg_write_no_reverse(0x1000,(uint16_t *)&current,1);
    collect_device_balance_check();
    collect_device_voltage_calibration_check();
    collect_device_co_calibration_check();
    collect_device_h2_calibration_check();
    collect_device_battery_temperature_calibration_check();
    collect_device_env_info_calibration_check();
    collect_device_h2_map_check();
}


void collect_device_balance_set(void)
{
    printf("[collect_device] Set Balance Mode\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_WRITE;
    uint16_t read_reg_addr = 0x0004,read_reg_num = 1;
    collect_device_clear_rx_fifo();
    collect_device_reg_write(read_reg_addr,read_reg_num,&collect_device.balance);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}
void collect_device_balance_get(void)
{
    printf("[collect_device] Get Balance Mode\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    uint16_t read_reg_addr = 0x0004,read_reg_num = 1;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}


void collect_device_calibration_open(void)
{
    printf("[collect_device] Calibration Open\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_WRITE;
    uint16_t read_reg_addr = 0x4000,read_reg_num = 1;
    uint16_t open_flag = 0x0100;
    collect_device_clear_rx_fifo();
    collect_device_reg_write(read_reg_addr,read_reg_num,&open_flag);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}

void collect_device_calibration_close(void)
{
    printf("[collect_device] Calibration Close\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_WRITE;
    uint16_t read_reg_addr = 0x4000,read_reg_num = 1;
    uint16_t open_flag = 0x0000;
    collect_device_clear_rx_fifo();
    collect_device_reg_write(read_reg_addr,read_reg_num,&open_flag);
    
    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}


void collect_device_voltage_calibration_set(void)
{ 
    collect_device_calibration_open();
    printf("[collect_device] Set Voltage Calibration\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_WRITE;
    uint16_t read_reg_addr = 0x4001,read_reg_num = 4;
    collect_device_clear_rx_fifo();
    collect_device_reg_write(read_reg_addr,read_reg_num,&collect_calibrate_table[0]);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);

    collect_device_calibration_close();
}

void collect_device_voltage_calibration_get(void)
{
    printf("[collect_device] Get Voltage Calibration\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    uint16_t read_reg_addr = 0x4001,read_reg_num = 4;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}

void collect_device_co_calibration_set(void)
{ 
    collect_device_calibration_open();
    printf("[collect_device] Set CO Calibration\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_WRITE;
    uint16_t read_reg_addr = 0x4005,read_reg_num = 12;
    collect_device_clear_rx_fifo();
    collect_device_reg_write(read_reg_addr,read_reg_num,&collect_calibrate_table[CALIBRATE_TABLE_CO_ADDR]);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);

    collect_device_calibration_close();
}
void collect_device_co_calibration_get(void)
{
    printf("[collect_device] Get CO Calibration\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    uint16_t read_reg_addr = 0x4005,read_reg_num = 12;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}
void collect_device_h2_calibration_set(void)
{ 
    collect_device_calibration_open();
    printf("[collect_device] Set H2 Calibration\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_WRITE;
    uint16_t read_reg_addr = 0x4011,read_reg_num = 12;
    collect_device_clear_rx_fifo();
    collect_device_reg_write(read_reg_addr,read_reg_num,&collect_calibrate_table[CALIBRATE_TABLE_H2_ADDR]);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);

    collect_device_calibration_close();
}
void collect_device_h2_calibration_get(void)
{
    printf("[collect_device] Get H2 Calibration\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    uint16_t read_reg_addr = 0x4011,read_reg_num = 12;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}

void collect_device_battery_temperature_calibration_set(void)
{ 
    collect_device_calibration_open();
    printf("[collect_device] Set Battery Temperature Calibration\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_WRITE;
    uint16_t read_reg_addr = 0x401D,read_reg_num = 4;
    collect_device_clear_rx_fifo();
    collect_device_reg_write(read_reg_addr,read_reg_num,&collect_calibrate_table[CALIBRATE_TABLE_BATTERY_TEMP_ADDR]);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);

    collect_device_calibration_close();
}
void collect_device_battery_temperature_calibration_get(void)
{
    printf("[collect_device] Get Battery Temperature Calibration\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    uint16_t read_reg_addr = 0x401D,read_reg_num = 4;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}

void collect_device_env_info_calibration_set(void)
{ 
    collect_device_calibration_open();
    printf("[collect_device] Set Environment Sersor Calibration\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_WRITE;
    uint16_t read_reg_addr = 0x4021,read_reg_num = 8;
    collect_device_clear_rx_fifo();
    collect_device_reg_write(read_reg_addr,read_reg_num,&collect_calibrate_table[CALIBRATE_TABLE_ENV_TEMP_ADDR]);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);

    collect_device_calibration_close();
}


void collect_device_env_info_calibration_get(void)
{
    printf("[collect_device] Get Environment Sersor Calibration\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    uint16_t read_reg_addr = 0x4021,read_reg_num = 8;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(COLLECT_DEVICE_RESPOND_TIME));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}

void collect_device_h2_map_set(void)
{ 
    collect_device_calibration_open();
    printf("[collect_device] Set H2 Sersor Map\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_WRITE;
    uint16_t read_reg_addr = 0x4029,read_reg_num = 50;
    collect_device_clear_rx_fifo();
    collect_device_reg_write(read_reg_addr,read_reg_num,&collect_map[MAP_TABLE_ZERO_ADDR]);

    vTaskDelay(pdMS_TO_TICKS(500));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);

    collect_device_calibration_close();
}

void collect_device_h2_map_get(void)
{
    printf("[collect_device] Get H2 Sersor Map\r\n");
    uint8_t addr = COLLECT_DEVICE_ADDR,cmd = COLLECT_DEVICE_FUNC_READ;
    uint16_t read_reg_addr = 0x4029,read_reg_num = 50;
    collect_device_clear_rx_fifo();
    collect_device_reg_read(read_reg_addr,read_reg_num);

    vTaskDelay(pdMS_TO_TICKS(500));
    collect_device_respond_msg_anlysis(cmd,read_reg_addr,read_reg_num);
}


void collect_device_event_handler(void)
{
    uint32_t collect_event = 0;
    sysEvent_get(collect_dev_event_group,&collect_event);
    if(collect_event&COLLECT_DEV_CALIB_EVENT_BIT){
        sysEvent_clear(collect_dev_event_group,COLLECT_DEV_CALIB_EVENT_BIT);
        collect_process = COLLECT_PROCESS_SET_CALIBRATION;
    }else if(collect_event&COLLECT_DEV_UPGRADE_EVENT_BIT){
        sysEvent_clear(collect_dev_event_group,COLLECT_DEV_UPGRADE_EVENT_BIT);
        collect_process = COLLECT_PROCESS_UPGRADE;
    }else if(collect_event&COLLECT_DEV_BALANCE_EVENT_BIT){
        sysEvent_clear(collect_dev_event_group,COLLECT_DEV_BALANCE_EVENT_BIT);
        collect_process = COLLECT_PROCESS_SET_BALANCE;
    }else if(collect_event&COLLECT_DEV_VOLTAGE_CALIB_EVENT_BIT){
        sysEvent_clear(collect_dev_event_group,COLLECT_DEV_VOLTAGE_CALIB_EVENT_BIT);
        collect_process = COLLECT_PROCESS_SET_VOLTAGE_CALIBRATION;
    }else if(collect_event&COLLECT_DEV_CO_CALIB_EVENT_BIT){
        sysEvent_clear(collect_dev_event_group,COLLECT_DEV_CO_CALIB_EVENT_BIT);
        collect_process = COLLECT_PROCESS_SET_CO_CALIBRATION;
    }else if(collect_event&COLLECT_DEV_H2_CALIB_EVENT_BIT){
        sysEvent_clear(collect_dev_event_group,COLLECT_DEV_H2_CALIB_EVENT_BIT);
        collect_process = COLLECT_PROCESS_SET_H2_CALIBRATION;
    }else if(collect_event&COLLECT_DEV_BATTERY_TEMP_CALIB_EVENT_BIT){
        sysEvent_clear(collect_dev_event_group,COLLECT_DEV_BATTERY_TEMP_CALIB_EVENT_BIT);
        collect_process = COLLECT_PROCESS_SET_BATTERY_TEMP_CALIBRATION;
    }else if(collect_event&COLLECT_DEV_ENV_INFO_CALIB_EVENT_BIT){
        sysEvent_clear(collect_dev_event_group,COLLECT_DEV_ENV_INFO_CALIB_EVENT_BIT);
        collect_process = COLLECT_PROCESS_SET_ENV_INFO_CALIBRATION;
    }else if(collect_event&COLLECT_DEV_H2_MAP_SET_EVENT_BIT){
        sysEvent_clear(collect_dev_event_group,COLLECT_DEV_H2_MAP_SET_EVENT_BIT);
        collect_process = COLLECT_PROCESS_SET_H2_MAP;
    }
    else{;}
}

void collect_device_offline_check(void)
{
    if(colDevKeepOnlineTime > 0){           //保持在线时间自减
        colDevKeepOnlineTime--;
    }else{                                  //保持在线时间为零，判定为离线
        colDevOnlineStatus = false;
        ESP_LOGE(TAG,"[collect_device] Device Offline\r\n");
    }
}

void collect_send_task_handler(void *pvParameters)
{
    uint16_t reply = 0;
    uint8_t cnt = 0;
    while(1){
        cnt++;
        cnt %= 20;
        if(cnt == 0){
            collect_device_offline_check();
        }
       

        collect_device_event_handler();
        switch (collect_process){
        case COLLECT_PROCESS_INIT:
            collect_device_read_calibration();
            collect_process = COLLECT_PROCESS_GET_VERSION;
            break;
        case COLLECT_PROCESS_GET_VERSION:
            collect_devicie_read_version();
            collect_process = COLLECT_PROCESS_READ_DATA;
            break;
        case COLLECT_PROCESS_UPGRADE: 
            collect_device_upgrade();
            collect_process = COLLECT_PROCESS_READ_DATA;
            break;
        case COLLECT_PROCESS_READ_DATA:
            if(cnt == 1){
                collect_device_read_data();
            }
            break;
        case COLLECT_PROCESS_SET_CALIBRATION:
            // collect_device_calibration_set();
            break;
        case COLLECT_PROCESS_READ_CALIBRATION:
            collect_device_read_calibration();
            break;
        case COLLECT_PROCESS_SET_BALANCE:
            collect_device_balance_set();
            collect_device_balance_get();
            collect_process = COLLECT_PROCESS_READ_DATA;
            break;
        case COLLECT_PROCESS_SET_VOLTAGE_CALIBRATION:
            collect_device_voltage_calibration_set();
            collect_device_voltage_calibration_get();
            collect_process = COLLECT_PROCESS_READ_DATA;
            break;
        case COLLECT_PROCESS_SET_CO_CALIBRATION:
            collect_device_co_calibration_set();
            collect_device_co_calibration_get();
            collect_process = COLLECT_PROCESS_READ_DATA;
            break;
        case COLLECT_PROCESS_SET_H2_CALIBRATION:
            collect_device_h2_calibration_set();
            collect_device_h2_calibration_get();
            collect_process = COLLECT_PROCESS_READ_DATA;
            break;
        case COLLECT_PROCESS_SET_BATTERY_TEMP_CALIBRATION:
            collect_device_battery_temperature_calibration_set();
            collect_device_battery_temperature_calibration_get();
            collect_process = COLLECT_PROCESS_READ_DATA;
            break;
        case COLLECT_PROCESS_SET_ENV_INFO_CALIBRATION:
            collect_device_env_info_calibration_set();
            collect_device_env_info_calibration_get();
            collect_process = COLLECT_PROCESS_READ_DATA;
            break;
        case COLLECT_PROCESS_SET_H2_MAP:
            collect_device_h2_map_set();
            collect_device_h2_map_get();
            collect_process = COLLECT_PROCESS_READ_DATA;
            break;
        default:
            break;
        }   
        collect_device_current_data_updata();
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void collect_recive_task_handler(void *pvParameters)
{
    uint8_t rx_data[200] = {0};
    uint16_t len = 0;
    size_t buffered_size = 0;

    while(1){
        uart_event_t event;
        int rx_bytes = 0;
        if(pdTRUE == xQueueReceive(collect_device_queue,&event,portMAX_DELAY)){
            switch (event.type){
                case UART_DATA:
                    if(event.timeout_flag == true){
                        uart_get_buffered_data_len(COLLECT_UART_PORT,&buffered_size);
                        while (buffered_size){
                            if(buffered_size > 200){
                                rx_bytes = uart_read_bytes(COLLECT_UART_PORT, rx_data, 200, 10 / portTICK_PERIOD_MS);
                                if(rx_bytes == 0) break;
                                collect_device_push_data_to_rx_fifo(rx_data,rx_bytes);
                                buffered_size -= rx_bytes;
                            }else{
                                rx_bytes = uart_read_bytes(COLLECT_UART_PORT, rx_data, buffered_size, 10 / portTICK_PERIOD_MS);
                                if(rx_bytes == 0) break;
                                collect_device_push_data_to_rx_fifo(rx_data,rx_bytes);
                                buffered_size -= rx_bytes;
                            }
                        }   
                    }
                    break;
                default:
                    break;
            }
        }
    }
}


void collect_device_init(void)
{
    collect_rx_fifo.pos = 0;
    collect_rx_fifo.tail = 0;
    collect_device_uart_config();
    collect_process = COLLECT_PROCESS_INIT;
    collect_device_bin_soft_version_get();
    xTaskCreatePinnedToCore(collect_send_task_handler,"rs485_send_task",1024*5,NULL,5,NULL,0);
    xTaskCreatePinnedToCore(collect_recive_task_handler,"rs485_recive_task",1024*5,NULL,5,NULL,0);
}









