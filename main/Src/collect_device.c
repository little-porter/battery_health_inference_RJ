#include "collect_device.h"
#include "modbus.h"



static const char *TAG = "PRJ_COLLECT";

#define COLLECT_UART_PORT     UART_NUM_1
#define COLLECT_UART_BAUD     115200
#define COLLECT_UART_TX_PIN   GPIO_NUM_43
#define COLLECT_UART_RX_PIN   GPIO_NUM_44
// #define COLLECT_EN_PIN        GPIO_NUM_45

#define COLLECT_DEVICE_ADDR         0x01
#define COLLECT_DEVICE_ADDR_IDX     0
#define COLLECT_DEVICE_FUNC_IDX     1
#define REG_NUM_IDX                 2
#define REG_DATA_IDX                4

QueueHandle_t collect_device_queue = NULL;

typedef enum _collect_process_t
{
    COLLECT_PROCESS_READ_DATA = 0,
    COLLECT_PROCESS_CALIBRATION = 1,
    COLLECT_PROCESS_READ_CALIBRATION = 2,
}collect_process_t;

collect_process_t collect_process = COLLECT_PROCESS_READ_DATA;


void collect_send_task_handler(void *pvParameters);
void collect_recive_task_handler(void *pvParameters);


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

    uart_driver_install(COLLECT_UART_PORT, 1024, 1024, 10, &collect_device_queue, 0);
}


void collect_device_init(void)
{
    collect_device_uart_config();
    collect_process = COLLECT_PROCESS_READ_CALIBRATION;
    xTaskCreatePinnedToCore(collect_send_task_handler,"rs485_send_task",1024*2,NULL,5,NULL,0);
    xTaskCreatePinnedToCore(collect_recive_task_handler,"rs485_recive_task",1024*5,NULL,5,NULL,0);
}

typedef union _u_float_u8
{
    /* data */
    float f_data;
    uint8_t u8_data[4];
}u_float_u8_t;


void collect_device_read_data(void)
{
    printf("[collect_device] collect_device_read_data\r\n");
    uint8_t data[8] = {0x01, 0x03, 0x10, 0x00, 0x00, 0x0B, 0x00, 0xCD};
    uint16_t len = 8;

    uart_write_bytes(COLLECT_UART_PORT,data,len);
    uart_wait_tx_done(COLLECT_UART_PORT,portMAX_DELAY);
}

void collect_device_calibrate(void)
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


void collect_send_task_handler(void *pvParameters)
{
    uint16_t reply = 0;
    while(1)
    {
        switch (collect_process)
        {
        case COLLECT_PROCESS_READ_DATA:
            if(collect_calibrate_flag == 1){
                collect_calibrate_flag = 0;
                collect_process = COLLECT_PROCESS_CALIBRATION;
            }
            collect_device_read_data();
            break;
        case COLLECT_PROCESS_CALIBRATION:
            collect_device_calibrate();
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


void collect_msg_deal(uint8_t *data,uint16_t length)
{
    uint16_t crc=0,cal_crc=0;
    if(data[COLLECT_DEVICE_ADDR_IDX] != COLLECT_DEVICE_ADDR) return;
    crc = data[length-2] | data[length-1]<<8;
    cal_crc = modbus_calculate_crc(data,length-2);
    if(crc != cal_crc) return;

    printf("data crc = %04x , cal_crc = %04x\r\n",crc,cal_crc);
   
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

void collect_device_calibrate_deal(uint8_t *data,uint16_t length)
{
    uint16_t crc=0,cal_crc=0;
    if(data[COLLECT_DEVICE_ADDR_IDX] != COLLECT_DEVICE_ADDR) return;
    crc = data[length-2] | data[length-1]<<8;
    cal_crc = modbus_calculate_crc(data,length-2);
    if(crc != cal_crc) return;

    printf("calibration set crc = %04x , cal_crc = %04x\r\n",crc,cal_crc);
   
    collect_process = COLLECT_PROCESS_READ_CALIBRATION;
}

void collect_device_calibration_deal(uint8_t *data,uint16_t length)
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

void collect_recive_task_handler(void *pvParameters)
{
    uint8_t rx_data[200] = {0};
    uint16_t len = 0;

    while(1)
    {
        uart_event_t event;
        int rx_bytes = 0;
        if(pdTRUE == xQueueReceive(collect_device_queue,&event,portMAX_DELAY))
        {
            switch (event.type)
            {
                case UART_DATA:
                    rx_bytes = uart_read_bytes(COLLECT_UART_PORT, rx_data, event.size, 100 / portTICK_PERIOD_MS);
                    switch (rx_data[COLLECT_DEVICE_FUNC_IDX])
                    {
                    case 0x03:
                        collect_msg_deal(rx_data,rx_bytes);
                        break;
                    case 0x10:
                        collect_device_calibrate_deal(rx_data,rx_bytes);
                        break;
                    case 0x04:
                        collect_device_calibration_deal(rx_data,rx_bytes);
                        break;
                    default:
                        break;
                    }
                    
                    // modbus_msg_deal_handler(rx_data,rx_bytes);
                    // rx_data[rx_bytes] = 0x00;
                    // ESP_LOGI(TAG,"*********************************************");
                    // ESP_LOGI(TAG,"rs485 接收数量：%d\r\n",rx_bytes);
                    break;
                default:
                    break;
            }
        }
    }
}








