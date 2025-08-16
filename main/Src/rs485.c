#include "rs485.h"
#include "modbus.h"

static const char *TAG = "PRJ_RS485";

#define RS485_UART_PORT     UART_NUM_2
#define RS485_UART_BAUD     115200
#define RS485_UART_TX_PIN   GPIO_NUM_16
#define RS485_UART_RX_PIN   GPIO_NUM_15
#define RS485_EN_PIN        GPIO_NUM_45


#define rs485_tx_enable()   gpio_set_level(RS485_EN_PIN,1)
#define rs485_tx_disable()  gpio_set_level(RS485_EN_PIN,0)

rs485_driver_t rs485_driver;


void rs485_send_task_handler(void *pvParameters);
void rs485_recive_task_handler(void *pvParameters);

void rs485_uart_init(rs485_driver_t *rs485_drv)
{
    uart_config_t uart_cfg = {
        .baud_rate = RS485_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,       
    };
    uart_param_config(RS485_UART_PORT, &uart_cfg);

    uart_set_pin(RS485_UART_PORT, RS485_UART_TX_PIN, RS485_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    rs485_drv->uart_queue = NULL;
    uart_driver_install(RS485_UART_PORT, 1024*4, 1024*4, 10, &rs485_drv->uart_queue, 0);
    // uart_driver_install(RS485_UART_PORT, 1024, 1024, 0, NULL, 0);

    /*�?2048�?字节才触发UART_DATA事件*/
    // uart_set_rx_full_threshold(RS485_UART_PORT, 2048);

    /*设置超时时间，连�?5�?字�?�没接收到数�?，触发UART_DATA事件 */
    // uart_set_rx_timeout(RS485_UART_PORT, 5);

    //�?�?使能
    // // uart_clear_intr_status(RS485_UART_PORT, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
    // uart_enable_rx_intr(RS485_UART_PORT);
    // // �?用接收超时和接收FIFO满中�?
    // uart_enable_intr_mask(RS485_UART_PORT, UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT);
    // // 设置�?�?处理函数
    // uart_isr_register(RS485_UART_PORT, uart_intr_handler, NULL, ESP_INTR_FLAG_LOWMED, NULL);
    // // 使能UART�?�?
    // uart_int_enable(RS485_UART_PORT);

    if(rs485_drv->uart_queue == NULL)
    {
        ESP_LOGE(TAG, "uart driver install failed");
    }
}


void rs485_uart_recive_intr_handler(void *arg)
{ 

}

void rs485_driver_init(rs485_driver_t *rs485_drv)
{
    memset(rs485_drv,0,sizeof(rs485_driver_t));
    rs485_drv->uart_port =  RS485_UART_PORT;
    rs485_uart_init(rs485_drv);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; // 禁用�?�?
    io_conf.mode = GPIO_MODE_OUTPUT; // 设置为输出模�?
    io_conf.pin_bit_mask = (1ULL << RS485_EN_PIN); // 选择具体的GPIO
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // 禁用下拉电阻
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // 禁用上拉电阻
    gpio_config(&io_conf);

    // xTaskCreatePinnedToCore(rs485_send_task_handler,"rs485_send_task",1024*2,rs485_drv,5,NULL,0);
    xTaskCreatePinnedToCore(rs485_recive_task_handler,"rs485_recive_task",1024*5,rs485_drv,6,NULL,0);

    ESP_LOGI(TAG,"RS485 init success!");
    // uint8_t msg[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x0A ,0x84};

    // rs485_data_send(msg,8);
}

void rs485_push_data_to_tx_fifo(rs485_driver_t *rs485_drv,uint8_t *data,uint16_t len)
{
    memcpy(rs485_drv->tx_fifo.data[rs485_drv->tx_fifo.tail],data,len);
    rs485_drv->tx_fifo.len[rs485_drv->tx_fifo.tail] = len;
    rs485_drv->tx_fifo.tail++;
    rs485_drv->tx_fifo.tail %= RS485_FIFO_NUM;
}

void rs485_data_send(uint8_t *data,uint16_t len)
{
    rs485_tx_enable();
    uart_write_bytes(rs485_driver.uart_port,data,len);
    uart_wait_tx_done(rs485_driver.uart_port,portMAX_DELAY);
    rs485_tx_disable();
}


void rs485_send_task_handler(void *pvParameters)
{
    rs485_driver_t *rs485_drv = (rs485_driver_t *)pvParameters;
    while (1)
    {
        while(rs485_drv->tx_fifo.pos != rs485_drv->tx_fifo.tail)
        {
            rs485_tx_enable();
            uart_write_bytes(rs485_drv->uart_port,rs485_drv->tx_fifo.data[rs485_drv->tx_fifo.pos],rs485_drv->tx_fifo.len[rs485_drv->tx_fifo.pos]);
            uart_wait_tx_done(rs485_drv->uart_port,portMAX_DELAY);
            rs485_drv->tx_fifo.pos++;
            rs485_drv->tx_fifo.pos %= RS485_FIFO_NUM;
            rs485_tx_disable();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
}

void rs485_recive_task_handler(void *pvParameters)
{
    rs485_driver_t *rs485_drv = (rs485_driver_t *)pvParameters;
    uint8_t rx_data[2048];
#ifdef USE_BMS_DATA
    bms_msg_t bms_msg;
#endif
    while(1)
    {
        uart_event_t event;
        int rx_bytes = 0;
        size_t buffered_size = 0;
        // rx_bytes = uart_read_bytes(rs485_drv->uart_port, rx_data, 2408, pdMS_TO_TICKS(150));
        // if(0 == rx_bytes)  continue;
        // modbus_msg_deal_handler(rx_data,rx_bytes);

        if(pdTRUE == xQueueReceive(rs485_drv->uart_queue,&event,portMAX_DELAY))
        {
            switch (event.type)
            {
                case UART_DATA:
                    if(event.timeout_flag == true){
                        uart_get_buffered_data_len(rs485_drv->uart_port,&buffered_size);
                        // if(buffered_size <= 5) continue;
                        while(buffered_size){
                            if(buffered_size > 2048){
                                rx_bytes = uart_read_bytes(rs485_drv->uart_port, rx_data, 2048, portMAX_DELAY);
                                if(0 == rx_bytes)  break;
                                modbus_msg_deal_handler(rx_data,rx_bytes);
                                buffered_size -= rx_bytes;
                            }else{
                                rx_bytes = uart_read_bytes(rs485_drv->uart_port, rx_data, buffered_size, portMAX_DELAY);
                                if(0 == rx_bytes)  break;
                                modbus_msg_deal_handler(rx_data,rx_bytes);
                                buffered_size -= rx_bytes;
                            }
                        }  
                    }

                break;
                default:
                break;
            }  
        }

        // vTaskDelay(pdMS_TO_TICKS(20));
    }
}



