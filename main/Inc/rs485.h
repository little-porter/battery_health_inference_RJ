#ifndef __RS485_H__
#define __RS485_H__

// #include "project_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"


#include "driver/uart.h"


#define RS485_FIFO_NUM          20

typedef struct _rs485_fifo
{
    /* data */
    uint8_t data[RS485_FIFO_NUM][200];
    uint16_t len[RS485_FIFO_NUM];
    uint16_t pos;
    uint16_t tail;
}rs485_fifo_t;


typedef struct _rs485_driver
{
    /* data */
    uart_port_t uart_port;
    rs485_fifo_t rx_fifo;
    rs485_fifo_t tx_fifo;

    QueueHandle_t uart_queue;
}rs485_driver_t;

extern rs485_driver_t rs485_driver;

void rs485_driver_init(rs485_driver_t *rs485_drv);
void rs485_push_data_to_tx_fifo(rs485_driver_t *rs485_drv,uint8_t *data,uint16_t len);


#endif
