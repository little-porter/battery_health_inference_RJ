#include "modbus.h"
#include "ota.h"
#include "iap.h"
#include "sys.h"
#include "sysEvent.h"

#include "net_config.h"

/*private 文件内部私有*/
#define POLYNOMIAL 0xA001   // Modbus CRC-16 polynomial (低字节优�?)
uint16_t crcTable[256];     // CRC-16 table

#define  REG_TABLE_LEN    150
uint16_t config_reg_table[REG_TABLE_LEN];
uint16_t data_reg_table[REG_TABLE_LEN];
uint16_t result_reg_table[REG_TABLE_LEN];
uint16_t info_reg_table[REG_TABLE_LEN];
uint16_t calibration_reg_table[REG_TABLE_LEN];

#define  MODBUS_ADDR_IDX        0
#define  MODBUS_FUNCODE_IDX     1
#define  REG_START_ADDR_IDX     2
#define  REG_NUM_IDX            4
#define  REG_DATA_IDX           6
#define  MODBUS_CRC_IDX         6

#define  MODBUS_READ_CONFIG_REG_CMD        (uint8_t)0x03
#define  MODBUS_READ_DATA_REG_CMD          (uint8_t)0x04
#define  MODBUS_WRITE_CONFIG_REG_CMD       (uint8_t)0x10
#define  MODBUS_SOFT_UPGRADE_CMD           (uint8_t)0xff

/*public 文件外部接口*/
uint8_t modbus_addr = DEVICE_ID;
uint8_t broadcast_addr = 0xff;

/*生成CRC-16�?*/
void modbus_generate_crcTable(void) {
    uint16_t polynomial = POLYNOMIAL;
    for (int i = 0; i < 256; i++) 
	{
        uint16_t crc = 0;
        uint16_t c = (uint16_t)i;
        for (int j = 0; j < 8; j++) 
		{
            if (((crc ^ c) & 0x0001) != 0)
                crc = (crc >> 1) ^ polynomial;
            else
                crc >>= 1;
            c >>= 1;
        }
        crcTable[i] = crc;
    }

    // config_reg_table[MODBUS_ADDR_IDX] = modbus_addr<<8;
    if(config_reg_table[MODBUS_ADDR_IDX]){
        modbus_addr = config_reg_table[MODBUS_ADDR_IDX]>>8;
    }
    
}

/*计算CRC*/
uint16_t modbus_calculate_crc(uint8_t *data,uint32_t length)
{
    uint16_t crc = 0xFFFF;

    for(uint16_t i = 0; i < length; i++){
        uint16_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ crcTable[index];
    }

    return crc;
}

uint16_t modbus_calculate_crc_ota(uint16_t cal_crc,uint8_t *data,uint32_t length)
{
    uint16_t crc = cal_crc;
    for(uint16_t i = 0; i < length; i++){
        uint16_t index = (crc ^ data[i]) & 0xFF;
        crc = (crc >> 8) ^ crcTable[index];
    }

    return crc;
}

void modbus_reg_data_reverse(uint16_t *reg,uint16_t num)
{
	uint16_t tem = 0;
	for(int i=0; i<num; i++){
		tem = ((*reg>>8)&0xFF) | ((*reg&0xFF)<<8);
		*reg = tem;
		reg++;
	}
}

void modbus_reg_write(uint16_t addr,uint16_t *data,uint16_t num)
{
    if(num ==0 || num > REG_TABLE_LEN) return;
    uint16_t *write_reg = NULL;
    if(addr<0x1000){
        write_reg = &config_reg_table[addr];
    }else if(addr<0x2000){
        write_reg = &data_reg_table[addr-0x1000];
    }else if(addr<0x3000){
        write_reg = &result_reg_table[addr-0x2000];
    }else if(addr<0x4000){
        write_reg = &info_reg_table[addr-0x3000];
    }else if(addr<0x5000){
		write_reg = &calibration_reg_table[addr-0x4000];
	}else{
        /* 非法地址 */
        return;
    }
    memcpy(write_reg,data,num*2); 
	modbus_reg_data_reverse(write_reg,num);	
}

void modbus_reg_write_no_reverse(uint16_t addr,uint16_t *data,uint16_t num)
{
    if(num ==0 || num > REG_TABLE_LEN) return;
    uint16_t *write_reg = NULL;
    if(addr<0x1000){
        write_reg = &config_reg_table[addr];
    }else if(addr<0x2000){
        write_reg = &data_reg_table[addr-0x1000];
    }else if(addr<0x3000){
        write_reg = &result_reg_table[addr-0x2000];
    }else if(addr<0x4000){
        write_reg = &info_reg_table[addr-0x3000];
    }else if(addr<0x5000){
		write_reg = &calibration_reg_table[addr-0x4000];
	}else{
        /* 非法地址 */
        return;
    }
    memcpy(write_reg,data,num*2); 
}

void modbus_reg_read(uint16_t addr,uint16_t *data,uint16_t num)
{
	if(num ==0 || num > REG_TABLE_LEN)	return;
	uint16_t *read_reg = NULL;
	uint16_t temp_reg[REG_TABLE_LEN] = {0};
    if(addr<0x1000){
        read_reg = &config_reg_table[addr];
    }else if(addr<0x2000){
        read_reg = &data_reg_table[addr-0x1000];
    }else if(addr<0x3000){
        read_reg = &result_reg_table[addr-0x2000];
    }else if(addr<0x4000){
        read_reg = &info_reg_table[addr-0x3000];
    }else if(addr<0x5000){
		read_reg = &calibration_reg_table[addr-0x4000];
	}else{
        /* 非法地址 */
        return;
    }
	
	memcpy(temp_reg,read_reg,num*2);
	modbus_reg_data_reverse(temp_reg,num);
	memcpy(data,temp_reg,num*2);
}

void modbus_reg_read_no_reverse(uint16_t addr,uint16_t *data,uint16_t num)
{
	if(num ==0 || num > REG_TABLE_LEN)	return;
	uint16_t *read_reg = NULL;
	uint16_t temp_reg[REG_TABLE_LEN] = {0};
    if(addr<0x1000){
        read_reg = &config_reg_table[addr];
    }else if(addr<0x2000){
        read_reg = &data_reg_table[addr-0x1000];
    }else if(addr<0x3000){
        read_reg = &result_reg_table[addr-0x2000];
    }else if(addr<0x4000){
        read_reg = &info_reg_table[addr-0x3000];
    }else if(addr<0x5000){
		read_reg = &calibration_reg_table[addr-0x4000];
	}else{
        /* 非法地址 */
        return;
    }
	
	memcpy(temp_reg,read_reg,num*2);
	memcpy(data,temp_reg,num*2);
}

void modbus_error_ask(uint8_t cmd,uint8_t error_code)
{
    uint8_t error_msg[256] = {0};
    uint16_t pos = 0,cal_crc = 0;
    error_msg[pos++] = modbus_addr;
    error_msg[pos++] = cmd+0x80;
    error_msg[pos++] = error_code;
    cal_crc = modbus_calculate_crc(error_msg,pos+1);
    error_msg[pos++] = cal_crc>>8&0xFF;
    error_msg[pos++] = cal_crc&0xFF;
    rs485_data_send(error_msg,pos);
}

void modbus_read_ack(uint8_t cmd, uint16_t addr,uint16_t num)
{
    uint8_t ack_msg[256] = {0};
    uint16_t pos = 0,cal_crc = 0;
    uint16_t *ack_reg = NULL;
    static uint16_t heart = 0;
    if(num > REG_TABLE_LEN){
        /* 寄存器数量错�? */
        modbus_error_ask(cmd,0x03);
        return;
    }

    if(addr<0x1000){
        ack_reg = &config_reg_table[addr];
    }else if(addr<0x2000){
        ack_reg = &data_reg_table[addr-0x1000];
        heart++;
        ack_reg[11] = heart>>8 | (heart&0xff)<<8;
    }else if(addr<0x3000){
        ack_reg = &result_reg_table[addr-0x2000];
    }else if(addr<0x4000){
        ack_reg = &info_reg_table[addr-0x3000];
    }else if(addr<0x5000){
		ack_reg = &calibration_reg_table[addr-0x4000];
	}else{
        /* 非法地址 */
        modbus_error_ask(cmd,0x02);
        return;
    }
    ack_msg[pos++] = modbus_addr;
    ack_msg[pos++] = cmd;
    ack_msg[pos++] = num*2>>8;
    ack_msg[pos++] = num*2&0xFF;
    memcpy(&ack_msg[pos],ack_reg,num*2);
    pos += num*2;
    cal_crc = modbus_calculate_crc(ack_msg,pos);
    ack_msg[pos++] = cal_crc&0xFF;
    ack_msg[pos++] = cal_crc>>8&0xFF;
    
    rs485_data_send(ack_msg,pos);
}

void modbus_write_ack(uint8_t cmd, uint16_t addr,uint16_t num,uint8_t *data)
{
    uint8_t ack_msg[256] = {0};
    uint16_t pos = 0,cal_crc = 0;
    uint16_t *ack_reg = NULL;

    if(num > REG_TABLE_LEN){
        /* 寄存器数量错�? */
        modbus_error_ask(cmd,0x03);
        return;
    }


    if(addr<0x1000){
        ack_reg = &config_reg_table[addr];
        sysEvent_set(sys_cfg_event_group,SYS_CFG_SAVE_EVENT_BIT);        //发送保存系统参数事�?
    }else if(addr<0x5000 && addr>=0x4000){
		ack_reg = &calibration_reg_table[addr-0x4000];
	}else{
        /* 非法地址 */
        modbus_error_ask(cmd,0x02);
        return;
    }

    /*校准寄存器�?�问标识判断*/

	// uint16_t open_flag = 0;
	// if(addr == 0x4000){
	// 	open_flag = data[REG_DATA_IDX]<<8 | data[REG_DATA_IDX+1];
	// }else{
	// 	open_flag = ((ack_reg[0]&0xFF)<<8) | ((ack_reg[0]>>8)&0xFF);
	// }
	// if(addr >= 0x4000 && open_flag != 1 && num != 1){
	// 	/* 地址不可访问 */
    //     modbus_error_ask(cmd,0x04);
    //     return;
	// }else 

    memcpy(ack_reg,&data[REG_DATA_IDX],num*2);

    ack_msg[pos++] = modbus_addr;
    ack_msg[pos++] = cmd;
    ack_msg[pos++] = addr>>8;
    ack_msg[pos++] = addr&0xFF;
    ack_msg[pos++] = num>>8;
    ack_msg[pos++] = num&0xFF;
    cal_crc = modbus_calculate_crc(ack_msg,pos);
    ack_msg[pos++] = cal_crc&0xFF;
    ack_msg[pos++] = cal_crc>>8&0xFF;
   

    rs485_data_send(ack_msg,pos);

    // if(open_flag == 1)
	// {
	// 	/*设置校准系数保存标志*/
        
	// }
}

void config_msg_deal_handler(uint8_t *data,uint16_t length);

void modbus_msg_deal_handler(uint8_t *data,uint16_t length)
{
    uint16_t crc=0,cal_crc=0;
    if(length < 4) return;
    uint8_t modbusAddr = data[MODBUS_ADDR_IDX];
    if((modbusAddr != device_cfg.modbus_addr) && (modbusAddr != broadcast_addr)) return;
    crc = data[length-2] | data[length-1]<<8;
    cal_crc = modbus_calculate_crc(data,length-2);
    if(crc != cal_crc) return;
    uint8_t cmd = data[MODBUS_FUNCODE_IDX];
    uint16_t addr = data[REG_START_ADDR_IDX]<<8 | data[REG_START_ADDR_IDX+1];
    uint16_t num = data[REG_NUM_IDX]<<8 | data[REG_NUM_IDX+1];

    // ESP_LOGE("MODBUS", "cmd:%d,modbusAddr:%d,num:%d",cmd,modbusAddr,num);
    // ESP_LOGE("MODBUS", "deviceADDR:%d,broadcast_addr:%d",device_cfg.modbus_addr,broadcast_addr);

    switch (cmd)
    {
    case MODBUS_READ_CONFIG_REG_CMD:
    case MODBUS_READ_DATA_REG_CMD: 
        modbus_read_ack(cmd,addr,num);
        break;
    case MODBUS_WRITE_CONFIG_REG_CMD:
        modbus_write_ack(cmd,addr,num,data);
        break;
    case MODBUS_SOFT_UPGRADE_CMD:
        if(data[2] == 0x01){
            ota_data_deal_handler(data,length);
        }else if(data[2] == 0x02){
            iap_msg_deal_handler(data,length);
        }else if(data[2] == 0xff){
            // config_msg_deal_handler(data,length);
        }else{;}
        
        // ESP_LOGI("OTA", "OTA recive data");
        break;
    default:
        /*功能码错�?*/
        modbus_error_ask(cmd,0x01);
        break;
    }
}






