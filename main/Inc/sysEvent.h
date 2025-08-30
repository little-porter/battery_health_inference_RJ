#ifndef __SYSEVENT_H
#define __SYSEVENT_H

#include "sys.h"

/******************************************************************************/
/*系统配置事件*/
/******************************************************************************/
#define SYS_CFG_SAVE_EVENT_BIT      (1<<0)              //保存系统配置
extern EventGroupHandle_t sys_cfg_event_group;

/******************************************************************************/
/*采集底板事件*/
/******************************************************************************/
#define COLLECT_DEV_CALIB_EVENT_BIT       (1<<0)        //底板校准
#define COLLECT_DEV_UPGRADE_EVENT_BIT     (1<<1)        //底板升级
#define COLLECT_DEV_BALANCE_EVENT_BIT     (1<<2)        //底板均衡模式设置
#define COLLECT_DEV_VOLTAGE_CALIB_EVENT_BIT     (1<<3)        //底板电压校准设置
#define COLLECT_DEV_CO_CALIB_EVENT_BIT     (1<<4)        //底板电压校准设置
#define COLLECT_DEV_H2_CALIB_EVENT_BIT     (1<<5)        //底板电压校准设置
extern EventGroupHandle_t collect_dev_event_group;


void sysEvent_init(void);
void sysEvent_set(EventGroupHandle_t event_group,uint32_t event);
void sysEvent_get(EventGroupHandle_t event_group,uint32_t *event);
void sysEvent_clear(EventGroupHandle_t event_group,uint32_t event);
void sysEvent_wait(EventGroupHandle_t event_group,uint32_t event,uint32_t timeout);

#endif

