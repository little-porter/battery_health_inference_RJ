#include "sysEvent.h"


EventGroupHandle_t sys_cfg_event_group;
EventGroupHandle_t collect_dev_event_group;


void sysEvent_init(void)
{
    sys_cfg_event_group = xEventGroupCreate();
    collect_dev_event_group = xEventGroupCreate();
}


void sysEvent_set(EventGroupHandle_t event_group,uint32_t event)
{
    xEventGroupSetBits(event_group, event);
}
void sysEvent_get(EventGroupHandle_t event_group,uint32_t *event)
{
    *event = xEventGroupGetBits(event_group);
}
void sysEvent_clear(EventGroupHandle_t event_group,uint32_t event)
{
    xEventGroupClearBits(event_group, event);
}
void sysEvent_wait(EventGroupHandle_t event_group,uint32_t event,uint32_t timeout)
{
    xEventGroupWaitBits(event_group, event, pdTRUE, pdFALSE, timeout);
}



