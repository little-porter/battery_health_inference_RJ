#include "net_config.h"
#include "esp_mac.h"
#include "littlefs_ops.h"
#include "cJSON.h"

static char *TAG = "net_config";

#define LITTLEFS_PATH     "/littlefs"

char *file = "/littlefs/NetConfig.json";

void net_config_init(void)
{
    uint8_t mac[8] = {0};
    char mac_str[18] = {0};
    char *p = mac_str;
    size_t remaining = sizeof(mac_str);
    esp_efuse_mac_get_default(mac);
    // printf("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("MAC: %s\n", mac);
    for(int i = 0; i < 8; i++)
    {
        int n = snprintf(p,remaining,"%02x ",mac[i]);
        if (n < 0 || (size_t)n >= remaining) { // 检查错误或缓冲区不足
            break;
        }
        p += n;
    }
    printf("MAC: %s\n", mac_str);


    littlefs_file_data_t config_file;
    char *str = littlefs_ops_read_file("NetConfig.json",&config_file);
    if(str == NULL) 
    {
        ESP_LOGI(TAG, "read config.json fail");
        return;
    }

    // 解析 JSON
    cJSON *root = cJSON_Parse(config_file.data);
    if (!root) {
        ESP_LOGE(TAG, "Error parsing JSON,use defualt config!");
        free(config_file.data);
        return;
    }

    ESP_LOGI(TAG, "start config mac!");
    cJSON *config_mac = cJSON_GetObjectItemCaseSensitive(root, "Configs");
    if(config_mac && cJSON_IsArray(config_mac))
    {
        int count = cJSON_GetArraySize(config_mac);
        for(int i=0; i<count; i++)
        {
            cJSON *item = cJSON_GetArrayItem(config_mac, i);
            if(item && cJSON_IsObject(item))
            {
                cJSON *mac_addr = cJSON_GetObjectItemCaseSensitive(item, "mac");
                if(mac_addr && cJSON_IsString(mac_addr))
                {
                    // strncmp(mac_addr->valuestring, mac_str, strlen(mac_addr->valuestring)) == 0
                }
            }
        }
    }
}

