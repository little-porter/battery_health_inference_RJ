#include "littlefs_ops.h"
#include <dirent.h>
#include "esp_flash.h"

static const char *TAG = "littlefs_ops";

#define LITTLEFS_DIRECTORY  "/littlefs"



void littlefs_ops_test_task(void *pvParameters);



void littlefs_ops_read_file_info(void)
{
    // 打开根目录
    DIR* dir = opendir(LITTLEFS_DIRECTORY);
    if (!dir) {
        ESP_LOGE(TAG, "Failed to open directory");
        return;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        // 输出文件名
        ESP_LOGI(TAG, "Found file: %s", entry->d_name);
    }

    // 关闭目录
    closedir(dir);
}

void littlefs_ops_init(void)
{
    esp_vfs_littlefs_conf_t conf = {
            .base_path = LITTLEFS_DIRECTORY,
            .partition_label = "lfs",
            .format_if_mount_failed = true,
            .dont_mount = false,
        };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            ESP_LOGE(TAG, "Failed to find LittleFS partition");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    littlefs_ops_read_file_info();

    // xTaskCreatePinnedToCore(littlefs_ops_test_task, "littlefs_test_task", 4096, NULL, 5, NULL, 0);
}



char *littlefs_ops_read_file(const char *file_name,littlefs_file_data_t *file_data)
{
    char *file_path = heap_caps_malloc(strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, MALLOC_CAP_SPIRAM);
    snprintf(file_path, strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, "%s/%s", LITTLEFS_DIRECTORY, file_name);
    ESP_LOGI(TAG,"file_path:%s",file_path);
    FILE* file = fopen(file_path, "r");
    if(file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return NULL;
    }
    fseek(file, 0L, SEEK_END);  // 将文件指针移动到文件末尾
    size_t size = ftell(file);
    rewind(file);
    file_data->size = size;
    file_data->data = heap_caps_malloc(size + 1, MALLOC_CAP_SPIRAM);
    memset(file_data->data, 0, size + 1);
    // char *str = fgets(file_data, size+1,     );
    size_t read_num = fread(file_data->data, 1, size, file);
    if(read_num != size)
    {
        ESP_LOGE(TAG, "Failed to read file");
        heap_caps_free(file_path);
        heap_caps_free(file_data->data);
        fclose(file);
        return NULL;
    }
    heap_caps_free(file_path);
    fclose(file);

    return  file_data;
}

bool littlefs_ops_write_file(const char *file_name, const char *file_data,uint32_t data_len)
{ 
    char *file_path = heap_caps_malloc(strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, MALLOC_CAP_SPIRAM);
    memset(file_path, 0, strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2);
    snprintf(file_path, strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, "%s/%s", LITTLEFS_DIRECTORY, file_name);
    ESP_LOGI(TAG,"file_path:%s,data_len:%d",file_path,(int)data_len);
    FILE* file = fopen(file_path, "w");
    if(file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return false;
    }

    size_t size = fwrite(file_data, 1, data_len, file);
    if(size != data_len)
    {
        ESP_LOGE(TAG, "Failed to write to file");
        heap_caps_free(file_path);
        fclose(file);
        return false;
    }

    heap_caps_free(file_path);
    fclose(file);
    return true;
}



void littlefs_test(void)
{
    // littlefs_ops_write_file("test.txt", "Hello two world!", strlen("Hello two world!"));
    littlefs_file_data_t file;
    char *str = littlefs_ops_read_file("template.hex",&file);
    if(str == NULL)
    {
        ESP_LOGI(TAG, "Failed to read file");
    }
    else
    {
        ESP_LOGI(TAG, "Read file len: %"PRIu32, file.size);
        // for(int i = 0; i < file.size; i++)
        // {
        //     // printf("%02x ", file.data[i]);
        //     // vTaskDelay(pdMS_TO_TICKS(2));
        //     ESP_LOGI(TAG, "Read file: %02x", file.data[i]);
        // }
        ESP_LOGI(TAG, "Read file: %s", file.data);
        printf("\r\n");
        
        heap_caps_free(file.data);
    }

    littlefs_ops_read_file_info();
}

void littlefs_ops_test_task(void *pvParameters)
{
    while(1)
    {
        littlefs_test();
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

