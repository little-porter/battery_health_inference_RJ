#include "littlefs_ops.h"
#include <dirent.h>
#include "esp_flash.h"
#include <unistd.h> // unlink 函数在此头文件中声明

static const char *TAG = "PRJ_LITTLEFS";

#define PRJ_LITTLEFS_LOG_ENABLE      0                     //soh log enable
#if PRJ_LITTLEFS_LOG_ENABLE
#define PRJ_LITTLEFS_PRINTF(x,...)           printf(x,##__VA_ARGS__)
#define PRJ_LITTLEFS_LOGI(format, ...)       ESP_LOGI(TAG,format, ##__VA_ARGS__)
#define PRJ_LITTLEFS_LOGW(format, ...)       ESP_LOGW(TAG,format, ##__VA_ARGS__)
#else
#define PRJ_LITTLEFS_PRINTF(x,...)          
#define PRJ_LITTLEFS_LOGI(format, ...)       
#define PRJ_LITTLEFS_LOGW(format, ...)       
#endif

#define PRJ_LITTLEFS_LOGE(format, ...)       ESP_LOGE(TAG,format, ##__VA_ARGS__)





#define LITTLEFS_DIRECTORY  "/littlefs"



void littlefs_ops_test_task(void *pvParameters);



void littlefs_ops_read_file_info(void)
{
    // 打开根目录
    DIR* dir = opendir(LITTLEFS_DIRECTORY);
    if (!dir) {
        PRJ_LITTLEFS_LOGE("Failed to open directory");
        return;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != NULL) {
        // 输出文件名
        PRJ_LITTLEFS_LOGI("Found file: %s", entry->d_name);
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
            PRJ_LITTLEFS_LOGE("Failed to mount or format filesystem");
        }
        else if (ret == ESP_ERR_NOT_FOUND)
        {
            PRJ_LITTLEFS_LOGE("Failed to find LittleFS partition");
        }
        else
        {
            PRJ_LITTLEFS_LOGE("Failed to initialize LittleFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
    {
        PRJ_LITTLEFS_LOGE("Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    }
    else
    {
        PRJ_LITTLEFS_LOGI("Partition size: total: %d, used: %d", total, used);
    }

    littlefs_ops_read_file_info();

    // xTaskCreatePinnedToCore(littlefs_ops_test_task, "littlefs_test_task", 4096, NULL, 5, NULL, 0);
}



bool littlefs_ops_read_file(const char *file_name,littlefs_file_data_t *file_data)
{
    bool ret = true;
    char *file_path = heap_caps_malloc(strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, MALLOC_CAP_SPIRAM);
    snprintf(file_path, strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, "%s/%s", LITTLEFS_DIRECTORY, file_name);
    PRJ_LITTLEFS_LOGI("file_path:%s",file_path);
    FILE* file = fopen(file_path, "r");
    if(file == NULL)
    {
        PRJ_LITTLEFS_LOGE("Failed to open file for reading");
        ret = false;
        return ret;
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
        PRJ_LITTLEFS_LOGE("Failed to read file");
        heap_caps_free(file_path);
        heap_caps_free(file_data->data);
        fclose(file);
        ret = false;
        return ret;
    }
    heap_caps_free(file_path);
    fclose(file);

    return  ret;
}

bool littlefs_ops_write_file(const char *file_name, const char *file_data,uint32_t data_len)
{ 
    char *file_path = heap_caps_malloc(strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, MALLOC_CAP_SPIRAM);
    memset(file_path, 0, strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2);
    snprintf(file_path, strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, "%s/%s", LITTLEFS_DIRECTORY, file_name);
    PRJ_LITTLEFS_LOGI("file_path:%s,data_len:%d",file_path,(int)data_len);
    FILE* file = fopen(file_path, "w");
    if(file == NULL)
    {
        PRJ_LITTLEFS_LOGE("Failed to open file for writing");
        return false;
    }

    size_t size = fwrite(file_data, 1, data_len, file);
    if(size != data_len)
    {
        PRJ_LITTLEFS_LOGE("Failed to write to file");
        heap_caps_free(file_path);
        fclose(file);
        return false;
    }

    heap_caps_free(file_path);
    fclose(file);
    return true;
}

bool littlefs_ops_write_file_append(const char *file_name, const char *file_data,uint32_t data_len)
{ 
    char *file_path = heap_caps_malloc(strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, MALLOC_CAP_SPIRAM);
    memset(file_path, 0, strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2);
    snprintf(file_path, strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, "%s/%s", LITTLEFS_DIRECTORY, file_name);
    PRJ_LITTLEFS_LOGI("file_path:%s,data_len:%d",file_path,(int)data_len);
    FILE* file = fopen(file_path, "a");
    if(file == NULL)
    {
        PRJ_LITTLEFS_LOGE("Failed to open file for writing");
        return false;
    }

    size_t size = fwrite(file_data, 1, data_len, file);
    if(size != data_len)
    {
        PRJ_LITTLEFS_LOGE("Failed to write to file");
        heap_caps_free(file_path);
        fclose(file);
        return false;
    }

    heap_caps_free(file_path);
    fclose(file);
    return true;
}

bool littlefs_ops_remove_file(const char *file_name)
{ 
    char *file_path = heap_caps_malloc(strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, MALLOC_CAP_SPIRAM);
    memset(file_path, 0, strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2);
    snprintf(file_path, strlen(file_name) + strlen(LITTLEFS_DIRECTORY) + 2, "%s/%s", LITTLEFS_DIRECTORY, file_name);

    if (unlink(file_path) == -1) {
        return false;
    } else {
        printf("File deleted successfully.\n");
        return true;
    }
}

void littlefs_test(void)
{
    // littlefs_ops_write_file("test.txt", "Hello two world!", strlen("Hello two world!"));
    littlefs_file_data_t file;
    // char *str = littlefs_ops_read_file("template.hex",&file);
    char *str = NULL;
    if(str == NULL)
    {
        PRJ_LITTLEFS_LOGI("Failed to read file");
    }
    else
    {
        PRJ_LITTLEFS_LOGI("Read file len: %"PRIu32, file.size);
        // for(int i = 0; i < file.size; i++)
        // {
        //     // printf("%02x ", file.data[i]);
        //     // vTaskDelay(pdMS_TO_TICKS(2));
        //     PRJ_LITTLEFS_LOGI(TAG, "Read file: %02x", file.data[i]);
        // }
        PRJ_LITTLEFS_LOGI("Read file: %s", file.data);
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

