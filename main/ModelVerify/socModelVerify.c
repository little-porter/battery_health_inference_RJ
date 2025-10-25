#include "socModelVerify.h"
#include "soc.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static char *TAG = "SOC_VERIFY";
static char *socTestFile = "/littlefs/socModelTestData.csv";


float socInputData[5] = {0};
// uint64_t index = 0;


void socVerity_inference_task_handler(void *parameters)
{ 
    while(1){
        soc_inference_task_handler(NULL);
        // soh_inference_task_handler(NULL);
        vTaskDelay(pdMS_TO_TICKS(10*1000));
    }
}
bool parse_float(const char *str, float *value) {
    if (str == NULL || strlen(str) == 0) {
        return false;
    }

    char *endptr;
    double val = strtod(str, &endptr);

    // 检查是否完全解析
    if (endptr == str || *endptr != '\0') {
        return false;
    }

    // 检查 NaN 或 Inf
    if (isnan(val) || isinf(val)) {
        return false;
    }

    *value = (float)val;
    return true;
}

bool socVerity_data_get_by_row(float *inputdata, int target_row)
{
    // 参数检查
    if (target_row < 1) {
        ESP_LOGE(TAG, "Invalid row number: %d", target_row);
        return false;
    }

    FILE* file = fopen(socTestFile, "r");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", socTestFile);
        return false;
    }

    char line[256];
    int current_row = 0;  // 当前行号

    // === 跳过标题行 ===
    if (fgets(line, sizeof(line), file)) {
        // current_row++;  // 标题是第1行
        ESP_LOGW(TAG, "Header: %s", line);
    } else {
        ESP_LOGE(TAG, "File is empty or no header");
        fclose(file);
        return false;
    }

    // === 跳过前 target_row-1 行数据 ===
    while (current_row < target_row) {
        if (fgets(line, sizeof(line), file) == NULL) {
            ESP_LOGE(TAG, "Row %d does not exist. File has fewer than %d data rows.", target_row, target_row);
            fclose(file);
            return false;
        }
        current_row++;
    }

    // === 现在 current_row == target_row，开始解析目标行 ===
    ESP_LOGI(TAG, "Reading row %d", target_row);

    // 初始化输入数组
    for (int i = 0; i < 5; i++) {
        inputdata[i] = 0.0f;
    }

    // 去除 \r\n
    int len = strlen(line);
    if (len > 0) {
        if (line[len - 1] == '\n') {
            line[len - 1] = '\0';
            len--;
        }
        if (len > 0 && line[len - 1] == '\r') {
            line[len - 1] = '\0';
        }
    }

    // 复制一份用于 strtok
    char *line_copy = strdup(line);
    if (!line_copy) {
        ESP_LOGE(TAG, "strdup failed");
        fclose(file);
        return false;
    }

    char *token = strtok(line_copy, ",");
    int col_index = 0;

    while (token != NULL && col_index < 5) {
        float f_val;
        bool is_float = parse_float(token, &f_val);

        if (is_float) {
            inputdata[col_index] = f_val;
            ESP_LOGI(TAG, "Col %d: '%s' -> %.6f", col_index, token, f_val);
        } else {
            inputdata[col_index] = 0.0f;
            ESP_LOGW(TAG, "Col %d: '%s' -> Invalid float", col_index, token);
        }

        col_index++;
        token = strtok(NULL, ",");
    }

    free(line_copy);
    fclose(file);

    // 打印最终结果
    ESP_LOGE(TAG, "Target Row %d -> Step_Index: %.3f, Voltage: %.3f, Test_Time: %.3f, SoC: %.3f, Q: %.3f",
             target_row,
             inputdata[0], inputdata[1], inputdata[2], inputdata[3], inputdata[4]);

    // 填充数据（根据你的业务逻辑）
    soc_input_data_fill(inputdata, 5);

    return true;
}

bool socVerity_data_get_soc(float *inputdata, int target_row)
{
    // 参数检查
    if (target_row < 1) {
        ESP_LOGE(TAG, "Invalid row number: %d", target_row);
        return false;
    }

    FILE* file = fopen(socTestFile, "r");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", socTestFile);
        return false;
    }

    char line[256];
    int current_row = 0;  // 当前行号

    // === 跳过标题行 ===
    if (fgets(line, sizeof(line), file)) {
        // current_row++;  // 标题是第1行
        ESP_LOGW(TAG, "Header: %s", line);
    } else {
        ESP_LOGE(TAG, "File is empty or no header");
        fclose(file);
        return false;
    }

    // === 跳过前 target_row-1 行数据 ===
    while (current_row < target_row) {
        if (fgets(line, sizeof(line), file) == NULL) {
            ESP_LOGE(TAG, "Row %d does not exist. File has fewer than %d data rows.", target_row, target_row);
            fclose(file);
            return false;
        }
        current_row++;
    }

    // === 现在 current_row == target_row，开始解析目标行 ===
    ESP_LOGI(TAG, "Reading row %d", target_row);

    // 去除 \r\n
    int len = strlen(line);
    if (len > 0) {
        if (line[len - 1] == '\n') {
            line[len - 1] = '\0';
            len--;
        }
        if (len > 0 && line[len - 1] == '\r') {
            line[len - 1] = '\0';
        }
    }

    // 复制一份用于 strtok
    char *line_copy = strdup(line);
    if (!line_copy) {
        ESP_LOGE(TAG, "strdup failed");
        fclose(file);
        return false;
    }

    char *token = strtok(line_copy, ",");
    int col_index = 0;

    while (token != NULL && col_index < 5) {
        float f_val;
        bool is_float = parse_float(token, &f_val);

        if (is_float && col_index == 3) {
            *inputdata = f_val;
        } else {
           
        }

        col_index++;
        token = strtok(NULL, ",");
    }

    free(line_copy);
    fclose(file);

    return true;
}

void socVerity_data_get_task_handler(void *pvParameters)
{
    uint64_t index = 0;
    while(1){
        uint64_t pre_index_1h = index+20;
        uint64_t pre_index_2h = index+40;
        float pre_soc_1h=0,pre_soc_2h=0;

        socVerity_data_get_by_row(socInputData,index++);
        socVerity_data_get_soc(&pre_soc_1h,pre_index_1h);
        socVerity_data_get_soc(&pre_soc_2h,pre_index_2h);
        
        soc_inference_task_handler(NULL);
        printf("index: %d\n",(int)index);
        printf("pre_soc_1h: %.3f, pre_soc_2h: %.3f\n",pre_soc_1h,pre_soc_2h);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } 
}


void socModelVerify_init(void)
{
    FILE* file = fopen(socTestFile, "r");
    ESP_LOGI(TAG, "***************************************************************************");
    if(file == NULL){
        ESP_LOGE(TAG, "open socModelTestData.csv error");
        return;
    }

    char line[256];
    int current_row = 0;  // 行计数器

    // 读取标题行并跳过
    if (fgets(line, sizeof(line), file)) {
        current_row++;
        ESP_LOGW(TAG, "Header: %s", line);
    }
    ESP_LOGI(TAG, "***************************************************************************");
    fclose(file);

    //创建电池数据更新任务
    xTaskCreatePinnedToCore(socVerity_data_get_task_handler,"VerGetDataTask",1024*5,NULL,6,NULL,1);
    //创建推理任务
    // xTaskCreatePinnedToCore(socVerity_inference_task_handler,"VerInfTask",1024*5,NULL,6,NULL,1);

}


