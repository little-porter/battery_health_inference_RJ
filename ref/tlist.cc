#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <float.h>
#include <string.h>
#include <assert.h>

#define MAX_DATA_LEN 10000
#define MAX_FUTURE_STEPS 1000
#define MIN_SEASONALITY 2
#define MAX_SEASONALITY 24
#define MAX_ITERATIONS 100
#define PARAM_TOLERANCE 1e-6

// 时间序列数据结构
typedef struct {
    double values[MAX_DATA_LEN];
    int length;
} TimeSeries;

// Holt-Winters模型（添加阻尼趋势）
typedef struct {
    double alpha;
    double beta;
    double gamma;
    double phi; // 阻尼系数
    double level;
    double trend;
    double seasonal[MAX_SEASONALITY];
    int seasonality;
    int last_season_index;
    double mse; // 存储模型的MSE
} HoltWintersModel;

// 初始化季节性分量（改进版）
int initialize_seasonal(const TimeSeries* ts, int seasonality, 
                        double* seasonal, double* init_level, double* init_trend) {
    if (seasonality < MIN_SEASONALITY || seasonality > MAX_SEASONALITY) {
        return 0;
    }
    
    int n = ts->length;
    int periods = n / seasonality;
    
    if (periods < 2) {
        return 0;
    }
    
    // 使用动态分配避免栈溢出
    double* period_avgs = (double*)malloc(periods * sizeof(double));
    if (!period_avgs) {
        fprintf(stderr, "内存分配失败\n");
        return 0;
    }
    
    // 1. 计算每个周期的平均值（使用加权移动平均）
    for (int i = 0; i < periods; i++) {
        double sum = 0.0;
        double total_weight = 0.0;
        int start_idx = i * seasonality;
        int end_idx = start_idx + seasonality;
        if (end_idx > n) end_idx = n;
        
        // 计算窗口中心
        double center = (start_idx + end_idx - 1) / 2.0;
        
        for (int j = start_idx; j < end_idx; j++) {
            // 计算归一化距离 (0 在中心, 1 在两端)
            double distance = fabs(j - center);
            double max_distance = (end_idx - start_idx) / 2.0;
            double normalized_dist = (max_distance > 0) ? distance / max_distance : 0;
            
            // 使用升余弦函数计算权重 (中心为1.5，两端为1.0)
            double weight = 1.0 + 0.5 * cos(M_PI * normalized_dist);
            
            sum += ts->values[j] * weight;
            total_weight += weight;
        }
        
        period_avgs[i] = (total_weight > 0) ? sum / total_weight : 0;
    }
    
    // 2. 计算初始趋势（使用线性回归）
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
    for (int i = 0; i < periods; i++) {
        double x = i * seasonality;
        sum_x += x;
        sum_y += period_avgs[i];
        sum_xy += x * period_avgs[i];
        sum_xx += x * x;
    }
    
    double denominator = periods * sum_xx - sum_x * sum_x;
    if (fabs(denominator) < 1e-10) {
        *init_trend = 0.0;
    } else {
        *init_trend = (periods * sum_xy - sum_x * sum_y) / denominator;
    }
    
    // 3. 计算初始水平（使用加权平均）
    *init_level = sum_y / periods - (*init_trend) * (periods - 1) * seasonality / 2.0;
    
    // 4. 计算季节性分量（归一化处理）
    double seasonal_sum = 0.0;
    for (int i = 0; i < seasonality; i++) {
        seasonal[i] = 0.0;
        int count = 0;
        for (int j = 0; j < periods; j++) {
            int idx = j * seasonality + i;
            if (idx >= n) continue;
            // 减去趋势影响
            double detrended = ts->values[idx] - (*init_level) - (*init_trend) * idx;
            seasonal[i] += detrended;
            count++;
        }
        if (count > 0) seasonal[i] /= count;
        seasonal_sum += seasonal[i];
    }
    
    // 归一化：确保季节性分量总和为0
    double seasonal_mean = seasonal_sum / seasonality;
    for (int i = 0; i < seasonality; i++) {
        seasonal[i] -= seasonal_mean;
    }
    
    free(period_avgs);
    return 1;
}

// 计算给定参数下的MSE（内联函数优化性能）
static inline double compute_mse_for_params(
    const TimeSeries* ts, int seasonality, 
    double alpha, double beta, double gamma, double phi,
    double init_level, double init_trend, 
    const double* initial_seasonal) 
{
    int n = ts->length;
    double current_level = init_level;
    double current_trend = init_trend;
    double seasonal[MAX_SEASONALITY];
    memcpy(seasonal, initial_seasonal, seasonality * sizeof(double));
    
    double mse = 0.0;
    int valid_points = 0;
    
    for (int i = seasonality; i < n; i++) {
        int prev_season_index = (i - seasonality) % seasonality;
        if (prev_season_index < 0) prev_season_index += seasonality;
        
        // 预测值
        double forecast = current_level + phi * current_trend + seasonal[prev_season_index];
        double error = ts->values[i] - forecast;
        mse += error * error;
        valid_points++;
        
        // 更新水平、趋势和季节性分量（添加阻尼趋势）
        double new_level = alpha * (ts->values[i] - seasonal[prev_season_index]) 
                        + (1 - alpha) * (current_level + phi * current_trend);
        double new_trend = beta * (new_level - current_level) 
                        + (1 - beta) * (phi * current_trend);
        int season_index = i % seasonality;
        seasonal[season_index] = gamma * (ts->values[i] - new_level) 
                               + (1 - gamma) * seasonal[prev_season_index];
        
        current_level = new_level;
        current_trend = new_trend;
        
        // 检测NaN
        if (isnan(new_level)) break;
    }
    
    return (valid_points > 0) ? mse / valid_points : DBL_MAX;
}

// 网格搜索阶段函数
static void grid_search(
    const TimeSeries* ts, int seasonality,
    double init_level, double init_trend, const double* initial_seasonal,
    double alpha_start, double alpha_end, double alpha_step,
    double beta_start, double beta_end, double beta_step,
    double gamma_start, double gamma_end, double gamma_step,
    double phi_start, double phi_end, double phi_step,
    double* best_mse, double* best_alpha, double* best_beta,
    double* best_gamma, double* best_phi)
{
    int n = ts->length;
    double current_mse;
    
    // 优化搜索顺序：从最有影响的参数开始
    for (double alpha = alpha_start; alpha <= alpha_end; alpha += alpha_step) {
        if (alpha < 0.05 || alpha > 0.95) continue;
        
        for (double gamma = gamma_start; gamma <= gamma_end; gamma += gamma_step) {
            if (gamma < 0.05 || gamma > 0.95) continue;
            
            for (double beta = beta_start; beta <= beta_end; beta += beta_step) {
                if (beta < 0.05 || beta > 0.95) continue;
                
                for (double phi = phi_start; phi <= phi_end; phi += phi_step) {
                    if (phi < 0.7 || phi > 1.0) continue;
                    
                    current_mse = compute_mse_for_params(
                        ts, seasonality, alpha, beta, gamma, phi,
                        init_level, init_trend, initial_seasonal);
                    
                    // 跳过无效结果
                    if (current_mse >= DBL_MAX || isnan(current_mse)) {
                        continue;
                    }
                    
                    // 更新最佳参数
                    if (current_mse < *best_mse) {
                        *best_mse = current_mse;
                        *best_alpha = alpha;
                        *best_beta = beta;
                        *best_gamma = gamma;
                        *best_phi = phi;
                    }
                }
            }
        }
    }
}

// 训练Holt-Winters模型（优化网格搜索）
HoltWintersModel train_holt_winters(const TimeSeries* ts, int seasonality) {
    HoltWintersModel model = {0};
    model.seasonality = seasonality;
    model.mse = DBL_MAX;
    
    if (seasonality < MIN_SEASONALITY || seasonality > MAX_SEASONALITY) {
        fprintf(stderr, "错误: 无效的季节性参数 %d (允许范围: %d-%d)\n", 
                seasonality, MIN_SEASONALITY, MAX_SEASONALITY);
        return model;
    }
    
    if (ts->length < 2 * seasonality) {
        fprintf(stderr, "错误: 数据长度不足 %d (需要至少 %d)\n", 
                ts->length, 2 * seasonality);
        return model;
    }

    double level, trend;
    double initial_seasonal[MAX_SEASONALITY];
    if (!initialize_seasonal(ts, seasonality, initial_seasonal, &level, &trend)) {
        fprintf(stderr, "错误: 季节性初始化失败\n");
        return model;
    }

    double best_alpha = 0.2, best_beta = 0.2, best_gamma = 0.2, best_phi = 0.95;
    double best_mse = DBL_MAX;
    double best_seasonal[MAX_SEASONALITY] = {0};
    double best_level = 0, best_trend = 0;
    int best_last_season_index = 0;
    int n = ts->length;
    
    // 阶段1: 粗粒度搜索
    grid_search(ts, seasonality, level, trend, initial_seasonal,
                0.1, 0.9, 0.1,   // alpha: 0.1-0.9 step 0.1
                0.1, 0.9, 0.1,   // beta: 0.1-0.9 step 0.1
                0.1, 0.9, 0.1,   // gamma: 0.1-0.9 step 0.1
                0.8, 1.0, 0.05,  // phi: 0.8-1.0 step 0.05
                &best_mse, &best_alpha, &best_beta, &best_gamma, &best_phi);
    
     // 阶段2: 细粒度搜索（范围缩小）
    double range = 0.05;
    grid_search(ts, seasonality, level, trend, initial_seasonal,
                fmax(0.05, best_alpha - range), fmin(0.95, best_alpha + range), 0.01,
                fmax(0.05, best_beta - range), fmin(0.95, best_beta + range), 0.01,
                fmax(0.05, best_gamma - range), fmin(0.95, best_gamma + range), 0.01,
                fmax(0.7, best_phi - 0.1), fmin(1.0, best_phi + 0.1), 0.02,
                &best_mse, &best_alpha, &best_beta, &best_gamma, &best_phi);
    
     // 阶段3: 微调（仅优化alpha和gamma）
    double micro_range = 0.01;
    for (double alpha = fmax(0.05, best_alpha - micro_range); 
         alpha <= fmin(0.95, best_alpha + micro_range); 
         alpha += 0.001) 
    {
        for (double gamma = fmax(0.05, best_gamma - micro_range); 
             gamma <= fmin(0.95, best_gamma + micro_range); 
             gamma += 0.001) 
        {
            double current_mse = compute_mse_for_params(
                ts, seasonality, alpha, best_beta, gamma, best_phi,
                level, trend, initial_seasonal);
            
            if (current_mse < best_mse) {
                best_mse = current_mse;
                best_alpha = alpha;
                best_gamma = gamma;
            }
        }
    }
    
    //使用最佳参数重新计算最终模型状态
    double current_level = level;
    double current_trend = trend;
    memcpy(best_seasonal, initial_seasonal, seasonality * sizeof(double));
    
    for (int i = seasonality; i < n; i++) {
        int prev_season_index = (i - seasonality) % seasonality;
        if (prev_season_index < 0) prev_season_index += seasonality;
        
        // 更新水平、趋势和季节性分量
        double new_level = best_alpha * (ts->values[i] - best_seasonal[prev_season_index]) 
                        + (1 - best_alpha) * (current_level + best_phi * current_trend);
        double new_trend = best_beta * (new_level - current_level) 
                        + (1 - best_beta) * (best_phi * current_trend);
        int season_index = i % seasonality;
        best_seasonal[season_index] = best_gamma * (ts->values[i] - new_level) 
                                   + (1 - best_gamma) * best_seasonal[prev_season_index];
        
        current_level = new_level;
        current_trend = new_trend;
    }
    
    // 设置最终模型参数
    model.alpha = best_alpha;
    model.beta = best_beta;
    model.gamma = best_gamma;
    model.phi = best_phi;
    model.level = current_level;
    model.trend = current_trend;
    model.last_season_index = (n - 1) % seasonality;
    model.mse = best_mse;
    memcpy(model.seasonal, best_seasonal, seasonality * sizeof(double));
    
    // 模型验证
    if (isnan(model.level) || isnan(model.trend) || isnan(model.mse)) {
        fprintf(stderr, "警告: 模型包含NaN值\n");
        model.mse = DBL_MAX;
    }
    
    printf("优化参数: alpha=%.4f, beta=%.4f, gamma=%.4f, phi=%.4f, MSE=%.6f\n", 
           model.alpha, model.beta, model.gamma, model.phi, model.mse);
    
    return model;
}

// Holt-Winters预测（改进阻尼趋势实现）
double predict_holt_winters(const HoltWintersModel* model, int step) {
    if (step <= 0) return model->level;
    
    int season_index = (model->last_season_index + step) % model->seasonality;
    if (season_index < 0) season_index += model->seasonality;
    
    // 添加阻尼趋势（优化实现）
    double trend_contribution = 0.0;
    if (fabs(model->phi - 1.0) < 1e-6) {
        // 无阻尼情况
        trend_contribution = model->trend * step;
    } else {
        // 阻尼趋势计算（避免除零错误）
        double phi_step = pow(model->phi, step);
        trend_contribution = model->trend * model->phi * (1 - phi_step) / (1 - model->phi);
    }
    
    double prediction = model->level + trend_contribution + model->seasonal[season_index];
    
    // 合理性检查
    if (isnan(prediction)) {
        fprintf(stderr, "警告: 预测值NaN (step=%d)\n", step);
        return model->level;
    }
    
    return prediction;
}

// 计算RMSE
double calculate_rmse(double mse) {
    return sqrt(fmax(mse, 0));
}

// 生成样本数据（优化实现）
void generate_sample_data(TimeSeries *ts, int pattern_type, double noise_level) {
    if (ts->length <= 0 || ts->length > MAX_DATA_LEN) {
        fprintf(stderr, "错误: 无效的数据长度 %d\n", ts->length);
        return;
    }
    
    srand((unsigned int)time(NULL));
    double value = 1.0;
    int change_point = ts->length / 2;
    int seasonality = 12;
    
    // 预计算三角函数值（使用固定大小数组）
    double sin_values[MAX_SEASONALITY];
    double cos_values[MAX_SEASONALITY];
    for (int i = 0; i < seasonality; i++) {
        sin_values[i] = sin(2 * M_PI * i / seasonality);
        cos_values[i] = cos(4 * M_PI * i / seasonality);
    }

    for (int i = 0; i < ts->length; i++) {
        double noise = ((double)rand() / RAND_MAX * 2.0 - 1.0) * noise_level;
        int season_index = i % seasonality;
        
        switch (pattern_type) {
            case 1: // 线性趋势
                value = 1 + 2.0 * i + noise;
                break;
            case 2: // 季节性模式
                value = 8 + 20 * sin_values[season_index] + noise;
                break;
            case 3: // 指数增长
                value = 1 * exp(0.05 * i) + noise;
                break;
            case 4: // 随机波动
                value = 5 + ((double)rand() / RAND_MAX * 100.0 - 50.0);
                break;
            case 5: // 线性+季节性
                value = 4 + 0.5 * i + 15 * sin_values[season_index] + noise;
                break;
            case 6: // 趋势变化
                if (i < change_point) {
                    value = 1 + 2.0 * i + noise;
                } else {
                    value = 1 + 2.0 * change_point + 0.5 * (i - change_point) + noise;
                }
                break;
            case 7: // 季节性突变
                if (i < change_point) {
                    value = 7 + 10 * sin_values[season_index] + noise;
                } else {
                    int alt_season_index = i % (seasonality/2);
                    value = 7 + 20 * sin(2 * M_PI * alt_season_index / (seasonality/2)) + noise;
                }
                break;
            default: // 复杂混合模式
                value = 4 + 0.8 * i + 20 * sin_values[season_index] + 10 * cos_values[season_index] + noise;
                break;
        }
        ts->values[i] = value;
    }
}

// 打印预测结果（优化输出）
void print_predictions(const TimeSeries* ts, const double* predictions, 
                      int steps, int start_index) {
    printf("\n未来预测结果 (从时间点%d开始):\n", start_index+1);
    printf("%-8s %-12s %-12s %-12s %-12s\n", 
           "时间点", "实际值", "预测值", "绝对误差", "相对误差(%)");
    
    double total_abs_error = 0.0;
    double total_rel_error = 0.0;
    int valid_points = 0;
    
    // 打印历史数据和预测值
    for (int i = 0; i < steps; i++) {
        int idx = start_index + i;
        if (idx >= ts->length) break; // 防止越界
        
        double actual = ts->values[idx];
        double predicted = predictions[i];
        double abs_error = fabs(actual - predicted);
        double rel_error = (actual != 0) ? fabs(actual - predicted) / fabs(actual) * 100 : 0;
        
        printf("%-8d %-12.2f %-12.2f %-12.2f %-12.2f\n", 
               idx + 1, actual, predicted, abs_error, rel_error);
        
        total_abs_error += abs_error;
        total_rel_error += rel_error;
        valid_points++;
    }
    
    if (valid_points > 0) {
        printf("\n平均绝对误差: %.4f\n", total_abs_error / valid_points);
        printf("平均相对误差: %.2f%%\n", total_rel_error / valid_points);
    }
}

// 评估模型在测试集上的性能
double evaluate_on_test_set(const TimeSeries* ts, const HoltWintersModel* model, 
                           int train_size, int forecast_steps) {
    if (ts->length <= train_size) {
        fprintf(stderr, "错误: 训练集大小超过数据长度\n");
        return DBL_MAX;
    }
    
    double total_error = 0.0;
    int test_points = ts->length - train_size;
    
    // 仅评估可用的测试点
    int eval_points = (test_points < forecast_steps) ? test_points : forecast_steps;
    
    // 评估多步预测性能
    for (int i = 0; i < eval_points; i++) {
        double pred = predict_holt_winters(model, i + 1);
        double actual = ts->values[train_size + i];
        double error = actual - pred;
        total_error += error * error;
    }
    
    return (eval_points > 0) ? total_error / eval_points : DBL_MAX;
}

// 滚动预测评估
double rolling_forecast_evaluation(const TimeSeries* ts, const HoltWintersModel* model, 
                                  int train_size, int horizon) {
    double total_error = 0.0;
    int test_points = ts->length - train_size;
    int count = 0;
    
    for (int i = 0; i < test_points - horizon; i++) {
        double horizon_error = 0.0;
        for (int j = 1; j <= horizon; j++) {
            double pred = predict_holt_winters(model, j);
            double actual = ts->values[train_size + i + j - 1];
            horizon_error += fabs(actual - pred);
        }
        total_error += horizon_error / horizon;
        count++;
    }
    
    return (count > 0) ? total_error / count : DBL_MAX;
}

// 主程序
int main() {
    printf("高级时间序列预测系统 (改进Holt-Winters模型)\n");
    printf("=========================================\n\n");
    
    // 1. 配置参数
    int data_pattern = 8;           // 线性+季节性
    double noise_level = 3.0;       // 噪声水平
    int seasonality = 12;           // 季节性长度
    int history_points = 500;       // 历史数据点数
    int forecast_steps =  120;      // 预测步数
    
    // 2. 生成样本数据
    TimeSeries ts;
    ts.length = history_points + forecast_steps;
    
    clock_t start_time = clock();
    generate_sample_data(&ts, data_pattern, noise_level);
    double gen_time = (double)(clock() - start_time) / CLOCKS_PER_SEC;
    
    printf("生成 %d 个数据点 (历史: %d, 测试: %d, 模式: %d, 噪声水平: %.1f) - 耗时: %.4f秒\n", 
           ts.length, history_points, forecast_steps, data_pattern, noise_level, gen_time);
    
    // 3. 划分数据集
    int train_size = history_points;
    TimeSeries train_ts;
    train_ts.length = train_size;
    memcpy(train_ts.values, ts.values, train_size * sizeof(double));
    
    // 4. 训练Holt-Winters模型
    printf("\n训练改进Holt-Winters模型中...\n");
    start_time = clock();
    HoltWintersModel model = train_holt_winters(&train_ts, seasonality);
    double train_time = (double)(clock() - start_time) / CLOCKS_PER_SEC;
    
    if (model.mse >= DBL_MAX || isnan(model.mse)) {
        fprintf(stderr, "错误: Holt-Winters模型训练失败\n");
        return 1;
    }
    
    printf("模型训练完成 - 耗时: %.4f秒\n", train_time);
    printf("最终模型参数: alpha=%.4f, beta=%.4f, gamma=%.4f, phi=%.4f\n",
           model.alpha, model.beta, model.gamma, model.phi);
    
    // // 5. 在测试集上评估模型
    printf("\n在测试集上评估模型...\n");
    start_time = clock();
    double test_mse = evaluate_on_test_set(&ts, &model, train_size, forecast_steps);
    double test_rmse = calculate_rmse(test_mse);
    double rolling_mae = rolling_forecast_evaluation(&ts, &model, train_size, 6);
    double eval_time = (double)(clock() - start_time) / CLOCKS_PER_SEC;
    
    printf("测试集评估结果:\n");
    printf("单步预测MSE:  %.6f\n", test_mse);
    printf("单步预测RMSE: %.6f\n", test_rmse);
    printf("6步滚动预测MAE: %.6f\n", rolling_mae);
    printf("评估耗时: %.4f秒\n", eval_time);
    
    // 6. 预测未来值
    double future[forecast_steps];
    printf("\n预测未来 %d 个时间点...\n", forecast_steps);
    start_time = clock();
    for (int i = 0; i < forecast_steps; i++) {
        future[i] = predict_holt_winters(&model, i + 1);
    }
    double predict_time = (double)(clock() - start_time) / CLOCKS_PER_SEC;
    
    // 7. 打印结果对比 - 从最后一个历史数据点开始
    print_predictions(&ts, future, forecast_steps, history_points);
    
    // printf("\n预测完成! 总耗时: %.4f秒\n", 
    //        gen_time + train_time + eval_time + predict_time);
    return 0;
}