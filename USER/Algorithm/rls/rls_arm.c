#include "rls_arm.h"
#include "bsp_log.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

// 内部辅助函数声明
static arm_status rls_arm_validate_params(const rls_arm_instance_f32 *S);
static void rls_arm_cleanup_memory(rls_arm_instance_f32 *S);
static arm_status rls_arm_allocate_memory(rls_arm_instance_f32 *S);
static arm_status rls_arm_update_core(rls_arm_instance_f32 *S, 
                                     const float32_t *p_input, 
                                     float32_t reference, 
                                     float32_t *p_output, 
                                     float32_t *p_error);
static arm_status rls_arm_update_direct(rls_arm_instance_f32 *S,
                                       const float32_t *p_input_vec,
                                       float32_t reference,
                                       float32_t *p_output,
                                       float32_t *p_error);

/**
 * @brief 参数验证函数
 */
static arm_status rls_arm_validate_params(const rls_arm_instance_f32 *S)
{
    if (S == NULL)
    {
        LOGERROR("RLS_ARM: Invalid instance pointer");
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (!S->is_initialized)
    {
        LOGERROR("RLS_ARM: Instance not initialized");
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (S->num_taps == 0 || S->num_taps > 1000)
    {
        LOGERROR("RLS_ARM: Invalid num_taps (%d)", S->num_taps);
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (S->lambda <= 0.0f || S->lambda > 1.0f)
    {
        LOGERROR("RLS_ARM: Invalid lambda (%.6f)", S->lambda);
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (S->delta <= 0.0f)
    {
        LOGERROR("RLS_ARM: Invalid delta (%.6f)", S->delta);
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    return ARM_MATH_SUCCESS;
}

/**
 * @brief 内存清理函数
 */
static void rls_arm_cleanup_memory(rls_arm_instance_f32 *S)
{
    if (S == NULL) return;
    
    if (S->p_data_buffer)
    {
        free(S->p_data_buffer);
        S->p_data_buffer = NULL;
    }
    
    // 重置所有指针
    S->p_state = NULL;
    S->p_mat_p_data = NULL;
    S->p_mat_w_data = NULL;
    S->p_mat_x_data = NULL;
    S->p_mat_k_data = NULL;
    S->p_temp_vec_data = NULL;
    S->p_temp_mat_data = NULL;
}

/**
 * @brief 内存分配函数
 */
static arm_status rls_arm_allocate_memory(rls_arm_instance_f32 *S)
{
    uint16_t num_taps = S->num_taps;
    uint32_t total_size = 0;
    
    // 计算所需内存大小
    uint32_t state_size = num_taps * sizeof(float32_t);
    uint32_t mat_p_size = num_taps * num_taps * sizeof(float32_t);
    uint32_t mat_w_size = num_taps * sizeof(float32_t);
    uint32_t mat_x_size = num_taps * sizeof(float32_t);
    uint32_t mat_k_size = num_taps * sizeof(float32_t);
    uint32_t temp_vec_size = num_taps * sizeof(float32_t);
    uint32_t temp_mat_size = num_taps * num_taps * sizeof(float32_t);
    
    total_size = state_size + mat_p_size + mat_w_size + mat_x_size + 
                 mat_k_size + temp_vec_size + temp_mat_size;
    
    // 分配连续内存块 (确保4字节对齐)
    S->p_data_buffer = (float32_t *)malloc(total_size);
    if (!S->p_data_buffer)
    {
        LOGERROR("RLS_ARM: Memory allocation failed");
        return ARM_MATH_LENGTH_ERROR;
    }
    
    // 检查内存对齐（ARM Cortex-M4要求4字节对齐）
    if ((uintptr_t)S->p_data_buffer & 0x3)
    {
        LOGERROR("RLS_ARM: Memory not 4-byte aligned, performance may be affected");
    }
    
    // 设置各数据区指针
    float32_t *ptr = S->p_data_buffer;
    S->p_state = ptr;
    ptr += num_taps;
    
    S->p_mat_p_data = ptr;
    ptr += num_taps * num_taps;
    
    S->p_mat_w_data = ptr;
    ptr += num_taps;
    
    S->p_mat_x_data = ptr;
    ptr += num_taps;
    
    S->p_mat_k_data = ptr;
    ptr += num_taps;
    
    S->p_temp_vec_data = ptr;
    ptr += num_taps;
    
    S->p_temp_mat_data = ptr;
    
    return ARM_MATH_SUCCESS;
}

/**
 * @brief RLS核心更新算法
 */
float32_t p_mat_w_data_look;
static arm_status rls_arm_update_core(rls_arm_instance_f32 *S, 
                                     const float32_t *p_input, 
                                     float32_t reference, 
                                     float32_t *p_output, 
                                     float32_t *p_error)
{
    arm_status status;
    uint16_t num_taps = S->num_taps;
    
    // 1. 更新输入向量 (滑动窗口)
    memmove(&S->p_state[1], &S->p_state[0], (num_taps - 1) * sizeof(float32_t));
    S->p_state[0] = *p_input;
    
    // 2. 计算滤波器输出: y = x^T * w
    float32_t output = 0.0f;
    arm_dot_prod_f32(S->p_state, S->p_mat_w_data, num_taps, &output);
    *p_output = output;
    
    // 3. 计算误差: e = d - y
    float32_t error = reference - output;
    *p_error = error;
    
    // 4. 计算增益向量: k = P * x / (lambda + x^T * P * x)
    // 计算 P * x
    status = arm_mat_mult_f32(&S->P, &S->x, &S->temp_vec);
    if (status != ARM_MATH_SUCCESS)
    {
        LOGERROR("RLS_ARM: Matrix multiplication failed");
        return status;
    }
    
    // 计算 x^T * P * x
    float32_t k_den = 0.0f;
    arm_dot_prod_f32(S->p_state, S->p_temp_vec_data, num_taps, &k_den);
    k_den += S->lambda;
    
    // 数值稳定性检查
    if (k_den < 1e-8f)
    {
        LOGERROR("RLS_ARM: k_den too small (%.2e), using regularization", k_den);
        k_den = 1e-8f;
    }
    
    // 计算增益向量 k = (P * x) / (lambda + x^T * P * x)
    arm_scale_f32(S->p_temp_vec_data, 1.0f / k_den, S->p_mat_k_data, num_taps);
    
    // 5. 更新权重: w = w + k * e
    arm_scale_f32(S->p_mat_k_data, error, S->p_temp_vec_data, num_taps);
    arm_mat_add_f32(&S->w, &S->temp_vec, &S->w);
    
    // 6. 更新协方差矩阵: P = (1/lambda) * (P - k * x^T * P)
    //    其中 temp_vec = P * x (已计算)
    // 计算 k * x^T * P = k * (P*x)^T (外积运算)
    // 为了保持数值稳定性，使用对称化的外积计算
    for (uint16_t row = 0; row < num_taps; row++)
    {
        float32_t k_row = S->p_mat_k_data[row];
        for (uint16_t col = row; col < num_taps; col++)  // 只计算上三角
        {
            float32_t val = k_row * S->p_temp_vec_data[col];
            S->p_temp_mat_data[row * num_taps + col] = val;
            S->p_temp_mat_data[col * num_taps + row] = val;  // 镜像到下三角
        }
    }
    
    // 计算 P = P - k * x^T * P
    arm_mat_sub_f32(&S->P, &S->temp_mat, &S->P);
    
    // 计算 P = (1/lambda) * P
    arm_mat_scale_f32(&S->P, S->lambda_inv, &S->P);
    
    // 强制协方差矩阵对称性（减少数值误差累积）
    // 每次更新后对称化：P = (P + P^T) / 2
    for (uint16_t i = 0; i < num_taps; i++)
    {
        for (uint16_t j = i + 1; j < num_taps; j++)
        {
            float32_t avg = 0.5f * (S->p_mat_p_data[i * num_taps + j] + 
                                     S->p_mat_p_data[j * num_taps + i]);
            S->p_mat_p_data[i * num_taps + j] = avg;
            S->p_mat_p_data[j * num_taps + i] = avg;
        }
    }
    
    // 定期添加正则化防止数值退化
    if (S->update_count > 0 && S->update_count % 1000 == 0)
    {
        for (uint16_t i = 0; i < num_taps; i++)
        {
            S->p_mat_p_data[i * num_taps + i] += 1e-6f;
        }
        LOGERROR("RLS_ARM: Applied regularization at update %d", S->update_count);
    }
    
    // 更新统计信息
    S->update_count++;
    S->last_error = error;
    
    // 更新误差方差 (指数移动平均)
    float32_t alpha = 0.1f;  // 平滑因子
    S->error_variance = alpha * error * error + (1.0f - alpha) * S->error_variance;
    
    return ARM_MATH_SUCCESS;
}

/**
 * @brief RLS直接向量更新算法（不使用滑动窗口）
 * @note 用于向量输入版本，输入向量直接作为特征向量使用
 */

static arm_status rls_arm_update_direct(rls_arm_instance_f32 *S,
                                       const float32_t *p_input_vec,
                                       float32_t reference,
                                       float32_t *p_output,
                                       float32_t *p_error)
{

    arm_status status;
    uint16_t num_taps = S->num_taps;
    
    // 1. 直接使用输入向量（不更新滑动窗口）
    // 注意：p_state已经在调用前被设置为输入向量
    
    // 2. 计算滤波器输出: y = x^T * w
    float32_t output = 0.0f;
    arm_dot_prod_f32(p_input_vec, S->p_mat_w_data, num_taps, &output);
    //////////////////////////////////////////////yk-o*T*Θk-1
    *p_output = output;

    // 3. 计算误差: e = d - y
    float32_t error = reference - output;
    *p_error = error;
    
    // 4. 计算增益向量: k = P * x / (lambda + x^T * P * x)
    // 计算 P * x
    status = arm_mat_mult_f32(&S->P, &S->x, &S->temp_vec);
    if (status != ARM_MATH_SUCCESS)
    {
        LOGERROR("RLS_ARM: Matrix multiplication failed");
        return status;
    }

    // 计算 x^T * P * x
    float32_t k_den = 0.0f;
    arm_dot_prod_f32(p_input_vec, S->p_temp_vec_data, num_taps, &k_den);
    k_den += S->lambda;
    //////////////////////////////////////////////////////γ+okt*pk-1*ok+γ
    // 数值稳定性检查
    if (k_den < 1e-8f)
    {
        LOGERROR("RLS_ARM: k_den too small (%.2e), using regularization", k_den);
        k_den = 1e-8f;
    }
    /////////////////////////////// 计算增益向量 k = (P * x) / (lambda + x^T * P * x)
    arm_scale_f32(S->p_temp_vec_data, 1.0f / k_den, S->p_mat_k_data, num_taps);

    // 5. 更新权重: w = w + k * e
    arm_scale_f32(S->p_mat_k_data, error, S->p_temp_vec_data, num_taps);
    if(isnan(*S->p_mat_w_data)  && p_mat_w_data_look == 0.0f)
    {
        p_mat_w_data_look = 3.2f;
    }
    arm_mat_add_f32(&S->w, &S->temp_vec, &S->w);//////////////////////到这里,temp_vec
    if(isnan(*S->p_mat_w_data)  && p_mat_w_data_look == 0.0f)
    {
        p_mat_w_data_look = 3.3f;
    }

    ///////////////////////////////////////////θk = θk-1+Kk*(yk-o*T*Θk-1)
    // 6. 更新协方差矩阵: P = (1/lambda) * (P - k * x^T * P)
    //    其中 temp_vec = P * x (已计算)
    // 计算 k * x^T * P = k * (P*x)^T (外积运算)
    // 为了保持数值稳定性，使用对称化的外积计算
    for (uint16_t row = 0; row < num_taps; row++)/////////////////计算 外积矩阵
    {
        float32_t k_row = S->p_mat_k_data[row];
        for (uint16_t col = row; col < num_taps; col++)  // 只计算上三角
        {
            float32_t val = k_row * S->p_temp_vec_data[col];
            S->p_temp_mat_data[row * num_taps + col] = val;
            S->p_temp_mat_data[col * num_taps + row] = val;  // 镜像到下三角
        }
    }
    // 计算 P = P - k * x^T * P
    arm_mat_sub_f32(&S->P, &S->temp_mat, &S->P);
    
    // 计算 P = (1/lambda) * P
    arm_mat_scale_f32(&S->P, S->lambda_inv, &S->P);
    
    // 强制协方差矩阵对称性（减少数值误差累积）
    // 每次更新后对称化：P = (P + P^T) / 2
    for (uint16_t i = 0; i < num_taps; i++)
    {
        for (uint16_t j = i + 1; j < num_taps; j++)
        {
            float32_t avg = 0.5f * (S->p_mat_p_data[i * num_taps + j] + 
                                     S->p_mat_p_data[j * num_taps + i]);
            S->p_mat_p_data[i * num_taps + j] = avg;
            S->p_mat_p_data[j * num_taps + i] = avg;
        }
    }

    // 定期添加正则化防止数值退化
    if (S->update_count > 0 && S->update_count % 1000 == 0)
    {
        for (uint16_t i = 0; i < num_taps; i++)
        {
            S->p_mat_p_data[i * num_taps + i] += 1e-6f;
        }
        LOGERROR("RLS_ARM: Applied regularization at update %d", S->update_count);
    }
    
    // 更新统计信息
    S->update_count++;
    S->last_error = error;
    
    // 更新误差方差 (指数移动平均)
    float32_t alpha = 0.1f;  // 平滑因子
    S->error_variance = alpha * error * error + (1.0f - alpha) * S->error_variance;

    return ARM_MATH_SUCCESS;
}

// ==================== 公共函数实现 ====================

/**
 * @brief 初始化RLS滤波器 (ARM优化版本)
 */
arm_status rls_arm_init_f32(rls_arm_instance_f32 *S, const rls_arm_config_t *config)
{
    if (S == NULL || config == NULL)
    {
        LOGERROR("RLS_ARM: Invalid parameters");
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    // 验证配置参数
    arm_status status = rls_arm_validate_config(config);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    
    // 如果已经初始化，先清理
    if (S->is_initialized)
    {
        rls_arm_deinit_f32(S);
    }
    
    // 设置基本参数
    S->num_taps = config->num_taps;
    S->lambda = config->lambda;
    S->delta = config->delta;
    S->lambda_inv = 1.0f / S->lambda;
    S->is_initialized = 0;
    S->update_count = 0;
    S->last_error = 0.0f;
    S->error_variance = 0.0f;
    S->avg_exec_time = 0.0f;
    S->stability_checks = 0;
    
    // 分配内存
    status = rls_arm_allocate_memory(S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    
    // 初始化状态向量为零
    arm_fill_f32(0.0f, S->p_state, S->num_taps);
    arm_fill_f32(0.0f, S->p_mat_w_data, S->num_taps);
    
    // 初始化协方差矩阵P为delta*I
    arm_fill_f32(0.0f, S->p_mat_p_data, S->num_taps * S->num_taps);
    for (uint16_t i = 0; i < S->num_taps; i++)
    {
        S->p_mat_p_data[i * S->num_taps + i] = S->delta;
    }
    
    // 初始化ARM矩阵实例
    arm_mat_init_f32(&S->P, S->num_taps, S->num_taps, S->p_mat_p_data);
    arm_mat_init_f32(&S->w, S->num_taps, 1, S->p_mat_w_data);
    arm_mat_init_f32(&S->x, S->num_taps, 1, S->p_state);
    arm_mat_init_f32(&S->k, S->num_taps, 1, S->p_mat_k_data);
    arm_mat_init_f32(&S->temp_vec, S->num_taps, 1, S->p_temp_vec_data);
    arm_mat_init_f32(&S->temp_mat, S->num_taps, S->num_taps, S->p_temp_mat_data);
    
    S->is_initialized = 1;
    LOGERROR("RLS_ARM: Initialized with num_taps=%d, lambda=%.6f, delta=%.6f", 
          S->num_taps, S->lambda, S->delta);
    
    return ARM_MATH_SUCCESS;
}

/**
 * @brief 释放RLS滤波器资源
 */
void rls_arm_deinit_f32(rls_arm_instance_f32 *S)
{
    if (S == NULL) return;
    
    rls_arm_cleanup_memory(S);
    S->is_initialized = 0;
    S->update_count = 0;
    S->num_taps = 0;
}

/**
 * @brief 重置RLS滤波器状态
 */
arm_status rls_arm_reset_f32(rls_arm_instance_f32 *S)
{
    arm_status status = rls_arm_validate_params(S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    
    // 重置状态向量
    arm_fill_f32(0.0f, S->p_state, S->num_taps);
    arm_fill_f32(0.0f, S->p_mat_w_data, S->num_taps);
    
    // 重置协方差矩阵
    arm_fill_f32(0.0f, S->p_mat_p_data, S->num_taps * S->num_taps);
    for (uint16_t i = 0; i < S->num_taps; i++)
    {
        S->p_mat_p_data[i * S->num_taps + i] = S->delta;
    }
    
    S->update_count = 0;
    S->last_error = 0.0f;
    S->error_variance = 0.0f;
    
    LOGERROR("RLS_ARM: Reset completed");
    
    return ARM_MATH_SUCCESS;
}

/**
 * @brief RLS滤波器更新函数 (标量输入版本)
 */
arm_status rls_arm_f32(rls_arm_instance_f32 *S,
                       const float32_t *p_src,
                       const float32_t *p_ref,
                       float32_t *p_out,
                       float32_t *p_err,
                       uint32_t block_size)
{
    arm_status status = rls_arm_validate_params(S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    
    if (p_src == NULL || p_ref == NULL || p_out == NULL || p_err == NULL)
    {
        LOGERROR("RLS_ARM: Invalid input/output pointers");
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (block_size == 0)
    {
        LOGERROR("RLS_ARM: block_size is 0, nothing to process");
        return ARM_MATH_SUCCESS;
    }
    
    float32_t exec_time;
    TIME_ELAPSE(exec_time,
    {
        for (uint32_t i = 0; i < block_size; i++)
        {
            status = rls_arm_update_core(S, &p_src[i], p_ref[i], &p_out[i], &p_err[i]);
            if (status != ARM_MATH_SUCCESS)
            {
                return status;
            }
        }
    });
    
    // 更新平均执行时间
    S->avg_exec_time = 0.9f * S->avg_exec_time + 0.1f * exec_time;
    
    return ARM_MATH_SUCCESS;
}

/**
 * @brief RLS滤波器更新函数 (向量输入版本)
 * @note 对于向量输入，直接使用完整的输入向量而不是滑动窗口
 */
arm_status rls_arm_vector_f32(rls_arm_instance_f32 *S,
                             const float32_t *p_src,
                             const float32_t *p_ref,
                             float32_t *p_out,
                             float32_t *p_err,
                             uint32_t num_vectors)
{
    arm_status status = rls_arm_validate_params(S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    
    if (p_src == NULL || p_ref == NULL || p_out == NULL || p_err == NULL)
    {
        LOGERROR("RLS_ARM: Invalid input/output pointers");
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (num_vectors == 0)
    {
        LOGERROR("RLS_ARM: num_vectors is 0, nothing to process");
        return ARM_MATH_SUCCESS;
    }
    
    for (uint32_t i = 0; i < num_vectors; i++)
    {
        // 直接复制完整的输入向量到状态缓冲区（不使用滑动窗口）
        const float32_t *p_input_vec = p_src + i * S->num_taps;
        memcpy(S->p_state, p_input_vec, S->num_taps * sizeof(float32_t));
        
        // 使用直接向量更新函数（不使用滑动窗口）
        status = rls_arm_update_direct(S, p_input_vec, p_ref[i], &p_out[i], &p_err[i]);
        if (status != ARM_MATH_SUCCESS)
        {
            return status;
        }
    }
    
    return ARM_MATH_SUCCESS;
}
/**
 * @brief RLS（递归最小二乘）算法用于自适应控制的参数更新函数
 * @note 控制领域应用中，通常直接使用完整的特征向量作为输入，而非滑动窗口缓存的历史数据
 *
 * @param[in,out] S        RLS算法实例结构体指针，包含算法状态、滤波器系数、增益等关键参数
 * @param[in]     p_input  输入特征向量指针，指向当前时刻的系统状态或输入信号（长度为num_taps）
 * @param[in]     target_output 期望输出（参考值），用于计算误差并驱动参数更新
 * @param[out]    p_error  输出误差指针,存储当前时刻的期望输出与实际输出的差值（target_output - 实际输出）
 *
 * @return arm_status 函数执行状态码
 *         - ARM_MATH_SUCCESS：成功执行
 *         - ARM_MATH_ARGUMENT_ERROR：输入参数指针为空
 *         - 其他错误码：由参数校验函数rls_arm_validate_params返回（如实例未初始化等）
 */
float32_t p_mat_w_data_look;
arm_status rls_arm_control_f32(rls_arm_instance_f32 *S,
                               const float32_t *p_input,
                               float32_t target_output,
                               float32_t *p_error)
{
    // 首先校验RLS实例参数的有效性（如系数长度、状态缓冲区等是否合法）
    arm_status status = rls_arm_validate_params(S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status; // 若参数无效，直接返回错误状态
    }


    // 检查输入/输出指针是否为空（空指针会导致访问异常）
    if (p_input == NULL || p_error == NULL)
    {
        LOGERROR("RLS_ARM: Invalid input pointers"); // 记录错误日志
        return ARM_MATH_ARGUMENT_ERROR;           // 返回参数错误码
    }

    // 将当前输入特征向量复制到状态缓冲区
    // （控制应用中，状态缓冲区直接存储当前完整输入，而非历史数据的滑动窗口）
    memcpy(S->p_state, p_input, S->num_taps * sizeof(float32_t));


    // 调用直接向量更新函数执行RLS迭代：
    // 1. 基于当前输入计算实际输出
    // 2. 计算误差（target_output - 实际输出）并写入p_error
    // 3. 根据误差更新滤波器系数和协方差矩阵
    float32_t output; // 临时变量，存储当前时刻的实际输出（由更新函数计算）
    status = rls_arm_update_direct(S, p_input, target_output, &output, p_error);


    return status; // 返回更新过程的执行状态
}
/**
 * @brief 获取RLS（递归最小二乘）滤波器当前的权重系数
 *
 * @param[in]  S         RLS算法实例实例结构体指针，包含当前的权重系数等状态信息
 * @param[out] p_weights 输出指针，用于存储获取到的权重系数（长度需与滤波器阶数num_taps一致）
 *
 * @return arm_status 函数执行状态码
 *         - ARM_MATH_SUCCESS：成功获取权重系数
 *         - ARM_MATH_ARGUMENT_ERROR：输出指针p_weights为空
 *         - 其他错误码：由参数校验函数rls_arm_validate_params返回（如实例未初始化、权重缓冲区无效等）
 */
arm_status rls_arm_get_weights_f32(rls_arm_instance_f32 *S, float32_t *p_weights)
{
    // 校验RLS实例参数的有效性（如实例是否初始化、权重缓冲区是否合法等）
    arm_status status = rls_arm_validate_params(S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status; // 若参数无效，直接返回错误状态
    }

    // 检查输出指针是否为空（空指针会导致数据写入异常）
    if (p_weights == NULL)
    {
        LOGERROR("RLS_ARM: Invalid output pointer"); // 记录错误日志
        return ARM_MATH_ARGUMENT_ERROR;           // 返回参数错误码
    }

    // 将RLS实例中存储的当前权重系数（p_mat_w_data）复制到输出缓冲区p_weights
    // 复制长度为滤波器阶数num_taps，确保完整获取所有权重
    arm_copy_f32(S->p_mat_w_data, p_weights, S->num_taps);

    return ARM_MATH_SUCCESS; // 成功返回权重系数
}

/**
 * @brief 设置RLS滤波器权重
 */
arm_status rls_arm_set_weights_f32(rls_arm_instance_f32 *S, const float32_t *p_weights)
{
    arm_status status = rls_arm_validate_params(S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    
    if (p_weights == NULL)
    {
        LOGERROR("RLS_ARM: Invalid input pointer");
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    arm_copy_f32(p_weights, S->p_mat_w_data, S->num_taps);
    
    LOGERROR("RLS_ARM: Weights updated");
    
    return ARM_MATH_SUCCESS;
}

/**
 * @brief 检查RLS滤波器数值稳定性
 */
arm_status rls_arm_check_stability_f32(rls_arm_instance_f32 *S)
{
    arm_status status = rls_arm_validate_params(S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    
    uint16_t num_taps = S->num_taps;
    float32_t *P_data = S->p_mat_p_data;
    
    // 1. 检查协方差矩阵P的对角线元素（正定性的必要条件）
    for (uint16_t i = 0; i < num_taps; i++)
    {
        float32_t diag_val = P_data[i * num_taps + i];
        if (diag_val <= 0.0f || diag_val > 1e6f)
        {
            LOGERROR("RLS_ARM: Unstable diagonal element P[%d,%d] = %.6f", i, i, diag_val);
            S->stability_checks++;
            return ARM_MATH_SINGULAR;
        }
    }
    
    // 2. 检查协方差矩阵P的迹
    float32_t trace = 0.0f;
    for (uint16_t i = 0; i < num_taps; i++)
    {
        trace += P_data[i * num_taps + i];
    }
    
    if (trace > 1e6f || trace < 1e-6f)
    {
        LOGERROR("RLS_ARM: Unstable trace = %.6f", trace);
        S->stability_checks++;
        return ARM_MATH_SINGULAR;
    }
    
    // 3. 检查协方差矩阵的对称性
    float32_t max_asymmetry = 0.0f;
    for (uint16_t i = 0; i < num_taps; i++)
    {
        for (uint16_t j = i + 1; j < num_taps; j++)
        {
            float32_t val_ij = P_data[i * num_taps + j];
            float32_t val_ji = P_data[j * num_taps + i];
            float32_t diff = fabsf(val_ij - val_ji);
            if (diff > max_asymmetry)
            {
                max_asymmetry = diff;
            }
        }
    }
    
    // 对称性容差：相对于迹的1e-6
    float32_t symmetry_threshold = trace * 1e-6f;
    if (max_asymmetry > symmetry_threshold)
    {
        LOGERROR("RLS_ARM: Matrix asymmetry detected: %.2e (threshold: %.2e)", 
              max_asymmetry, symmetry_threshold);
        S->stability_checks++;
        // 不返回错误，因为我们有对称性强制机制
    }
    
    // 4. 检查权重向量的幅度
    for (uint16_t i = 0; i < num_taps; i++)
    {
        float32_t weight = S->p_mat_w_data[i];
        if (fabsf(weight) > 1e6f || isnan(weight) || isinf(weight))
        {
            LOGERROR("RLS_ARM: Unstable weight[%d] = %.6f", i, weight);
            S->stability_checks++;
            return ARM_MATH_SINGULAR;
        }
    }
    
    // 5. 检查NaN和Inf（协方差矩阵）
    for (uint16_t i = 0; i < num_taps * num_taps; i++)
    {
        if (isnan(P_data[i]) || isinf(P_data[i]))
        {
            LOGERROR("RLS_ARM: NaN or Inf detected in P matrix");
            S->stability_checks++;
            return ARM_MATH_SINGULAR;
        }
    }
    
    return ARM_MATH_SUCCESS;
}

/**
 * @brief 获取RLS滤波器状态信息
 */
arm_status rls_arm_get_status_f32(rls_arm_instance_f32 *S, rls_arm_status_t *status)
{
    arm_status arm_status = rls_arm_validate_params(S);
    if (arm_status != ARM_MATH_SUCCESS)
    {
        return arm_status;
    }
    
    if (status == NULL)
    {
        LOGERROR("RLS_ARM: Invalid output pointer");
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    uint16_t num_taps = S->num_taps;
    float32_t *P_data = S->p_mat_p_data;
    
    // 计算协方差矩阵的迹和条件数
    float32_t trace = 0.0f;
    float32_t max_eigenval = 0.0f;
    float32_t min_eigenval = 1e6f;
    
    for (uint16_t i = 0; i < num_taps; i++)
    {
        float32_t diag_val = P_data[i * num_taps + i];
        trace += diag_val;
        
        if (diag_val > max_eigenval)
            max_eigenval = diag_val;
        if (diag_val < min_eigenval)
            min_eigenval = diag_val;
    }
    
    status->trace = trace;
    
    // 估算条件数
    if (min_eigenval > 1e-12f)
    {
        status->condition_number = max_eigenval / min_eigenval;
    }
    else
    {
        status->condition_number = 1e12f;  // 表示数值不稳定
    }
    
    // 计算误差和权重范数
    arm_power_f32(&S->last_error, 1, &status->error_norm);
    arm_power_f32(S->p_mat_w_data, num_taps, &status->weight_norm);
    status->weight_norm = sqrtf(status->weight_norm);
    
    status->update_count = S->update_count;
    status->is_stable = (rls_arm_check_stability_f32(S) == ARM_MATH_SUCCESS) ? 1 : 0;
    
    return ARM_MATH_SUCCESS;
}

/**
 * @brief 自适应调整遗忘因子
 */
arm_status rls_arm_adapt_lambda_f32(rls_arm_instance_f32 *S, float32_t error_magnitude)
{
    arm_status status = rls_arm_validate_params(S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    
    // 基于误差幅度自适应调整遗忘因子
    float32_t error_threshold = sqrtf(S->error_variance);
    float32_t adaptation_factor = 0.1f;
    
    // 避免除零：如果error_threshold太小，使用默认阈值
    if (error_threshold < 1e-6f)
    {
        error_threshold = 1e-3f;  // 默认小误差阈值
    }
    
    if (error_magnitude > error_threshold)
    {
        // 误差较大，减小遗忘因子，提高适应性
        S->lambda = fmaxf(0.9f, S->lambda - adaptation_factor * (error_magnitude / error_threshold - 1.0f));
    }
    else
    {
        // 误差较小，增大遗忘因子，提高稳定性
        S->lambda = fminf(0.999f, S->lambda + adaptation_factor * (1.0f - error_magnitude / error_threshold));
    }
    
    S->lambda_inv = 1.0f / S->lambda;
    
    return ARM_MATH_SUCCESS;
}

/**
 * @brief 计算滤波器输出 (不更新权重和状态)
 * @note 这是一个纯预测函数，不会修改滤波器的内部状态
 * @note 使用内部临时缓冲区，避免VLA和栈溢出问题
 */
arm_status rls_arm_predict_f32(rls_arm_instance_f32 *S, 
                               const float32_t *p_input, 
                               float32_t *p_output)
{
    arm_status status = rls_arm_validate_params(S);
    if (status != ARM_MATH_SUCCESS)
    {
        return status;
    }
    
    if (p_input == NULL || p_output == NULL)
    {
        LOGERROR("RLS_ARM: Invalid input/output pointers");
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    // 对于标量输入，使用临时缓冲区p_temp_vec_data来构建输入向量
    // 这样避免了VLA和栈溢出问题
    
    // 构建临时输入向量：[new_input, old_state[0], old_state[1], ..., old_state[num_taps-2]]
    S->p_temp_vec_data[0] = *p_input;
    if (S->num_taps > 1)
    {
        memcpy(&S->p_temp_vec_data[1], S->p_state, (S->num_taps - 1) * sizeof(float32_t));
    }
    
    // 计算滤波器输出: y = x^T * w
    arm_dot_prod_f32(S->p_temp_vec_data, S->p_mat_w_data, S->num_taps, p_output);
    
    return ARM_MATH_SUCCESS;
}

// ==================== 工具函数实现 ====================

/**
 * @brief 计算默认配置参数
 */
void rls_arm_get_default_config(uint16_t num_taps, rls_arm_config_t *config)
{
    if (config == NULL) return;
    
    config->num_taps = num_taps;
    config->lambda = 0.99f;                    // 默认遗忘因子
    config->delta = 1.0f;                     // 默认初始协方差
    config->stability_threshold = 1e-6f;      // 稳定性阈值
    config->max_updates = 1000000;            // 最大更新次数
    config->enable_adaptive_lambda = 1;       // 启用自适应遗忘因子
    config->enable_stability_check = 1;       // 启用稳定性检查
}

/**
 * @brief 验证配置参数
 */
arm_status rls_arm_validate_config(const rls_arm_config_t *config)
{
    
    if (config == NULL)
    {
        LOGERROR("RLS_ARM: Invalid config pointer");
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (config->num_taps == 0 || config->num_taps > 1000)
    {
        LOGERROR("RLS_ARM: Invalid num_taps (%d)", config->num_taps);
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (config->lambda <= 0.0f || config->lambda > 1.0f)
    {
        LOGERROR("RLS_ARM: Invalid lambda (%.6f)", config->lambda);
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    if (config->delta <= 0.0f)
    {
        LOGERROR("RLS_ARM: Invalid delta (%.6f)", config->delta);
        return ARM_MATH_ARGUMENT_ERROR;
    }
    
    return ARM_MATH_SUCCESS;
}

/**
 * @brief 计算所需内存大小
 */
uint32_t rls_arm_get_memory_size(uint16_t num_taps)
{
    uint32_t state_size = num_taps * sizeof(float32_t);
    uint32_t mat_p_size = num_taps * num_taps * sizeof(float32_t);
    uint32_t mat_w_size = num_taps * sizeof(float32_t);
    uint32_t mat_x_size = num_taps * sizeof(float32_t);
    uint32_t mat_k_size = num_taps * sizeof(float32_t);
    uint32_t temp_vec_size = num_taps * sizeof(float32_t);
    uint32_t temp_mat_size = num_taps * num_taps * sizeof(float32_t);
    
    return state_size + mat_p_size + mat_w_size + mat_x_size + 
           mat_k_size + temp_vec_size + temp_mat_size;
}
