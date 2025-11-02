#ifndef RLS_ARM_H
#define RLS_ARM_H

#include "arm_math.h"
#include"drv_dwt.h"
/**
 * @brief RLS算法参数说明
 * 
 * lambda (遗忘因子): 0 < lambda <= 1
 * - 接近1: 长期记忆，适合稳态环境
 * - 接近0: 短期记忆，适合时变环境
 * 
 * delta (初始协方差): delta > 0
 * - 较大值: 快速收敛，但可能不稳定
 * - 较小值: 稳定收敛，但速度较慢
 */

/**
 * @brief RLS 滤波器实例结构体 (ARM优化版本)
 */
typedef struct
{
    // 基本参数
    uint16_t num_taps;      // 滤波器阶数 (权重数量)
    float32_t lambda;       // 遗忘因子 (0 < lambda <= 1)
    float32_t delta;        // 初始协方差矩阵P的对角线元素值
    float32_t lambda_inv;   // 1/lambda 预计算值，提高性能
    
    // ARM CMSIS DSP 矩阵实例
    arm_matrix_instance_f32 P;      // 协方差矩阵 P (num_taps x num_taps)
    arm_matrix_instance_f32 w;      // 权重向量 w (num_taps x 1)
    arm_matrix_instance_f32 x;      // 输入向量 x (num_taps x 1)
    arm_matrix_instance_f32 k;      // 增益向量 k (num_taps x 1)
    arm_matrix_instance_f32 temp_vec; // 临时向量 (num_taps x 1)
    arm_matrix_instance_f32 temp_mat; // 临时矩阵 (num_taps x num_taps)
    
    // 数据缓冲区 - 连续内存布局，提高缓存效率
    float32_t *p_data_buffer;       // 主数据缓冲区
    float32_t *p_state;             // 状态缓冲区 (num_taps)
    float32_t *p_mat_p_data;        // P 矩阵数据区 (num_taps * num_taps)
    float32_t *p_mat_w_data;        // w 向量数据区 (num_taps)
    float32_t *p_mat_x_data;        // x 向量数据区 (num_taps)
    float32_t *p_mat_k_data;        // k 向量数据区 (num_taps)
    float32_t *p_temp_vec_data;     // 临时向量数据区 (num_taps)
    float32_t *p_temp_mat_data;     // 临时矩阵数据区 (num_taps * num_taps)
    
    // 算法状态
    uint32_t is_initialized;        // 初始化标志
    uint32_t update_count;          // 更新次数计数
    float32_t last_error;            // 上次误差，用于稳定性检查
    float32_t error_variance;       // 误差方差，用于自适应调整
    
    // 性能统计
    float32_t avg_exec_time;        // 平均执行时间
    uint32_t stability_checks;      // 稳定性检查次数
} rls_arm_instance_f32;

/**
 * @brief RLS算法配置结构体
 */
typedef struct
{
    uint16_t num_taps;              // 滤波器阶数
    float32_t lambda;               // 遗忘因子
    float32_t delta;                // 初始协方差
    float32_t stability_threshold;  // 稳定性阈值
    uint32_t max_updates;           // 最大更新次数
    uint8_t enable_adaptive_lambda; // 启用自适应遗忘因子
    uint8_t enable_stability_check; // 启用稳定性检查
} rls_arm_config_t;

/**
 * @brief RLS算法状态信息
 */
typedef struct
{
    float32_t condition_number;     // 协方差矩阵条件数
    float32_t trace;                // 协方差矩阵迹
    float32_t error_norm;           // 误差范数
    float32_t weight_norm;          // 权重范数
    uint32_t update_count;          // 更新次数
    uint8_t is_stable;              // 稳定性标志
} rls_arm_status_t;

// ==================== 核心函数接口 ====================

/**
 * @brief 初始化RLS滤波器 (ARM优化版本)
 * @param S 指向RLS滤波器实例的指针
 * @param config 配置参数
 * @return 成功返回 ARM_MATH_SUCCESS, 失败返回错误码
 */
arm_status rls_arm_init_f32(rls_arm_instance_f32 *S, const rls_arm_config_t *config);

/**
 * @brief 释放RLS滤波器资源
 * @param S 指向RLS滤波器实例的指针
 */
void rls_arm_deinit_f32(rls_arm_instance_f32 *S);

/**
 * @brief 重置RLS滤波器状态
 * @param S 指向RLS滤波器实例的指针
 * @return 成功返回 ARM_MATH_SUCCESS, 失败返回错误码
 */
arm_status rls_arm_reset_f32(rls_arm_instance_f32 *S);

/**
 * @brief RLS滤波器更新函数 (标量输入版本)
 * @param S 指向RLS滤波器实例的指针
 * @param p_src 输入信号样本序列
 * @param p_ref 参考(期望)信号样本序列
 * @param p_out 滤波器输出信号
 * @param p_err 误差信号
 * @param block_size 处理的样本数量
 * @return 成功返回 ARM_MATH_SUCCESS, 失败返回错误码
 */
arm_status rls_arm_f32(rls_arm_instance_f32 *S,
                       const float32_t *p_src,
                       const float32_t *p_ref,
                       float32_t *p_out,
                       float32_t *p_err,
                       uint32_t block_size);

/**
 * @brief RLS滤波器更新函数 (向量输入版本)
 * @note 向量输入模式不使用滑动窗口，每个输入向量是完整的特征向量
 * @param S 指向RLS滤波器实例的指针
 * @param p_src 输入向量序列 (按行存储: [vec1, vec2, ...])，每个向量长度为num_taps
 * @param p_ref 参考信号样本
 * @param p_out 滤波器输出信号
 * @param p_err 误差信号
 * @param num_vectors 处理的向量数量
 * @return 成功返回 ARM_MATH_SUCCESS, 失败返回错误码
 */
arm_status rls_arm_vector_f32(rls_arm_instance_f32 *S,
                             const float32_t *p_src,
                             const float32_t *p_ref,
                             float32_t *p_out,
                             float32_t *p_err,
                             uint32_t num_vectors);

/**
 * @brief RLS算法用于自适应控制的更新函数
 * @note 此函数使用完整的特征向量输入，不使用滑动窗口
 * @param S 指向RLS滤波器实例的指针
 * @param p_input 当前输入特征向量（长度为num_taps）
 * @param actual_output 系统实际输出（当前未使用）
 * @param target_output 期望输出
 * @param p_error 计算得到的误差
 * @return 成功返回 ARM_MATH_SUCCESS, 失败返回错误码
 */
arm_status rls_arm_control_f32(rls_arm_instance_f32 *S,
                               const float32_t *p_input,

                               float32_t target_output,
                               float32_t *p_error);

// ==================== 辅助函数接口 ====================

/**
 * @brief 获取RLS滤波器当前权重
 * @param S 指向RLS滤波器实例的指针
 * @param p_weights 输出权重向量缓冲区
 * @return 成功返回 ARM_MATH_SUCCESS, 失败返回错误码
 */
arm_status rls_arm_get_weights_f32(rls_arm_instance_f32 *S, float32_t *p_weights);

/**
 * @brief 设置RLS滤波器权重
 * @param S 指向RLS滤波器实例的指针
 * @param p_weights 输入权重向量
 * @return 成功返回 ARM_MATH_SUCCESS, 失败返回错误码
 */
arm_status rls_arm_set_weights_f32(rls_arm_instance_f32 *S, const float32_t *p_weights);

/**
 * @brief 检查RLS滤波器数值稳定性
 * @param S 指向RLS滤波器实例的指针
 * @return 稳定返回 ARM_MATH_SUCCESS, 不稳定返回 ARM_MATH_SINGULAR
 */
arm_status rls_arm_check_stability_f32(rls_arm_instance_f32 *S);

/**
 * @brief 获取RLS滤波器状态信息
 * @param S 指向RLS滤波器实例的指针
 * @param status 输出状态信息
 * @return 成功返回 ARM_MATH_SUCCESS, 失败返回错误码
 */
arm_status rls_arm_get_status_f32(rls_arm_instance_f32 *S, rls_arm_status_t *status);

/**
 * @brief 自适应调整遗忘因子
 * @param S 指向RLS滤波器实例的指针
 * @param error_magnitude 当前误差幅度
 * @return 成功返回 ARM_MATH_SUCCESS, 失败返回错误码
 */
arm_status rls_arm_adapt_lambda_f32(rls_arm_instance_f32 *S, float32_t error_magnitude);

/**
 * @brief 计算滤波器输出 (不更新权重和状态)
 * @note 这是一个纯预测函数，不会修改滤波器内部状态
 * @note 使用内部临时缓冲区，无栈溢出风险
 * @param S 指向RLS滤波器实例的指针
 * @param p_input 输入标量
 * @param p_output 输出标量
 * @return 成功返回 ARM_MATH_SUCCESS, 失败返回错误码
 */
arm_status rls_arm_predict_f32(rls_arm_instance_f32 *S, 
                               const float32_t *p_input, 
                               float32_t *p_output);

// ==================== 工具函数接口 ====================

/**
 * @brief 计算默认配置参数
 * @param num_taps 滤波器阶数
 * @param config 输出配置参数
 */
void rls_arm_get_default_config(uint16_t num_taps, rls_arm_config_t *config);

/**
 * @brief 验证配置参数
 * @param config 配置参数
 * @return 有效返回 ARM_MATH_SUCCESS, 无效返回 ARM_MATH_ARGUMENT_ERROR
 */
arm_status rls_arm_validate_config(const rls_arm_config_t *config);

/**
 * @brief 计算所需内存大小
 * @param num_taps 滤波器阶数
 * @return 所需内存字节数
 */
uint32_t rls_arm_get_memory_size(uint16_t num_taps);

#endif // RLS_ARM_H
