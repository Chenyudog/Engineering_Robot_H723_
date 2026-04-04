//
// Created by 刘嘉俊 on 26-3-7.
//

#ifndef CTRBOARD_H7_ALL_KINEMATICMDHDSP_H
#define CTRBOARD_H7_ALL_KINEMATICMDHDSP_H


#include "stm32h7xx_hal.h"
#include "arm_math.h"
#include <stdbool.h>

// 定义PI免去一次浮点计算
#ifndef M_PI
#define M_PI        3.14159265358979323846f
#endif
#ifndef M_PI_2
#define M_PI_2      1.57079632679489661923f
#endif
// 弧度转角度常量
#define RAD_TO_DEG  57.295777754771045f

// DSP库加速运算
#define cosf(x) arm_cos_f32(x)
#define sinf(x) arm_sin_f32(x)

#define Matrix arm_matrix_instance_f32 // 矩阵描述结构体（存储了矩阵行列数和数据区指针）
#define Matrix_Init arm_mat_init_f32      // 矩阵初始化，一块float数组会被包装成矩阵
#define Matrix_Add arm_mat_add_f32        // 矩阵加法
#define Matrix_Subtract arm_mat_sub_f32   // 矩阵减法
#define Matrix_Multiply arm_mat_mult_f32  // 矩阵乘法
#define Matrix_Transpose arm_mat_trans_f32// 矩阵转置
#define Matrix_Inverse arm_mat_inverse_f32// 矩阵求逆

//typedef struct {
//    // 此处按理来说应该是theta，但由于机械编码器安装错位或者反相，可能导致多一个或者少一个PI/2的偏置补偿
//    // 所以此处先写theta_offset，则公式为：theta = joint[i] + arm_sdh_table[i].theta_offset;
//    // 在公式中theta是参与运动学解算的参数，而joint是我们最终要求解的关节角度，当偏置为0时，theta就是所求的关节角度
//    float theta_offset;
//    float d;       // DH参数中的d（连杆偏移）
//    float a;       // DH参数中的a（连杆长度）
//    float alpha;   // DH参数中的α（连杆扭转角）
//} SDH_Param_t;

typedef struct {
    float theta_offset;   // theta_i = q[i] + theta_offset
    float d;              // d_i
    float a_prev;         // a_(i-1)
    float alpha_prev;     // alpha_(i-1)  在改进型DH参数中，a和alpha是前一个关节的参数
} MDH_Param_t;

typedef struct {
    float X, Y, Z;   // 位置
    float ROLL, PITCH, YAW;   // 欧拉角，弧度
    float roll_deg, pitch_deg, yaw_deg; // 欧拉角，角度
    float R[9];      // 3x3旋转矩阵，按行存储
    bool hasR;  // 校验是否保存了旋转矩阵
} Pose6D_t;

bool MDH_FK_ToPose6D(const MDH_Param_t table[6], const float q[6], Pose6D_t *pose);


#endif //CTRBOARD_H7_ALL_KINEMATICMDHDSP_H
