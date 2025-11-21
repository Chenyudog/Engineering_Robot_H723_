//
// Created by 刘嘉俊 on 25-1-2.
//

#ifndef CTRBOARD_H7_ALL_CHASSIS_TASK_H
#define CTRBOARD_H7_ALL_CHASSIS_TASK_H
#include "cmsis_os.h"

/**
 * @brief 底盘模式
 */
typedef enum
{
    CHASSIS_RELAX,         //底盘失能
    CHASSIS_ENABLE,     //底盘开环
    CHASSIS_STOP,          //底盘停止
    CHASSIS_FOLLOW_GIMBAL, //底盘跟随云台
    CHASSIS_SPIN,          //底盘陀螺模式
    CHASSIS_FLY,           //底盘飞坡模式
    CHASSIS_AUTO           //底盘自动模式
} chassis_mode_e;

typedef struct
{
    float K1;
    float K2;
    float K3;
    float Err_Lower;
    float Err_Upper;
} PowerCtrl_Parameter_Typedef;


typedef struct
{

    // 经验公式系数

    float A;
    float B;
    float C;
    // 最小二乘法系数
    float Delta;
    float Sqrt;
    // 一元二次方程中间变量

    float K;
    float Power_Max;      // 最大功率
    float Power_Limit[4]; // 输出功率限制
    float Menbership[4];  // 隶属度
    float Torque[4];      // 输出力矩

    float Err[4]; // 期望输出电流于电机实际电流
    struct
    {
        float Torque2_Sum;
        float Omiga2_Sum;
        float Power_Sum;
        float Err_Sum;
        float power_useful_Sum;
        float input_Sum;
    }
    Sum; // 一些数据求和
    struct
    {
        float RLS_Input[4];
        float Torque[4];
        float Omiga[4];
        float Torque_2[4];
        float Omiga_2[4];
        float power_useful[4];
    }
    Measure; // 模型输入的参数

    struct
    {
        float RLS_Input_cmd[4];
        float Torque_cmd[4];
        float Omiga_cmd[4];
    }
        Target; // 模型输入的参数

    float Output[4];
    PowerCtrl_Parameter_Typedef Param;
} PowerCtrl_Typedef;

extern PowerCtrl_Typedef PowerCtrl_Info;
void chassis_cmd_state_machine(void);

#endif //CTRBOARD_H7_ALL_CHASSIS_TASK_H
