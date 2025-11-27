/**
  ******************************************************************************
  * @file    algorithm_task.c
  * @author  Liu JiaJun(187353224@qq.com)
  * @version V1.0.0
  * @date    2025-01-10
  * @brief   机器人算法任务线程，处理复杂算法，避免在其他线程中计算造成阻塞
  ******************************************************************************
  * @attention
  *
  * 本代码遵循GPLv3开源协议，仅供学习交流使用
  * 未经许可不得用于商业用途
  *
  ******************************************************************************
  */
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "chassis_task.h"
#include "PID.h"
#include "dj_motor.h"
#include "stdio.h"
#include "bsp_fdcan.h"
#include "robot.h"
#include "drv_dwt.h"
#include "user_lib.h"
#include "motor_def.h"
#include "robot_task.h"
#include "referee_system.h"
#include "msg_freertos.h"
#include "rls_arm.h"
#include "Power_task.h"

//#define RLS_POWER_LIMIT//功率限制开关

#define K_power 0.10472f//  rpm -> rad/s
#define wheel_ratio    0.05207463310219f  //转子转速转换成轮子转速   1/减速比 ≈ 187/3591 =0.052074
#define K_current 0.001220703125f   //  20 / 16384
#define K_torque  0.3f      //转矩常数 0.3N*M/A
#define CURRENT_TO_TORQUE_RATIO (K_current * K_torque * wheel_ratio)// 转子电流到转子力矩的转换系数，单位：Nm/A   约等于1.9e-5


/*================================= 功率控制相关 ================================= */

PowerCtrl_Typedef PowerCtrl_Info;
RLS_Info_TypeDef RLS_Power_Info;  // RLS滤波器实例，用于功率模型参数辨识
static float Power_Ctrl_Param[5] = {1e-05f, 20.0f, 2.8f, 0.0001f, 500};//RLS拟合初始化参数

float I_cmd[4];//PID计算出来的要发送的电流
uint8_t powerOverloadFlag = 0;  //超功率标志位
float power_max = 20.0f;//便于调试
float Decrease;  // 功率衰减系数

/*================================= 功率控制相关 ================================= */
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */

static struct ins_msg ins_data;
static float target_yaw = 0.0f;
static pid_obj_t *chassis_yaw_pid;
static pid_config_t chassis_yaw_config = INIT_PID_CONFIG(0.373, 0.0, 0.0135, 0.0, 4.3, PID_Trapezoid_Intergral);
static publisher_t * pub_chassis;
static subscriber_t* sub_ins;

static void chassis_pub_init(void);
static void chassis_sub_init(void);
static void chassis_pub_push(void);
static void chassis_sub_pull(void);
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t chassis_task_dwt = 0;   // 毫秒监测
static float chassis_task_dt = 0;       // 线程实际运行时间dt
static float chassis_task_delta = 0;    // 监测线程运行时间
static float chassis_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

struct cmd_chassis_msg cmd_chassis;
extern struct referee_fdb_msg referee_fdb;

static struct chassis_controller_t
{
    pid_obj_t *speed_pid;
}chassis_controller[4];

static dji_motor_object_t *chassis_motor[4];

static int16_t motor_target_speed_rad[4];

static void chassis_motor_init();
static void mecanum_calc(struct cmd_chassis_msg *cmd, int16_t* out_speed);


/* --------------------------------- 电机控制相关 --------------------------------- */
#define CURRENT_POWER_LIMIT_RATE 80
static int16_t motor_control_0(dji_motor_measure_t measure)
{
    static int16_t motor_current_set = 0;
    static int16_t motor_max_current=0;   // 电流值范围：-16380~0~16380
    static int16_t chassis_power_limit=0;
    if (chassis_power_limit==0)
    {
        motor_max_current = 3000;
    }
    motor_current_set =(int16_t) pid_calculate(chassis_controller[0].speed_pid, measure.speed_rad, motor_target_speed_rad[0]);
    VAL_LIMIT(motor_current_set , -motor_max_current, motor_max_current);
    return motor_current_set;
}

static int16_t motor_control_1(dji_motor_measure_t measure)
{
    static int16_t motor_current_set = 0;
    static int16_t motor_max_current = 0;   // 电流值范围：-16380~0~16380,电流值限幅变量
    static int16_t chassis_power_limit = 0;
    if (chassis_power_limit==0)
    {
        motor_max_current = 3000;
    }
    motor_current_set =(int16_t) pid_calculate(chassis_controller[1].speed_pid, measure.speed_rad , motor_target_speed_rad[1] );
    VAL_LIMIT(motor_current_set , -motor_max_current, motor_max_current);
    return motor_current_set;
}

static int16_t motor_control_2(dji_motor_measure_t measure)
{
    static int16_t motor_current_set = 0;
    static int16_t motor_max_current = 0;   // 电流值范围：-16380~0~16380
    static int16_t chassis_power_limit = 0;
    if (chassis_power_limit==0)
    {
        motor_max_current = 3000;
    }
    motor_current_set =(int16_t) pid_calculate(chassis_controller[2].speed_pid, measure.speed_rad, motor_target_speed_rad[2]);
    VAL_LIMIT(motor_current_set , -motor_max_current, motor_max_current);
    return motor_current_set;
}

static int16_t motor_control_3(dji_motor_measure_t measure)
{
    static int16_t motor_current_set = 0;
    static int16_t motor_max_current = 0;   // 电流值范围：-16380~0~16380
    static int16_t chassis_power_limit = 0;
    if (chassis_power_limit==0)
    {
        motor_max_current = 3000;
    }
    motor_current_set =(int16_t) pid_calculate(chassis_controller[3].speed_pid, measure.speed_rad , motor_target_speed_rad[3]);
    VAL_LIMIT(motor_current_set , -motor_max_current, motor_max_current);
    return motor_current_set;
}

/* 底盘每个电机对应的控制函数 */
static void *motor_control[4] ={motor_control_0,motor_control_1,motor_control_2,motor_control_3};

// TODO：将参数都放到配置文件中，通过宏定义进行替换
motor_config_t chassis_motor_config[4] =
        {
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS_NAME,
                        .rx_id = 0x201,
                        .tx_id = 0x201,
                        .controller = &chassis_controller[0],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS_NAME,
                        .rx_id = 0x202,
                        .tx_id = 0x202,
                        .controller = &chassis_controller[1],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS_NAME,
                        .rx_id = 0x203,
                        .tx_id = 0x203,
                        .controller = &chassis_controller[2],
                },
                {
                        .motor_type = M3508,
                        .can_name = CAN_CHASSIS_NAME,
                        .rx_id = 0x204,
                        .tx_id = 0x204,
                        .controller = &chassis_controller[3],
                }
        };


/**
 * @brief 注册底盘电机及其控制器初始化
 */
static void chassis_motor_init()
{
    pid_config_t chassis_speed_config = INIT_PID_CONFIG(CHASSIS_KP_V_MOTOR, CHASSIS_KI_V_MOTOR, CHASSIS_KD_V_MOTOR, CHASSIS_INTEGRAL_V_MOTOR, CHASSIS_MAX_V_MOTOR,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_controller[i].speed_pid = pid_register(&chassis_speed_config);
        chassis_motor[i] = dji_motor_register(&chassis_motor_config[i], motor_control[i]);

        cmd_chassis.ctrl_mode = CHASSIS_ENABLE;
        cmd_chassis.last_mode = CHASSIS_ENABLE;
    }
}

static void mecanum_calc(struct cmd_chassis_msg *cmd, int16_t* out_speed)
{
    // 轮子转速转换系数，转为轮子转速，为rpm/min，每分钟多少转,60是指60秒，转换成分钟，
    // 空载转速482rpm，3Nm满载最高转速469rpm
    // static float wheel_rpm_ratio = 60.0f * CHASSIS_MOTOR_REDUCTION_RATIO / WHEEL_PERIMETER ;
    int16_t wheel_rad[4];  // 转换电机转子的期望转速，而非实际输出轮子的转速

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //m/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //m/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);  //rad/s

//    if (cmd_chassis.ctrl_mode == CHASSIS_ENABLE) {
//        target_yaw -= cmd_chassis.vw * chassis_task_dt * 57.3;
//    }//加负号让其满足左加右
//    else if (cmd_chassis.ctrl_mode == CHASSIS_RELAX) {
//        target_yaw = ins_data.yaw_total_angle;
//    }
//
//    cmd->vw = -pid_calculate(chassis_yaw_pid,ins_data.yaw_total_angle,target_yaw);
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VW_SPEED, MAX_CHASSIS_VW_SPEED);  //rad/s
    // Vw的正负取决与遥感通道是否是正的还是负数的
    // 前后运动相反，则反转vx的正负
    // 左右运动相反，则反转vy的正负
    wheel_rad[3] = (int16_t)((( -cmd->vx - cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO) *K_power);    // 左後輪
    wheel_rad[0] = (int16_t)((( -cmd->vx + cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO) *K_power);    // 左前轮
    wheel_rad[1] = (int16_t)((( +cmd->vx + cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO) *K_power);     // 右前轮
    wheel_rad[2] = (int16_t)((( +cmd->vx - cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO) *K_power);     // 右後輪

    memcpy(out_speed, wheel_rad, 4*sizeof(int16_t));//copy the rpm to out_speed
}

// 定义结构体存储里程计数据
typedef struct {
    float x;           // X轴位移 (m)
    float y;           // Y轴位移 (m)
    float yaw;         // 航向角 (rad)
    float vx;          // X轴速度 (m/s)
    float vy;          // Y轴速度 (m/s)
    float vw;          // 角速度 (rad/s)
    uint32_t last_cyccnt; // DWT时间戳（用于积分）
} odometry_t;


void odometry_update(int32_t *wheel_rpm_actual, odometry_t *odom) {
    // 参数定义
    const float K = (WHEEL_TRACK + WHEEL_BASE) / 2.0f;
    const float ratio_inv = 1.0f / WHELL_RPM_RATIO;

    // 转换电机转速为线速度（m/s）
    float wheel_speed[4];
    for (int i = 0; i < 4; i++) {
        wheel_speed[i] = wheel_rpm_actual[i] * ratio_inv;
    }

    // 计算瞬时速度（逆解核心公式）
    odom->vx = (wheel_speed[1] + wheel_speed[2] - wheel_speed[0] - wheel_speed[3]) / 4.0f;
    odom->vy = (wheel_speed[0] + wheel_speed[1] - wheel_speed[2] - wheel_speed[3]) / 4.0f;
    odom->vw = (wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) / (4.0f * K);

    // 使用DWT计算积分周期（自动处理32位溢出）
    float dt = dwt_get_delta(&odom->last_cyccnt); // 获取精确时间差

    // 更新位姿（带坐标系旋转）
    odom->yaw += odom->vw * dt; // 航向角积分
    const float cos_yaw = cosf(odom->yaw);
    const float sin_yaw = sinf(odom->yaw);

    // 将底盘坐标系速度转换到全局坐标系
    odom->x += (odom->vx * cos_yaw - odom->vy * sin_yaw) * dt;
    odom->y += (odom->vx * sin_yaw + odom->vy * cos_yaw) * dt;
}



void chassis_cmd_enable(void) {
    if (cmd_chassis.last_mode == CHASSIS_RELAX && cmd_chassis.ctrl_mode == CHASSIS_ENABLE)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            dji_motor_enable(chassis_motor[i]);
        }
    }
    cmd_chassis.last_mode = CHASSIS_ENABLE;
}

void chassis_cmd_disable(void) {
    if (cmd_chassis.last_mode == CHASSIS_ENABLE && cmd_chassis.ctrl_mode == CHASSIS_RELAX) {
        for (uint8_t i = 0; i < 4; i++)
        {
            dji_motor_relax(chassis_motor[i]);
        }
        cmd_chassis.last_mode = CHASSIS_RELAX;
    }
}

void chassis_cmd_state_machine(void)
{
    switch (cmd_chassis.ctrl_mode)
    {
        case CHASSIS_RELAX:
            chassis_cmd_disable();
            break;
        case CHASSIS_ENABLE:
            chassis_cmd_enable();
            break;
        case CHASSIS_STOP:
            memset(motor_target_speed_rad, 0, sizeof(motor_target_speed_rad));
            break;
        default:
            // chassis_cmd_disable();
            break;
    }
}

#ifdef RLS_POWER_LIMIT
/**
 * @brief 基于递推最小二乘(RLS)的功率限制函数
 * @param update_weights 是否更新RLS算法的权重参数（1表示更新，0表示不更新）
 * @note 功能：通过RLS算法实时估计电机功率损耗模型，当总功率超过限制时进行功率分配，防止超功率
 */
void rls_power_limit(uint8_t update_weights) {
    dji_motor_object_t *motor;       // 电机对象指针,用于获取电机信息
    dji_motor_measure_t measure;     // 电机测量数据结构体（包含转速等信息）
    PowerCtrl_Info.Power_Max = power_max * 0.8f ;//便于调试   //*0.8,确保安全
    //PowerCtrl_Info.Power_Max = referee_fdb.robot_status.chassis_power_limit;
/*-------------------------更新RLS拟合部分--------------------------*/
    // 遍历4个底盘电机，计算单电机参数并累积总和
    for (int i = 0; i < 4; i++)
    {
//========================================获取RLS拟合需要的数据====================================//

        motor = chassis_motor[i];               // 获取第i个电机的对象
        measure = motor->measure;               // 获取该电机的实时测量数据
        I_cmd[i] = (float)motor->control(measure) ;     //转子目标输出电流
        PowerCtrl_Info.Measure.Omiga[i] = measure.speed_rad;           //转子实际转速,单位rad/s
        PowerCtrl_Info.Measure.Torque[i] = (float)measure.real_current * CURRENT_TO_TORQUE_RATIO;//转子实际转矩 单位N*M/A

        PowerCtrl_Info.Err[i] = fabsf((float)motor_target_speed_rad[i] - (float)measure.speed_rad);
        PowerCtrl_Info.Measure.power_useful[i] = PowerCtrl_Info.Measure.Omiga[i] * PowerCtrl_Info.Measure.Torque[i];//禁止加绝对值，会导致疯车
        PowerCtrl_Info.Measure.Omiga_2[i] = powf(PowerCtrl_Info.Measure.Omiga[i], 2.f);
        PowerCtrl_Info.Measure.Torque_2[i] = powf(PowerCtrl_Info.Measure.Torque[i], 2.f);
        //功率模型:P = k1*w² + k2*τ² + k3 + τ*w
        PowerCtrl_Info.Measure.RLS_Input[i] = (PowerCtrl_Info.Param.K1 * PowerCtrl_Info.Measure.Omiga_2[i] +
                                              PowerCtrl_Info.Param.K2 * PowerCtrl_Info.Measure.Torque_2[i]);

//========================================获取RLS拟合需要的数据====================================//
    }

    /* ==================== 总和计算与RLS更新 ==================== */

    // 计算四个电机的参数总和
    // 计算总误差绝对值（控制精度指标）
    PowerCtrl_Info.Sum.Err_Sum = PowerCtrl_Info.Err[0] + PowerCtrl_Info.Err[1] +
                                 PowerCtrl_Info.Err[2] + PowerCtrl_Info.Err[3];

    //转子力矩平方总和
    PowerCtrl_Info.Sum.Torque2_Sum = PowerCtrl_Info.Measure.Torque_2[0] + PowerCtrl_Info.Measure.Torque_2[1] +
                                     PowerCtrl_Info.Measure.Torque_2[2] + PowerCtrl_Info.Measure.Torque_2[3];
    //转子角速度平方总和
    PowerCtrl_Info.Sum.Omiga2_Sum = PowerCtrl_Info.Measure.Omiga_2[0] + PowerCtrl_Info.Measure.Omiga_2[1] +
                                    PowerCtrl_Info.Measure.Omiga_2[2] + PowerCtrl_Info.Measure.Omiga_2[3];

    PowerCtrl_Info.Sum.power_useful_Sum = PowerCtrl_Info.Measure.power_useful[0] + PowerCtrl_Info.Measure.power_useful[1] +
                                          PowerCtrl_Info.Measure.power_useful[2] +PowerCtrl_Info.Measure.power_useful[3];
    PowerCtrl_Info.Sum.input_Sum = PowerCtrl_Info.Measure.RLS_Input[0] + PowerCtrl_Info.Measure.RLS_Input[1] +
                                   PowerCtrl_Info.Measure.RLS_Input[2] + PowerCtrl_Info.Measure.RLS_Input[3];
    // 总功率预测 = 各电机功率和 + 固定损耗K3

    RLS_Power_Info.Data.X[0] = PowerCtrl_Info.Sum.Omiga2_Sum;
    RLS_Power_Info.Data.X[1] = PowerCtrl_Info.Sum.Torque2_Sum;

    // RLS期望输出：模型预测功率
    RLS_Power_Info.Data.U[0] = PowerCtrl_Info.Sum.input_Sum ;
    // RLS实际输出：功率计测量的底盘实际功率 + 偏移量3W
    RLS_Power_Info.Data.Y[0] = ina226_power - PowerCtrl_Info.Sum.power_useful_Sum - PowerCtrl_Info.Param.K3;

    if (PowerCtrl_Info.Sum.Omiga2_Sum > 50.0f && update_weights)//防止静止时也拟合,导致拟合发散
    {
        // 更新RLS滤波器权重参数
        RLS_Update(&RLS_Power_Info);

        // 获取更新后的功率模型参数
        PowerCtrl_Info.Param.K1 = RLS_Power_Info.Data.W[0];
        PowerCtrl_Info.Param.K2 = RLS_Power_Info.Data.W[1];

        // 使用新参数重新计算总功率预测
        PowerCtrl_Info.Sum.Power_Sum =PowerCtrl_Info.Param.K1 * PowerCtrl_Info.Sum.Omiga2_Sum +
                                      PowerCtrl_Info.Param.K2 * PowerCtrl_Info.Sum.Torque2_Sum + PowerCtrl_Info.Param.K3 + PowerCtrl_Info.Sum.power_useful_Sum;
    }

/*-------------------------更新RLS拟合部分--------------------------*/


/*-----------------------------功率分配部分-----------------------*/
    powerOverloadFlag = 0;  //清除超功率标志位
    if (PowerCtrl_Info.Sum.Power_Sum >= PowerCtrl_Info.Power_Max)
    {
        powerOverloadFlag = 1;  //开启超功率标志位
    // 计算功率分配因子K（基于误差大小的自适应权重）
    if (PowerCtrl_Info.Sum.Err_Sum > PowerCtrl_Info.Param.Err_Upper)
        PowerCtrl_Info.K = 1;  // 误差大，完全按误差分配
    else if (PowerCtrl_Info.Sum.Err_Sum < PowerCtrl_Info.Param.Err_Lower)
        PowerCtrl_Info.K = 0;  // 误差小，完全按功率分配
    else
        // 误差中等,线性插值分配
        PowerCtrl_Info.K =
                fmaxf(0.0f,
                      fminf(1.0f,
                            (PowerCtrl_Info.Sum.Err_Sum - PowerCtrl_Info.Param.Err_Lower) /
                            (PowerCtrl_Info.Param.Err_Upper - PowerCtrl_Info.Param.Err_Lower)
                      )
                );

    // 计算每个电机的功率分配权重（港科大论文的功率分配方法）
    for (int i = 0; i < 4; i++)
    {
        // 1. 计算每个电机的权重（0-1之间）
        float error_weight = PowerCtrl_Info.Power_Max * (PowerCtrl_Info.Err[i] / PowerCtrl_Info.Sum.Err_Sum);
        float power_weight = PowerCtrl_Info.Power_Max * ((PowerCtrl_Info.Measure.RLS_Input[i] +
                              PowerCtrl_Info.Measure.power_useful[i] +
                              PowerCtrl_Info.Param.K3 * 0.25f) / PowerCtrl_Info.Sum.Power_Sum);
        PowerCtrl_Info.Power_Limit[i] = PowerCtrl_Info.K * error_weight + (1.0f - PowerCtrl_Info.K) * power_weight;
    }

        // 计算功率衰减系数：功率限制/实际功率，限制在[0,1]范围
        Decrease = PowerCtrl_Info.Power_Max / PowerCtrl_Info.Sum.Power_Sum;
        VAL_LIMIT(Decrease, 0, 1);

        // 对每个电机进行功率限制计算
        for (int i = 0; i < 4; i++)
        {
            /* 单个电机求解功率方程：P_limit = K1*ω² + K2*τ² + τ*ω + K3/4
               整理为二次方程：K1*ω² + ω*τ + (K2*τ² + K3/4 - P_limit) = 0
               标准形式：A*τ² + B*τ + C = 0
            */
            PowerCtrl_Info.A = PowerCtrl_Info.Param.K2;  // 二次项系数
            PowerCtrl_Info.B = (float)motor_target_speed_rad[i];  // 一次项系数（角速度）
            PowerCtrl_Info.C = (float)motor_target_speed_rad[i] * (float)motor_target_speed_rad[i] * PowerCtrl_Info.Param.K1 +
                               PowerCtrl_Info.Param.K3 * 0.25f - PowerCtrl_Info.Power_Limit[i];  // 常数项

            // 计算判别式Δ = B² - 4AC
            PowerCtrl_Info.Delta = powf(PowerCtrl_Info.B, 2.f) - 4 * PowerCtrl_Info.A * PowerCtrl_Info.C;

            // 检查判别式是否有效
            if (isnan(PowerCtrl_Info.Delta) == 1 || isinf(PowerCtrl_Info.Delta) == 1)
                PowerCtrl_Info.Delta = 0;  // 无效值处理

            // 根据判别式大小求解转矩
            if (PowerCtrl_Info.Delta >= 0)  // 有实数解
            {
                PowerCtrl_Info.Sqrt = sqrtf(PowerCtrl_Info.Delta);

                if (I_cmd[i] >= 0.0f)  // 正转情况
                {
                    // 取正根解
                    PowerCtrl_Info.Torque[i] = (-PowerCtrl_Info.B + PowerCtrl_Info.Sqrt) / (2 * PowerCtrl_Info.A);
                    // 转矩转电流，并应用功率衰减
                    PowerCtrl_Info.Output[i] = (PowerCtrl_Info.Torque[i] / CURRENT_TO_TORQUE_RATIO * Decrease);
                }
                else  // 反转情况
                {
                    // 取负根解
                    PowerCtrl_Info.Torque[i] = (-PowerCtrl_Info.B - PowerCtrl_Info.Sqrt) / (2 * PowerCtrl_Info.A);
                    PowerCtrl_Info.Output[i] = ((PowerCtrl_Info.Torque[i] / CURRENT_TO_TORQUE_RATIO) * Decrease);
                }
            }
            else  // 无实数解，使用近似解
            {
                if (I_cmd[i] >= 0)
                {
                    // 使用顶点近似：τ = -B/2A
                    PowerCtrl_Info.Torque[i] = (-PowerCtrl_Info.B) / (2.0f * PowerCtrl_Info.A);
                    PowerCtrl_Info.Output[i] = ((PowerCtrl_Info.Torque[i] / CURRENT_TO_TORQUE_RATIO) * Decrease);
                }
                else
                {
                    PowerCtrl_Info.Torque[i] = (PowerCtrl_Info.B) / (2.0f * PowerCtrl_Info.A);
                    PowerCtrl_Info.Output[i] = ((PowerCtrl_Info.Torque[i] / CURRENT_TO_TORQUE_RATIO) * Decrease);
                }
            }
            VAL_LIMIT(PowerCtrl_Info.Output[i], -2300, 2300);//限幅防止跑飞
        }
    }

}

/**
 * @brief 功率控制系统初始化
 * @param PowerCtrl_Info 功率控制信息结构体指针
 * @param Lamda RLS遗忘因子（0-1，值越大记忆越长）
 * @param P RLS协方差矩阵初始值
 * @param PowerCtrl_Param 功率控制参数数组
 * @note 初始化RLS滤波器和功率控制参数
 */
void PowerCtrl_Init(PowerCtrl_Typedef *Power_Info, float Lamda, float P, float PowerCtrlParam[5])
{
    // 初始化RLS滤波器：2个输入(转矩平方和,角速度平方和)，1个输出(功率)
    RLS_Init(&RLS_Power_Info, 2, 1, Lamda, P);

    // 设置功率模型参数：K1-转速损耗系数，K2-转矩损耗系数，K3-固定损耗
    Power_Info->Param.K1 = PowerCtrlParam[0];
    Power_Info->Param.K2 = PowerCtrlParam[1];
    Power_Info->Param.K3 = PowerCtrlParam[2];

    // 设置误差阈值：用于功率分配权重计算
    Power_Info->Param.Err_Lower = PowerCtrlParam[3];  // 误差下限
    Power_Info->Param.Err_Upper = PowerCtrlParam[4];  // 误差上限

    // 初始化RLS权重参数
    RLS_Power_Info.Data.W[0] = Power_Info->Param.K1;
    RLS_Power_Info.Data.W[1] = Power_Info->Param.K2;

}

#endif
/*-----------------------------功率分配部分-----------------------*/

/* -------------------------------- 线程入口 ------------------------------- */
void ChassisTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    chassis_motor_init();
    bsp_can_init();
    can_filter_init();
    chassis_yaw_pid = pid_register(&chassis_yaw_config);   /* 注册 PID 实例 */

#ifdef RLS_POWER_LIMIT
    PowerCtrl_Init(&PowerCtrl_Info,0.99999f,1e-5f,Power_Ctrl_Param);
#endif

/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
    chassis_pub_init();
    chassis_sub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    chassis_task_dt = dwt_get_delta(&chassis_task_dwt);
    chassis_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        chassis_task_delta = dwt_get_time_ms() - chassis_task_start_dt;
        chassis_task_start_dt = dwt_get_time_ms();
        chassis_task_dt = dwt_get_delta(&chassis_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
        chassis_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
        mecanum_calc(&cmd_chassis, motor_target_speed_rad);
#ifdef RLS_POWER_LIMIT
        static float last_power_timestamp = 0.0f;  // 记录上次RLS更新权重时的时间戳
        uint8_t should_update_weights = 0;
        // 通过比较时间戳判断功率数据是否更新
        if (power_update_timestamp != last_power_timestamp)
        {
            should_update_weights = 1;  // 真实功率数据已更新，需要更新RLS权重
            last_power_timestamp = power_update_timestamp;
        }
        // 每个周期都执行,但只在功率数据更新和运动时才更新权重系数
        rls_power_limit(should_update_weights);
#endif
        dji_motor_control();
/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        chassis_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(1);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/**
 * @brief chassis 线程中所有发布者初始化
 */
static void chassis_pub_init(void)
{
    pub_chassis = pub_register("cmd_cha_pub",sizeof(struct cmd_chassis_msg));
}

/**
 * @brief chassis 线程中所有订阅者初始化
 */
static void chassis_sub_init(void)
{
    sub_ins = sub_register("ins_pub", sizeof(struct ins_msg));
}

/**
 * @brief chassis 线程中所有发布者推送更新话题
 */
static void chassis_pub_push(void)
{
    pub_push_msg(pub_chassis,&cmd_chassis);
}

/**
 * @brief chassis 线程中所有订阅者获取更新话题
 */
static void chassis_sub_pull(void)
{
    sub_get_msg(sub_ins, &ins_data);
}
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */