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
#include "bsp_log.h"
#include "Power_task.h"

#define RLS_POWER_LIMIT
#define K_power 0.10472f//  rpm -> rad/s
#define wheel_ratio    0.052074f  //转子转速转换成轮子转速   1/减速比 ≈ 1/19 =0.052074
#define Icmd_2_current 0.0012207f   //  20 / 16384
#define K_torque  0.3f        //转矩常数

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
int16_t motor_target_current_look[4];

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
        motor_max_current = 5000;
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
        motor_max_current = 5000;
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
        motor_max_current = 5000;
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
        motor_max_current = 5000;
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

static bool floatEqual(float a, float b) { return fabsf(a - b) < 1e-5f; }  //近似a大返回0，b大返回1
#ifdef RLS_POWER_LIMIT

RLS_Info_TypeDef rls_filter;
int16_t I_cmd[4];//PID计算出来的要发送的电流
float omega[4];//转子转速 单位：rad/s
float omega_cmd[4];
float omega2_all;//转子转速总和 单位：rad/s
float omega_cmd_all;
float power[4],power_all,power_useful[4],power_useful_all;//单个电机功率，四个电机功率总和，单个电机有用功，四个电机有用功总和
float rotor_torque[4];  //力矩
//RLS拟合和底盘限制变量
float rotor_torque2_all;  //转矩平方总和
float rotor_torque2_all_cmd;  //转矩平方总和
float wasted_power_all;   //无用功总和
float32_t input_vector[2]; //rls拟合输入量
float rls_error;   //用于RLS拟合
float power_w[2];  //拟合输出量
float k1,k2,k3 = 2.8f;    //拟合量
float power_cmd[4];  //单个电机分配得功率
float power_max = 25;  //最大功率//测试用
float torque_cmd[4];  //目标力矩
uint8_t power_out_state = 0;  //超功率标志位
float32_t sqrt_input = 0.0f ;  //detal
float32_t rotor_torque_cmd[4];
float measure_error[4] ;   //每个电机的error值
float sum_error = 0.0f;
float powerWeight_Error = 0.0f;
float powerWeight_Prop = 0.0f;
float error_powerDistribution_set = 20.0f;
float prop_powerDistribution_set = 30.0f;
float Power_In[4];
/**
 * @brief RLS功率限制计算函数
 * @param update_weights 是否更新RLS权重系数 (1=更新, 0=不更新)
 * @note 每个周期都需要计算功率限制，但只在裁判系统功率数据更新时才更新权重
 */
/**
 * @brief 基于递推最小二乘(RLS)的功率限制函数
 * @param update_weights 是否更新RLS算法的权重参数（1表示更新，0表示不更新）
 * @note 功能：通过RLS算法实时估计电机功率损耗模型，当总总功率超过限制时进行功率分配，防止超功率
 */
void rls_power_limit(uint8_t update_weights) {
    dji_motor_object_t *motor;       // 电机对象指针,用于获取电机信息
    dji_motor_measure_t measure;     // 电机测量数据结构体（包含转速等信息）

    omega2_all = 0;                     // 所有电机转速绝对值之和（用于判断运动状态）
    omega_cmd_all = 0;                  // 所有电机目标转速绝对值之和（用于判断运动状态）
    rotor_torque2_all = 0;             // 所有电机转矩平方和（用于损耗模型）
    rotor_torque2_all_cmd = 0;          // 所有电机目标转矩平方和（用于损耗模型）
    power_useful_all = 0;              // 所有电机有用功率总和
    power_all = 0;                     // 所有电机总功率（有用功率+损耗）总和
    power_out_state = 0;               //超功率标志位
    sum_error = 0.0f;                  //误差总和,用来分配功率
/*-------------------------更新RLS拟合部分--------------------------*/
    // 遍历4个底盘电机，计算单电机参数并累积总和
    for (int i = 0; i < 4; i++) {
        motor = chassis_motor[i];    // 获取第i个电机的对象
        measure = motor->measure;    // 获取该电机的实时测量数据
        I_cmd[i] = motor->control(measure);// 计算电机的电流指令（通过pid获取本次计算出来的目标电流值）

        omega[i] = measure.speed_rad;
        rotor_torque[i] = (float) measure.real_current * Icmd_2_current * K_torque * wheel_ratio;//转子实际转矩
        rotor_torque_cmd[i] = I_cmd[i] * Icmd_2_current * K_torque * wheel_ratio;               //计算出来本次电流值对应的转矩
        rotor_torque2_all += rotor_torque[i] * rotor_torque[i]; // 实际转矩平方和
        rotor_torque2_all_cmd += rotor_torque_cmd[i] * rotor_torque_cmd[i];
        power_useful[i] = rotor_torque[i] * omega[i];           //每个电机的有用功率
        power_useful_all += power_useful[i];                    //有用功率总和
        omega2_all += omega[i] * omega[i];                           // 转速绝对值总和（判断是否运动:静止时避免RLS拟合错误）
        omega_cmd[i] = motor_target_speed_rad[i];
        omega_cmd_all += fabsf(omega_cmd[i]);
        measure_error[i] = (float) abs(motor_target_speed_rad[i] - measure.speed_rad);  //单个电机误差值
        sum_error += measure_error[i];  //误差值总和，用于功率分配
        Power_In[i] = (k1 * omega[i] * omega[i] + k2 * rotor_torque[i] * rotor_torque[i]);
    }

    rls_filter.Data.X[0] = omega2_all;
    rls_filter.Data.X[1] = rotor_torque2_all;
    rls_filter.Data.U[0] = Power_In[0] + Power_In[1] + Power_In[2] + Power_In[3];
    rls_filter.Data.Y[0] = ina226_power - k3 - power_useful_all + 5;    // 总功率P

    if (omega2_all > 50.0f && update_weights) {
        RLS_Update(&rls_filter);

        k1 = rls_filter.Data.W[0];  // 学习到的k₁
        k2 = rls_filter.Data.W[1];  // 学习到的k₂

        power_all = Power_In[0] + Power_In[1] + Power_In[2] + Power_In[3] + k3 + power_useful_all;
    }

/*-------------------------更新RLS拟合部分--------------------------*/


/*-----------------------------功率分配部分-----------------------*/
    for (int i = 0; i < 4; i++)// 计算每个电机的总功率（有用功率+损耗），并累加总功率
    {
        // 总功率 = 有用功率 + 损耗功率
        // 损耗功率模型：k1*|转速| + k2*目标转矩² + k3/4（k3平均分配到4个电机）
        power[i] = Power_In[i] + k3/4.0f + power_useful[i];
    }
//    // 当总功率超过上限时，执行功率限制逻辑
    if (power_all > power_max) {
        float Decrease;  // 功率衰减系数
        Decrease = power_max / power_all;
        power_out_state = 1;  //控制是否直接发送给电机计算得来得力矩值
        float errorConfidence = 0.0f;
        if (sum_error > 500.0f) {
            errorConfidence = 1.0f;
        } else if (sum_error < 0.0001) {
            errorConfidence = 0.0f;
        } else {
            errorConfidence =
                    fmaxf(0.0f,
                          fminf(1.0f,
                                (sum_error - 0.0001f) /
                                (500.0f - 0.0001f)
                          )
                    );
        }

        for (int i = 0; i < 4; i++) {
            powerWeight_Error = power_max * (measure_error[i] / sum_error); //按误差error大小分配权重
            powerWeight_Prop = power_max * (power[i] / power_all);        //比例分配权重
            power_cmd[i] = errorConfidence * powerWeight_Error + (1.0f - errorConfidence) * powerWeight_Prop;

            // 功率方程：power_cmd = 有用功率 + 损耗功率
            // 代入有用功率 = 转矩×转速、损耗功率=k1*|转速| + k2*转矩² + k3/4
            // 整理得：k2*转矩² + 转速*转矩 + (k1*转速 + k3/4 - power_cmd) = 0
            // 以下为二次方程ax²+bx+c=0的判别式：b²-4ac
            sqrt_input = (omega_cmd[i] * omega_cmd[i]) -
                         (4.0f * k2 * (k1 * omega_cmd[i] * omega_cmd[i]) + k3 / 4.0f - power_cmd[i]);
            if (floatEqual(sqrt_input, 0.0f))//sqrt_input比0.0小
            {
                torque_cmd[i] = -omega_cmd[i] / 2.0f / k2;  //无解，近似于-B/2A
            }
            else if (sqrt_input > 0.0f)
            {
                if (I_cmd[i] > 0) {
                    torque_cmd[i] = (-omega_cmd[i] + sqrtf(sqrt_input)) / (2.0f * k2);
                } else {
                    torque_cmd[i] = (-omega_cmd[i] - sqrtf(sqrt_input)) / (2.0f * k2);
                }
            } else {
                torque_cmd[i] = -omega_cmd[i] / 2.0f / k2;  //唯一解，近似于-B/2A
            }
            VAL_LIMIT(torque_cmd[i], -0.2f, 0.2f);  //限幅防止跑飞
            motor_target_current_look[i] = (int16_t) (torque_cmd[i] / Icmd_2_current / K_torque / wheel_ratio * Decrease);//从力矩换到发送的电流值
            VAL_LIMIT(motor_target_current_look[i], -4000, 4000);//限幅防止跑飞
        }
    } else {
        motor_target_current_look[0] = 0;//如果没超功率，把先前计算的值归零,然后自动调用pid计算
        motor_target_current_look[1] = 0;
        motor_target_current_look[2] = 0;
        motor_target_current_look[3] = 0;
        torque_cmd[0] = 0;
        torque_cmd[1] = 0;
        torque_cmd[2] = 0;
        torque_cmd[3] = 0;
    }
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
    // 初始化RLS滤波器
    // 参数：滤波器实例指针, 输入向量大小, 输出向量大小, 遗忘因子, P矩阵初始值
    RLS_Init(&rls_filter, 2, 1, 0.99999f, 1e-5f);
    k1 = 1e-05f;
    k2 = 22.0f;
    k3 = 3.0f;
    rls_filter.Data.W[0] = k1;
    rls_filter.Data.W[1] = k2;
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
        /* RLS功率限制计算 - 每周期计算，但只在功率数据更新时更新权重 */
        static float last_power_timestamp = 0.0f;  // 记录上次RLS更新权重时的时间戳
        uint8_t should_update_weights = 0;

        // 通过比较时间戳判断功率数据是否更新
        if (power_update_timestamp != last_power_timestamp)
        {
            should_update_weights = 1;  // 功率数据已更新，需要更新RLS权重
            last_power_timestamp = power_update_timestamp;
        }

        // 每个周期都执行，但只在功率数据更新时才更新权重系数
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