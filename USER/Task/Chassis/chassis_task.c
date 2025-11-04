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

static int32_t motor_target_speed_rpm[4];
int32_t motor_target_speed_rpm_look[4];

static void chassis_motor_init();
static void mecanum_calc(struct cmd_chassis_msg *cmd, int32_t* out_speed);


/* --------------------------------- 电机控制相关 --------------------------------- */
#define CURRENT_POWER_LIMIT_RATE 80
static int32_t motor_control_0(dji_motor_measure_t measure)
{
    static int32_t motor_current_set = 0;
    static int32_t motor_max_current=0;   // 电流值范围：-16380~0~16380
    static int32_t chassis_power_limit=0;
    if (chassis_power_limit==0)
    {
        motor_max_current = 8000;
    }
    motor_current_set =(int32_t) pid_calculate(chassis_controller[0].speed_pid, measure.speed_rpm, motor_target_speed_rpm[0]);
    VAL_LIMIT(motor_current_set , -motor_max_current, motor_max_current);
    return motor_current_set;
}

static int32_t motor_control_1(dji_motor_measure_t measure)
{
    static int32_t motor_current_set = 0;
    static int32_t motor_max_current = 0;   // 电流值范围：-16380~0~16380,电流值限幅变量
    static int32_t chassis_power_limit = 0;
    if (chassis_power_limit==0)
    {
        motor_max_current = 8000;
    }
    motor_current_set =(int32_t) pid_calculate(chassis_controller[1].speed_pid, measure.speed_rpm, motor_target_speed_rpm[1]);
    VAL_LIMIT(motor_current_set , -motor_max_current, motor_max_current);
    return motor_current_set;
}

static int32_t motor_control_2(dji_motor_measure_t measure)
{
    static int32_t motor_current_set = 0;
    static int32_t motor_max_current = 0;   // 电流值范围：-16380~0~16380
    static int32_t chassis_power_limit = 0;
    if (chassis_power_limit==0)
    {
        motor_max_current = 8000;
    }
    motor_current_set =(int32_t) pid_calculate(chassis_controller[2].speed_pid, measure.speed_rpm, motor_target_speed_rpm[2]);
    VAL_LIMIT(motor_current_set , -motor_max_current, motor_max_current);
    return motor_current_set;
}

static int32_t motor_control_3(dji_motor_measure_t measure)
{
    static int32_t motor_current_set = 0;
    static int32_t motor_max_current = 0;   // 电流值范围：-16380~0~16380
    static int32_t chassis_power_limit = 0;
    if (chassis_power_limit==0)
    {
        motor_max_current = 8000;
    }
    motor_current_set =(int32_t) pid_calculate(chassis_controller[3].speed_pid, measure.speed_rpm, motor_target_speed_rpm[3]);
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

static void mecanum_calc(struct cmd_chassis_msg *cmd, int32_t* out_speed)
{
    // 轮子转速转换系数，转为轮子转速，为rpm/min，每分钟多少转,60是指60秒，转换成分钟，
    // 空载转速482rpm，3Nm满载最高转速469rpm
    // static float wheel_rpm_ratio = 60.0f * CHASSIS_MOTOR_REDUCTION_RATIO / WHEEL_PERIMETER ;
    int32_t wheel_rpm[4];  // 转换电机转子的期望转速，而非实际输出轮子的转速

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
    wheel_rpm[3] = (int32_t)(( -cmd->vx - cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO);    // 左後輪
    wheel_rpm[0] = (int32_t)(( -cmd->vx + cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO);    // 左前轮
    wheel_rpm[1] = (int32_t)(( +cmd->vx + cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO);     // 右前轮
    wheel_rpm[2] = (int32_t)(( +cmd->vx - cmd->vy + cmd->vw * ((WHEEL_TRACK + WHEEL_BASE)/2.0f)) * WHELL_RPM_RATIO);     // 右後輪

    memcpy(out_speed, wheel_rpm, 4*sizeof(int32_t));//copy the rpm to out_speed
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
            memset(motor_target_speed_rpm, 0, sizeof(motor_target_speed_rpm));
            break;
        default:
            // chassis_cmd_disable();
            break;
    }
}



#ifdef RLS_POWER_LIMIT
int32_t I_cmd[4];
static rls_arm_instance_f32 power_rls;
rls_arm_config_t cfg;
float omega[4];
float omega_all,I_cmd_all;
float power[4],power_all,power_useful[4],power_useful_all;

float rotor_torque[4];  //力矩
#define K_power 0.10472f//  rpm -> rad/s
#define wheel_ratio    0.052074f  //转换成轮子转速
#define RPM_SCALE 1e-07f
#define TORQUE_SCALE 0.1f
#define Icmd_2_current 0.0012207f   //  20 / 16384
#define K_torque 0.3f        //转矩常数
void rls_power_init()
{
    rls_arm_get_default_config(3, &cfg);
    cfg.lambda = 0.99999f;     //  遗忘因子：适中
    cfg.delta  = 20.0f;      //  初始协方差：略大，快速初始学习

    // 稳定性参数
    cfg.stability_threshold = 1e-5f;        //  稳定性阈值
    cfg.max_updates = 7500;               //  最大更新次数：~3.3分钟
    cfg.enable_adaptive_lambda = 0;        //  禁用自适应（保持稳定）
    cfg.enable_stability_check = 1;        //  启用稳定性检查

    /* 初始化实例 */
    arm_status status = rls_arm_init_f32(&power_rls, &cfg);
    if (status != ARM_MATH_SUCCESS)
    {
        LOGERROR("rls init failed: %d\n", status);
        return;
    }
}
float rotor_torque2_all;
float wasted_power_all;
float32_t input_vector[3];
float rls_error;
float power_w[3];
float k1,k2,k3;
float power_cmd[4];
float power_max;
float torque_cmd[4];
int out_power_flag=0;
float32_t look = 0.0f;

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
void rls_power_limit(uint8_t update_weights)
{
    dji_motor_object_t *motor;       // 电机对象指针,用于获取电机信息
    dji_motor_measure_t measure;     // 电机测量数据结构体（包含转速等信息）

    // 清零所有累积变量，避免上次计算结果对本次产生干扰
    omega_all = 0;                     // 所有电机转速绝对值之和（用于判断运动状态）
    rotor_torque2_all = 0;           // 所有电机转矩平方和（用于损耗模型）
    power_useful_all = 0;            // 所有电机有用功率总和
    power_all = 0;                   // 所有电机总功率（有用功率+损耗）总和

    // 遍历4个底盘电机，计算单电机参数并累积总和
    for (int i = 0; i < 4; i++)
    {
        motor = chassis_motor[i];    // 获取第i个电机的对象
        measure = motor->measure;    // 获取该电机的实时测量数据

        // 计算电机的电流指令（由电机控制算法输出，如PID控制）
        I_cmd[i] = motor->control(measure);

        // 将电流指令转换为电机转矩：
        // Icmd_2_current：电流指令到实际电流的转换系数（可能包含限幅/标定）
        // K_torque：电流到转矩的转换系数（电机参数，N·m/A）
        rotor_torque[i] = fabsf((float)measure.real_current )*Icmd_2_current * K_torque;

        // 计算电机的有用功率（机械功率）
        // 转矩 × 转速（需转换单位，K_power包含rpm到rad/s的换算及单位统一）
        power_useful[i] = fabsf(rotor_torque[i]) * fabsf(measure.speed_rpm * wheel_ratio * K_power);

        omega[i] = measure.speed_rpm * K_power * wheel_ratio;  // 保存当前电机的转速（rpm）

        // 累积计算损耗模型所需的变量
        rotor_torque2_all += rotor_torque[i] * rotor_torque[i];  // 转矩平方和（用于非线性损耗项）
        power_useful_all += power_useful[i];         // 有用功率总和
        omega_all += fabsf(omega[i]);  // 转速绝对值总和（判断是否运动：静止时避免RLS拟合错误）
    }

    // 当需要更新权重且系统处于有效运动状态时，执行RLS算法更新损耗模型参数
    if (update_weights)
    {
        // 计算总损耗功率：
        // 裁判系统反馈的总功率（reserved_3通常表示实际消耗的总功率）减去有用功率总和
        wasted_power_all = ina226_power - power_useful_all;

        // 构造RLS算法的输入向量（对应损耗模型的自变量）：
        // 输入向量为 [转速平方项, 转矩平方项, 常数项]，通过缩放使各分量数量级一致
        input_vector[0] = fabsf((float)omega_all);       // 转速平方项（缩放后）
        input_vector[1] = rotor_torque2_all;  // 转矩平方项（缩放后）
        input_vector[2] = 1.0f;                                 // 常数项（偏置项）

        // 运动状态判断：满足以下条件时认为系统在运动，可执行RLS更新
        // 1. 总转速绝对值>300rpm（整体运动）；2. 任一电机电流指令≥100（电机发力）
        // 目的：避免静止时数据无效导致RLS参数拟合发散

        if (omega_all > 10.0f)
        {
            // 调用RLS算法更新参数：根据输入向量和总损耗功率，优化模型权重
            rls_arm_control_f32(&power_rls, input_vector, wasted_power_all, &rls_error);
        }
    }
    // 从RLS算法中获取最新的权重参数（k1, k2, k3分别对应损耗模型的系数）
    rls_arm_get_weights_f32(&power_rls, power_w);
    // 还原缩放后的参数（与输入向量的缩放对应，确保单位正确）
    k1 = power_w[0] ;    // 转速平方项系数（对应风阻/涡流损耗）
    k2 = power_w[1] ; // 转矩平方项系数（对应非线性损耗）
    k3 = power_w[2];                // 常数项系数（对应固定损耗）

    // 计算每个电机的总功率（有用功率+损耗），并累加总功率
    for (int i = 0; i < 4; i++)
    {
        // 总功率 = 有用功率 + 损耗功率
        // 损耗功率模型：k1*转速² + k2*转矩² + k3/4（k3平均分配到4个电机）
        power[i] = power_useful[i] + (k1 * (fabsf(omega[i])) +
                                      k2 * rotor_torque[i] * rotor_torque[i] +
                                      k3 / 4.0f);
        power_all += power[i];  // 累加4个电机的总功率
    }

    // 确定功率上限：裁判系统限制的底盘功率减去4W余量（避免触发硬限制）
    power_max = 30.0f;

//    // 当总功率超过上限时，执行功率限制逻辑
//    if (power_all > power_max)
//    {
//        // 遍历电机，计算每个电机的限制后功率及对应的转矩指令
//        for (int i = 0; i < 4; i++)
//        {
//            // 按比例分配功率上限：每个电机的目标功率 = 总上限 × (该电机功率/总功率)
//            power_cmd[i] = power_max / power_all * power[i];
//
//            // 求解二次方程得到限制后的转矩指令：
//            // 功率方程：power_cmd = 有用功率 + 损耗功率
//            // 代入有用功率=转矩×转速×K_power、损耗功率=k1*转速² + k2*转矩² + k3/4
//            // 整理得：k2*转矩² + (K_power*转速)*转矩 + (k1*转速² + k3/4 - power_cmd) = 0
//            // 以下为二次方程ax²+bx+c=0的判别式：b²-4ac
//            float sqrt_input = (((float)(rpm[i] * rpm[i]) * K_power * K_power) -
//                    (4.0f * k2 * (k1 * K_power*(float)(rpm[i] * rpm[i]) + k3 / 4.0f - power_cmd[i])));
//            float sqrt_output;  // 判别式的平方根
//
//            // 计算平方根（使用ARM库函数，确保数值稳定性）
//            arm_status status = arm_sqrt_f32(sqrt_input, &sqrt_output);
//            // TODO：此处需补充错误处理（如判别式为负时的容错逻辑）
//            if (status != ARM_MATH_SUCCESS)
//            {
//                return;  // 平方根计算失败时退出，避免错误指令
//            }
//
//            // 根据电流指令的正负（决定转矩方向），求解二次方程的根
//            if (I_cmd[i] >= 0)  // 正向转矩（电流为正），取较小的根（避免过补偿）
//            {
//                torque_cmd[i] = (int32_t)((-K_power * (float)rpm[i] + sqrt_output) / (2.0f * k2));
//            }
//            else  // 反向转矩（电流为负），取较大的根
//            {
//                torque_cmd[i] = (int32_t)((-K_power * (float)rpm[i] - sqrt_output) / (2.0f * k2));
//            }
//
//            #ifdef RLS_POWER_LIMIT
//                motor_target_speed_rpm_look[i] = (int32_t)(torque_cmd[i]/Icmd_2_current/K_torque);
//            #endif
//        }
//    }

}

#endif


/* -------------------------------- 线程入口 ------------------------------- */
void ChassisTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    chassis_motor_init();
    bsp_can_init();
    can_filter_init();
    chassis_yaw_pid = pid_register(&chassis_yaw_config);   /* 注册 PID 实例 */
#ifdef RLS_POWER_LIMIT
    rls_power_init();
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
        mecanum_calc(&cmd_chassis, motor_target_speed_rpm);
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