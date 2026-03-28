//
// Created by 刘嘉俊 on 26-3-26.
// //

#include "TrajectoryPlanning.h"
#include <string.h>
/**
************************************************************************
* @brief:      	pos_speed_ctrl: 位置速度控制函数
* @param[in]:   hcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
//extern void pos_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel);

extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

JointMotorObj_t joint_motor[JOINT_NUM] =
        {
                { .motor = &motor[Motor1], .hcan = &hfdcan3 },
                { .motor = &motor[Motor2], .hcan = &hfdcan3 },
                { .motor = &motor[Motor3], .hcan = &hfdcan2 },
                { .motor = &motor[Motor4], .hcan = &hfdcan2 },
                { .motor = &motor[Motor5], .hcan = &hfdcan2 },
                { .motor = &motor[Motor6], .hcan = &hfdcan2 }
        };
/**
 ************************************************************************
 * @brief:       关节位置单位转换：弧度 -> 驱动器位置单位
 * @param[in]:   joint_idx: 关节下标（0~5）
 * @param[in]:   q_rad:     规划器输出的关节位置，单位 rad
 * @retval:      驱动器使用的位置单位
 * @details:
 * 1. 当前实现默认驱动器也使用弧度，因此直接返回 q_rad
 * 2. 如果你的驱动器位置单位不是弧度，例如“度”或“编码器计数”，
 *    可以在这里统一做转换
 ************************************************************************
 **/
static float JointPos_RadToDrvUnit(uint8_t joint_idx, float q_rad)
{
    (void)joint_idx;
    return q_rad;
}

/**
 ************************************************************************
 * @brief:       关节速度单位转换：弧度/秒 -> 驱动器速度单位
 * @param[in]:   joint_idx: 关节下标（0~5）
 * @param[in]:   v_radps:   规划器输出的关节速度，单位 rad/s
 * @retval:      驱动器使用的速度单位
 * @details:
 * 1. 当前实现默认驱动器也使用 rad/s，因此直接返回 v_radps
 * 2. 如果你的驱动器速度单位不是 rad/s，例如“度/秒”或“rpm”，
 *    可以在这里统一做转换
 ************************************************************************
 **/
static float JointVel_RadpsToDrvUnit(uint8_t joint_idx, float v_radps)
{
    (void)joint_idx;
    return v_radps;
}

/**
 ************************************************************************
 * @brief:       机械臂轨迹执行器初始化
 * @param[in]:   arm: 轨迹执行器对象
 * @retval:      无
 * @details:
 * 1. 清空整个 ArmTrajMotion_t 结构体
 * 2. 初始化内部的关节空间 MoveJ 规划器
 * 3. 将执行状态置为 JMOVE_IDLE
 ************************************************************************
 **/
void ArmTrajectoryPlanner_Init(ArmTrajMotion_t *arm)
{
    if (arm == 0)
    {
        return;
    }

    memset(arm, 0, sizeof(ArmTrajMotion_t));
    JointMoveJ_Init(&arm->planner);
    arm->state = JMOVE_IDLE;
}

/**
 ************************************************************************
 * @brief:       启动一次关节空间目标点运动
 * @param[in]:   arm:        轨迹执行器对象
 * @param[in]:   q_current:  当前关节角（起点）
 * @param[in]:   q_target:   目标关节角（终点）
 * @param[in]:   vmax:       每个关节的最大速度约束
 * @param[in]:   amax:       每个关节的最大加速度约束
 * @param[in]:   limit:      关节限位
 * @retval:      true=启动成功，false=启动失败
 * @details:
 * 1. 内部调用 JointMoveJ_Start() 启动关节空间同步轨迹
 * 2. 成功后缓存最近一次命令
 * 3. 根据内部规划器状态同步更新整体执行状态
 ************************************************************************
 **/
bool ArmMotion_StartJointMove(ArmTrajMotion_t *arm,
                              const float q_current[JOINT_NUM],
                              const float q_target[JOINT_NUM],
                              const float vmax[JOINT_NUM],
                              const float amax[JOINT_NUM],
                              const JointLimit_t *limit)
{
    int i;

    if ((arm == 0) || (q_current == 0) || (q_target == 0) || (vmax == 0) || (amax == 0))
    {
        return false;
    }

    if (!JointMoveJ_Start(&arm->planner, q_current, q_target, vmax, amax, limit))
    {
        arm->state = JMOVE_FAULT;
        return false;
    }

    for (i = 0; i < JOINT_NUM; i++)
    {
        arm->last_q_cmd[i] = q_current[i];
        arm->last_v_cmd[i] = 0.0f;
    }

    if (arm->planner.state == JP_DONE)
    {
        arm->state = JMOVE_DONE;
    }
    else
    {
        arm->state = JMOVE_RUNNING;
    }

    return true;
}

/**
 ************************************************************************
 * @brief:       周期执行一次机械臂轨迹
 * @param[in]:   arm:   轨迹执行器对象
 * @param[in]:   hcan:  保留参数，为兼容当前头文件；实际发送时不使用
 * @param[in]:   motor: 6个关节的电机对象数组，每个对象包含电机和所属CAN总线
 * @param[in]:   dt:    控制周期，单位秒
 * @retval:      true=执行正常，false=执行失败
 * @details:
 * 1. 推进内部 JointMoveJ 规划器一个周期
 * 2. 读取每个关节当前周期的 q_ref / v_ref
 * 3. 做单位转换
 * 4. 使用“每个关节自己的 hcan 总线”发送位置速度命令
 *
 * 注意：
 * - 这里不再使用统一总线 hcan
 * - joint1、joint2 可以绑定 fdcan3
 * - joint3、joint4、joint5、joint6 可以绑定 fdcan2
 * - 具体由外部传入的 motor[i].hcan 决定
 ************************************************************************
 **/
bool ArmMotion_Tick(ArmTrajMotion_t *arm,
                    void *hcan,
                    const JointMotorObj_t motor[JOINT_NUM],
                    float dt)
{
    int i;
    float q_cmd_drv;
    float v_cmd_drv;

    /* 当前版本中不再使用统一hcan参数，仅保留为了兼容头文件 */
    (void)hcan;

    if ((arm == 0) || (motor == 0) || (dt <= 0.0f))
    {
        return false;
    }

    if (arm->state == JMOVE_IDLE)
    {
        return true;
    }

    if ((arm->state == JMOVE_ABORT) || (arm->state == JMOVE_FAULT))
    {
        return false;
    }

    if (arm->state == JMOVE_DONE)
    {
        return true;
    }

    if (!JointMoveJ_Update(&arm->planner, dt))
    {
        arm->state = JMOVE_FAULT;
        return false;
    }

    for (i = 0; i < JOINT_NUM; i++)
    {
        /* 判空保护：每个关节必须绑定电机对象和总线 */
        if ((motor[i].motor == 0) || (motor[i].hcan == 0))
        {
            arm->state = JMOVE_FAULT;
            return false;
        }

        /* 保存本周期规划器输出 */
        arm->last_q_cmd[i] = arm->planner.q_ref[i];
        arm->last_v_cmd[i] = arm->planner.v_ref[i];

        /* 单位转换 */
        q_cmd_drv = JointPos_RadToDrvUnit((uint8_t)i, arm->planner.q_ref[i]);

        /* 位置-速度模式里速度通常作为限速值，取绝对值 */
        v_cmd_drv = JointVel_RadpsToDrvUnit((uint8_t)i, fabsf(arm->planner.v_ref[i]));

        /* 每个关节走各自绑定的FDCAN总线 */
        pos_ctrl(motor[i].hcan, motor[i].motor->id, q_cmd_drv, v_cmd_drv);
    }

    if (arm->planner.state == JP_DONE)
    {
        arm->state = JMOVE_DONE;
    }
    else
    {
        arm->state = JMOVE_RUNNING;
    }

    return true;
}

/**
 ************************************************************************
 * @brief:       停止当前运动
 * @param[in]:   arm: 轨迹执行器对象
 * @retval:      无
 * @details:
 * 1. 停止内部 JointMoveJ 规划器
 * 2. 将最近一次速度命令缓存清零
 * 3. 将整体状态置为 JMOVE_ABORT
 ************************************************************************
 **/
void ArmMotion_Stop(ArmTrajMotion_t *arm)
{
    int i;

    if (arm == 0)
    {
        return;
    }

    JointMoveJ_Stop(&arm->planner);

    for (i = 0; i < JOINT_NUM; i++)
    {
        arm->last_v_cmd[i] = 0.0f;
    }

    arm->state = JMOVE_ABORT;
}

/**
 ************************************************************************
 * @brief:       查询轨迹执行器是否忙
 * @param[in]:   arm: 轨迹执行器对象
 * @retval:      true=正在运行，false=非运行状态
 ************************************************************************
 **/
bool ArmMotion_IsBusy(const ArmTrajMotion_t *arm)
{
    if (arm == 0)
    {
        return false;
    }

    return (arm->state == JMOVE_RUNNING);
}