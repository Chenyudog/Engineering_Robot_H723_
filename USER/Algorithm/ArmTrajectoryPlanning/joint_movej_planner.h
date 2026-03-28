//
// Created by 刘嘉俊 on 26-3-26.
//

#ifndef CTRBOARD_H7_ALL_JOINT_MOVEJ_PLANNER_H
#define CTRBOARD_H7_ALL_JOINT_MOVEJ_PLANNER_H

#include <stdint.h>
#include <stdbool.h>
#include "trajectory_time_synchronization.h"
#include "KinematicSDHdsp.h"

#define JOINT_NUM       6         // 机械臂关节数量，这里固定为 6 轴
#define JOINT_EPS_MOVE  1e-6f     // 判断“关节是否真的需要运动”的小阈值

/* MoveJ 规划器的运行状态 */
typedef enum
{
    JP_IDLE = 0,   // 空闲：规划器刚初始化，还没有启动轨迹
    JP_RUNNING,    // 运行中：正在执行一条 MoveJ 轨迹
    JP_DONE,       // 已完成：轨迹已经到达终点
    JP_ABORT,      // 已中止：轨迹被外部强制停止
    JP_FAULT       // 故障：参数错误、限位错误、内部轨迹启动失败等
} JointPlannerState_e;

/* 关节空间 MoveJ 规划器对象 */
typedef struct
{
    float q0[JOINT_NUM];      // 起点关节角（单位一般为 rad）
    float q1[JOINT_NUM];      // 终点关节角
    float dq[JOINT_NUM];      // 每个关节的总位移 dq = q1 - q0

    float vmax[JOINT_NUM];    // 每个关节各自的最大速度约束
    float amax[JOINT_NUM];    // 每个关节各自的最大加速度约束

    float q_ref[JOINT_NUM];   // 当前周期输出的关节参考位置
    float v_ref[JOINT_NUM];   // 当前周期输出的关节参考速度

    float vs_max;             // 统一标量进度 s 的最大速度约束
    float as_max;             // 统一标量进度 s 的最大加速度约束

    uint8_t moving_joint_count; // 实际参与这次运动的关节数量（位移不为 0 的关节数）

    SpeedTimeSYNC_t prof;     // 内部“时间同步器 / 标量进度轨迹”对象
    // 用来生成统一进度 s(t) 和 sdot(t)

    JointPlannerState_e state; // 当前规划器状态：空闲 / 运行 / 完成 / 中止 / 故障
} JointMoveJPlanner_t;

/* 初始化 */
void JointMoveJ_Init(JointMoveJPlanner_t *jp);

/* 启动一次 MoveJ 规划 */
bool JointMoveJ_Start(JointMoveJPlanner_t *jp,
                      const float q0[JOINT_NUM],
                      const float q1[JOINT_NUM],
                      const float vmax[JOINT_NUM],
                      const float amax[JOINT_NUM],
                      const JointLimit_t *limit);

/* 周期更新，dt 单位：秒 */
bool JointMoveJ_Update(JointMoveJPlanner_t *jp, float dt);

/* 强制停止 */
void JointMoveJ_Stop(JointMoveJPlanner_t *jp);

#endif // CTRBOARD_H7_ALL_JOINT_MOVEJ_PLANNER_H