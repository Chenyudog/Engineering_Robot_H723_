//
// Created by 刘嘉俊 on 26-3-26.
//

#ifndef CTRBOARD_H7_ALL_TRAJECTORYPLANNING_H
#define CTRBOARD_H7_ALL_TRAJECTORYPLANNING_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "joint_movej_planner.h"

// 添加实际关节电机控制接口
#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"

/* 你可以根据自己的工程替换这两个类型 */
typedef struct
{
    motor_t *motor;  // 电机对象，包含电机ID、控制模式等信息
    FDCAN_HandleTypeDef *hcan;            // 控制电机的can总线
} JointMotorObj_t;

/* 运动执行状态 */
typedef enum
{
    JMOVE_IDLE = 0,  // 空闲
    JMOVE_RUNNING,   // 运动中
    JMOVE_DONE,      // 运动完成
    JMOVE_ABORT,     // 运动中断
    JMOVE_FAULT      // 运动故障或执行失败
} ArmTrajectoryMotionState_e;

/* 运动执行器 */
typedef struct
{
    JointMoveJPlanner_t planner;

    float last_q_cmd[JOINT_NUM];
    float last_v_cmd[JOINT_NUM];

    ArmTrajectoryMotionState_e state;
} ArmTrajMotion_t;

/* 初始化 */
void ArmTrajectoryPlanner_Init(ArmTrajMotion_t *arm);

/* 启动一次关节空间目标点到达 */
bool ArmMotion_StartJointMove(ArmTrajMotion_t *arm,
                                const float q_current[JOINT_NUM],
                                const float q_target[JOINT_NUM],
                                const float vmax[JOINT_NUM],
                                const float amax[JOINT_NUM],
                                const JointLimit_t *limit);

/* 周期执行：dt单位秒 */
bool ArmMotion_Tick(ArmTrajMotion_t *arm,
                      void *hcan,
                      const JointMotorObj_t motor[JOINT_NUM],
                      float dt);

/* 停止 */
void ArmMotion_Stop(ArmTrajMotion_t *rm);

/* 查询是否忙 */
bool ArmMotion_IsBusy(const ArmTrajMotion_t *rm);

#endif //CTRBOARD_H7_ALL_TRAJECTORYPLANNING_H
