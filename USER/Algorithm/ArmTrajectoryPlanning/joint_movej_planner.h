//
// Created by 刘嘉俊 on 26-3-26.
//

#ifndef CTRBOARD_H7_ALL_JOINT_MOVEJ_PLANNER_H
#define CTRBOARD_H7_ALL_JOINT_MOVEJ_PLANNER_H



#include <stdint.h>
#include <stdbool.h>
#include "traj_scalar_trap.h"

#define JOINT_NUM       6
#define JOINT_EPS_MOVE  1e-6f

typedef enum
{
    JP_IDLE = 0,
    JP_RUNNING,
    JP_DONE,
    JP_ABORT,
    JP_FAULT
} JointPlannerState_e;

typedef struct
{
    float q0[JOINT_NUM];
    float q1[JOINT_NUM];
    float dq[JOINT_NUM];

    float vmax[JOINT_NUM];
    float amax[JOINT_NUM];

    float q_ref[JOINT_NUM];
    float v_ref[JOINT_NUM];

    float vs_max;
    float as_max;

    uint8_t moving_joint_count;

    ScalarTrapProfile_t prof;
    JointPlannerState_e state;
} JointMoveJPlanner_t;

/* 初始化 */
void JointMoveJ_Init(JointMoveJPlanner_t *jp);

/* 启动一次 MoveJ 规划 */
bool JointMoveJ_Start(JointMoveJPlanner_t *jp,
                      const float q0[JOINT_NUM],
                      const float q1[JOINT_NUM],
                      const float vmax[JOINT_NUM],
                      const float amax[JOINT_NUM]);

/* 周期更新，dt 单位：秒 */
bool JointMoveJ_Update(JointMoveJPlanner_t *jp, float dt);

/* 强制停止 */
void JointMoveJ_Stop(JointMoveJPlanner_t *jp);


#endif //CTRBOARD_H7_ALL_JOINT_MOVEJ_PLANNER_H
