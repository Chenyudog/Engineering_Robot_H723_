//
// Created by 刘嘉俊 on 26-4-2.
//

#ifndef CTRBOARD_H7_ALL_ALGO_IK_PLAN_H
#define CTRBOARD_H7_ALL_ALGO_IK_PLAN_H

#include "KinematicSDHdsp.h"
#include "joint_movej_planner.h"   // 里面会包含 trajectory_time_synchronization.h

/* ---------------------------- 对外输入：关节反馈 ---------------------------- */
typedef struct
{
    float q_fb[JOINT_NUM];
    float v_fb[JOINT_NUM];
    uint8_t valid;
} AlgoFeedback_t;

typedef enum
{
    ALGO_CMD_IDLE = 0,
    ALGO_CMD_PENDING,
    ALGO_CMD_IK_OK,
    ALGO_CMD_IK_FAILED,
    ALGO_CMD_MOVEJ_STARTED,
    ALGO_CMD_MOVEJ_START_FAILED
} AlgoCmdState_e;


/* ---------------------------- 对外输出：规划结果 ---------------------------- */
typedef struct
{
    uint8_t valid;          /* q_ref/v_ref 当前是否有效 */
    uint8_t planner_state;  /* JP_IDLE / JP_RUNNING / JP_DONE / JP_FAULT */

    uint32_t cmd_seq;       /* 最新命令序号 */
    uint8_t  cmd_state;     /* AlgoCmdState_e */

    float q_ref[JOINT_NUM];
    float v_ref[JOINT_NUM];

    float q_fb[JOINT_NUM];
    float v_fb[JOINT_NUM];
} AlgoOutput_t;

/* ---------------------------- 对外接口 ---------------------------- */

/**
 * @brief 初始化算法上下文（只调用一次）
 */
void Algo_InitContext(void);

/**
 * @brief 设置最新反馈（由任务线程把Topic数据转换后喂进来）
 * @param fb 关节反馈
 */
void Algo_SetFeedback(const AlgoFeedback_t *fb);

/**
 * @brief 投递一个目标Pose
 * @param pose_target 目标位姿
 * @return true 投递成功
 * @return false 投递失败
 */
bool Algo_PostPoseTarget(const Pose6D_t *pose_target);

/**
 * @brief 直接使用XYZ + ROLL/YAW/PITCH(rad)投递目标Pose
 */
bool Algo_PostPoseTargetXYZRYP_Rad(float x, float y, float z,
                                   float roll, float yaw, float pitch);

/**
 * @brief 算法层步进一次
 * @param dt 当前任务线程实际dt（秒）
 */
void Algo_Step(float dt);

/**
 * @brief 获取当前规划输出
 * @param out 输出结构体
 */
void Algo_GetOutput(AlgoOutput_t *out);


#endif //CTRBOARD_H7_ALL_ALGO_IK_PLAN_H
