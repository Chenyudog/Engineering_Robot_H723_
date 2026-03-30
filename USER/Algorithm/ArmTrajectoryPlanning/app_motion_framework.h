#ifndef APP_MOTION_FRAMEWORK_H
#define APP_MOTION_FRAMEWORK_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "KinematicSDHdsp.h"
#include "TrajectoryPlanning.h"


/* 轨迹默认容差，直接沿用你运动学头文件里的宏也可以 */
#define APP_IK_POS_TOL               POSITION_TOLERANCE
#define APP_IK_ORI_TOL               ORIENTATION_TOLERANCE

typedef enum
{
    ALGO_POSE_CMD_MOVEJ = 0,
    ALGO_POSE_CMD_STOP  = 1
} AlgoPoseCmdType_e;

typedef struct
{
    AlgoPoseCmdType_e type;
    uint32_t seq;

    Pose6D_t target_pose;

    /* frame6 -> TCP 偏置；若目标本身就是 frame6，则传 {0,0,0} */
    float wrist_offset[3];

    float vmax[JOINT_NUM];
    float amax[JOINT_NUM];
} AlgoPoseCmdMsg_t;

typedef enum
{
    MOTOR_CMD_NONE        = 0,
    MOTOR_CMD_START_MOVEJ = 1,
    MOTOR_CMD_STOP        = 2
} MotorCmdType_e;

typedef struct
{
    MotorCmdType_e type;
    uint32_t seq;

    float q_target[JOINT_NUM];
    float vmax[JOINT_NUM];
    float amax[JOINT_NUM];
} MotorCmdMsg_t;

typedef struct
{
    uint8_t feedback_valid;
    uint8_t ik_ok;
    int32_t ik_candidate_count;

    uint32_t latest_request_seq;
    uint32_t active_seq;
    uint32_t completed_seq;

    ArmTrajectoryMotionState_e motion_state;

    float q_fb[JOINT_NUM];
    float q_target[JOINT_NUM];
    float q_cmd[JOINT_NUM];
    float v_cmd[JOINT_NUM];
} MotionFrameworkStatus_t;

extern QueueHandle_t g_pose_cmd_queue;
extern QueueHandle_t g_motor_cmd_queue;

extern TaskHandle_t g_algorithm_task_handle;
extern TaskHandle_t g_motor_task_handle;

/* 初始化：创建队列 + 创建两个任务 */
bool MotionFramework_Init(void);

/* 外部提交一个目标位姿给 algorithm 线程 */
bool MotionFramework_SubmitPoseTarget(const Pose6D_t *target_pose,
                                      const float wrist_offset[3],
                                      const float vmax[JOINT_NUM],
                                      const float amax[JOINT_NUM],
                                      uint32_t *out_seq);

/* 外部请求停止 */
bool MotionFramework_RequestStop(void);

/* 外部读取状态快照 */
bool MotionFramework_GetStatus(MotionFrameworkStatus_t *out_status);

/* 两个任务函数 */
void AlgorithmTask(void *argument);
void MotorTask(void *argument);

/* ========= 需要你在项目中覆盖/实现的硬件接口 ========= */

/* 读取当前 6 个关节实际反馈角，单位 rad
 * 返回 true 表示读取成功 */
bool App_ReadJointFeedback(float q_fb[JOINT_NUM]);

/* 可选回调：IK失败 */
void App_OnIKFailed(uint32_t seq, const Pose6D_t *target_pose);

/* 可选回调：运动完成 */
void App_OnMotionDone(uint32_t seq);

/* 可选回调：运动故障 */
void App_OnMotionFault(uint32_t seq);



#endif