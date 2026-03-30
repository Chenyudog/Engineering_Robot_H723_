#include "app_motion_framework.h"

#include <string.h>
#include <math.h>

/* 你的执行器文件里已经定义了 global joint_motor[6] */
extern JointMotorObj_t joint_motor[JOINT_NUM];

/* ================= 全局对象 ================= */

QueueHandle_t g_pose_cmd_queue = NULL;
QueueHandle_t g_motor_cmd_queue = NULL;

TaskHandle_t g_algorithm_task_handle = NULL;
TaskHandle_t g_motor_task_handle = NULL;

static MotionFrameworkStatus_t g_motion_status;
static uint32_t g_next_seq = 1U;

/* ================= 内部辅助函数 ================= */

static void MotionFramework_ClearStatus(void)
{
    taskENTER_CRITICAL();
    memset(&g_motion_status, 0, sizeof(g_motion_status));
    g_motion_status.motion_state = JMOVE_IDLE;
    taskEXIT_CRITICAL();
}

static void MotionFramework_UpdateStatusSnapshot(const MotionFrameworkStatus_t *src)
{
    taskENTER_CRITICAL();
    g_motion_status = *src;
    taskEXIT_CRITICAL();
}

static void MotionFramework_ReadStatusSnapshot(MotionFrameworkStatus_t *dst)
{
    taskENTER_CRITICAL();
    *dst = g_motion_status;
    taskEXIT_CRITICAL();
}

static uint32_t MotionFramework_AllocSeq(void)
{
    uint32_t seq;
    taskENTER_CRITICAL();
    seq = g_next_seq++;
    taskEXIT_CRITICAL();
    return seq;
}

static void FillArrayCopy(float dst[JOINT_NUM], const float src[JOINT_NUM])
{
    memcpy(dst, src, sizeof(float) * JOINT_NUM);
}

static void FillArrayZero(float dst[JOINT_NUM])
{
    memset(dst, 0, sizeof(float) * JOINT_NUM);
}

/* ================= 外部接口实现 ================= */

bool MotionFramework_Init(void)
{
    BaseType_t ok;

    MotionFramework_ClearStatus();

    /* 长度 1：始终保留“最新命令” */
    g_pose_cmd_queue = xQueueCreate(1, sizeof(AlgoPoseCmdMsg_t));
    if (g_pose_cmd_queue == NULL)
    {
        return false;
    }

    g_motor_cmd_queue = xQueueCreate(1, sizeof(MotorCmdMsg_t));
    if (g_motor_cmd_queue == NULL)
    {
        return false;
    }

    return true;
}

bool MotionFramework_SubmitPoseTarget(const Pose6D_t *target_pose,
                                      const float wrist_offset[3],
                                      const float vmax[JOINT_NUM],
                                      const float amax[JOINT_NUM],
                                      uint32_t *out_seq)
{
    AlgoPoseCmdMsg_t msg;

    if ((g_pose_cmd_queue == NULL) ||
        (target_pose == NULL) ||
        (wrist_offset == NULL) ||
        (vmax == NULL) ||
        (amax == NULL))
    {
        return false;
    }

    memset(&msg, 0, sizeof(msg));
    msg.type = ALGO_POSE_CMD_MOVEJ;
    msg.seq = MotionFramework_AllocSeq();
    msg.target_pose = *target_pose;
    msg.wrist_offset[0] = wrist_offset[0];
    msg.wrist_offset[1] = wrist_offset[1];
    msg.wrist_offset[2] = wrist_offset[2];
    FillArrayCopy(msg.vmax, vmax);
    FillArrayCopy(msg.amax, amax);

    if (out_seq != NULL)
    {
        *out_seq = msg.seq;
    }

    taskENTER_CRITICAL();
    g_motion_status.latest_request_seq = msg.seq;
    taskEXIT_CRITICAL();

    return (xQueueOverwrite(g_pose_cmd_queue, &msg) == pdPASS);
}

bool MotionFramework_RequestStop(void)
{
    MotorCmdMsg_t stop_msg;

    if (g_motor_cmd_queue == NULL)
    {
        return false;
    }

    /* 丢弃还没处理的 pose 请求 */
    if (g_pose_cmd_queue != NULL)
    {
        xQueueReset(g_pose_cmd_queue);
    }

    memset(&stop_msg, 0, sizeof(stop_msg));
    stop_msg.type = MOTOR_CMD_STOP;

    return (xQueueOverwrite(g_motor_cmd_queue, &stop_msg) == pdPASS);
}

bool MotionFramework_GetStatus(MotionFrameworkStatus_t *out_status)
{
    if (out_status == NULL)
    {
        return false;
    }

    MotionFramework_ReadStatusSnapshot(out_status);
    return true;
}

/* ================= algorithm 线程 ================= */

void AlgorithmTask(void *argument)
{
    AlgoPoseCmdMsg_t pose_msg;
    MotorCmdMsg_t motor_msg;
    MotionFrameworkStatus_t status;

    float q_seed[JOINT_NUM];
    float q_best[JOINT_NUM];
    IKCandidate_t cand[IK_MAX_SOLUTIONS];
    int cand_count;
    int ik_ok;

    (void)argument;

    memset(&pose_msg, 0, sizeof(pose_msg));
    memset(&motor_msg, 0, sizeof(motor_msg));
    memset(&status, 0, sizeof(status));

    for (;;)
    {
        /* 阻塞等待新目标 */
        if (xQueueReceive(g_pose_cmd_queue, &pose_msg, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        if (pose_msg.type == ALGO_POSE_CMD_STOP)
        {
            memset(&motor_msg, 0, sizeof(motor_msg));
            motor_msg.type = MOTOR_CMD_STOP;
            motor_msg.seq = pose_msg.seq;
            (void)xQueueOverwrite(g_motor_cmd_queue, &motor_msg);
            continue;
        }

        /* 读取当前关节反馈，作为 IK 初值 q_last */
        if (!App_ReadJointFeedback(q_seed))
        {
            MotionFramework_ReadStatusSnapshot(&status);
            status.feedback_valid = 0U;
            status.ik_ok = 0U;
            status.ik_candidate_count = 0;
            status.latest_request_seq = pose_msg.seq;
            MotionFramework_UpdateStatusSnapshot(&status);

            App_OnIKFailed(pose_msg.seq, &pose_msg.target_pose);
            continue;
        }

        cand_count = 0;
        memset(cand, 0, sizeof(cand));
        memset(q_best, 0, sizeof(q_best));

        /* 直接调用你现有 IK 主接口 */
        ik_ok = IK_Solve_All(arm_sdh_table,
                             pose_msg.wrist_offset,
                             &pose_msg.target_pose,
                             q_seed,
                             &limit,
                             APP_IK_POS_TOL,
                             APP_IK_ORI_TOL,
                             q_best,
                             cand,
                             &cand_count);

        MotionFramework_ReadStatusSnapshot(&status);
        status.feedback_valid = 1U;
        status.ik_ok = (ik_ok != 0) ? 1U : 0U;
        status.ik_candidate_count = cand_count;
        status.latest_request_seq = pose_msg.seq;
        FillArrayCopy(status.q_fb, q_seed);

        if (ik_ok == 0)
        {
            FillArrayZero(status.q_target);
            MotionFramework_UpdateStatusSnapshot(&status);

            App_OnIKFailed(pose_msg.seq, &pose_msg.target_pose);
            continue;
        }

        FillArrayCopy(status.q_target, q_best);
        MotionFramework_UpdateStatusSnapshot(&status);

        memset(&motor_msg, 0, sizeof(motor_msg));
        motor_msg.type = MOTOR_CMD_START_MOVEJ;
        motor_msg.seq = pose_msg.seq;
        FillArrayCopy(motor_msg.q_target, q_best);
        FillArrayCopy(motor_msg.vmax, pose_msg.vmax);
        FillArrayCopy(motor_msg.amax, pose_msg.amax);

        /* 发给 motor 线程。队列长度 1，保留最新命令 */
        (void)xQueueOverwrite(g_motor_cmd_queue, &motor_msg);
    }
}

/* ================= motor 线程 ================= */

void MotorTask(void *argument)
{
    TickType_t last_wake_time;
    ArmTrajMotion_t arm;

    MotionFrameworkStatus_t status;
    MotorCmdMsg_t motor_msg;

    float q_current[JOINT_NUM];
    ArmTrajectoryMotionState_e prev_state;

    (void)argument;

    ArmTrajectoryPlanner_Init(&arm);
    last_wake_time = xTaskGetTickCount();
    prev_state = arm.state;

    memset(&status, 0, sizeof(status));
    memset(&motor_msg, 0, sizeof(motor_msg));

    for (;;)
    {
        /* 先看有没有新命令 */
        if (xQueueReceive(g_motor_cmd_queue, &motor_msg, 0) == pdPASS)
        {
            if (motor_msg.type == MOTOR_CMD_STOP)
            {
                ArmMotion_Stop(&arm);

                MotionFramework_ReadStatusSnapshot(&status);
                status.motion_state = arm.state;
                FillArrayZero(status.v_cmd);
                status.active_seq = 0U;
                MotionFramework_UpdateStatusSnapshot(&status);
            }
            else if (motor_msg.type == MOTOR_CMD_START_MOVEJ)
            {
                /* 用“当前实际反馈角”作为新轨迹起点 */
                if (!App_ReadJointFeedback(q_current))
                {
                    MotionFramework_ReadStatusSnapshot(&status);
                    status.feedback_valid = 0U;
                    status.motion_state = JMOVE_FAULT;
                    MotionFramework_UpdateStatusSnapshot(&status);

                    App_OnMotionFault(motor_msg.seq);
                }
                else
                {
                    if (!ArmMotion_StartJointMove(&arm,
                                                  q_current,
                                                  motor_msg.q_target,
                                                  motor_msg.vmax,
                                                  motor_msg.amax,
                                                  &limit))
                    {
                        MotionFramework_ReadStatusSnapshot(&status);
                        status.feedback_valid = 1U;
                        FillArrayCopy(status.q_fb, q_current);
                        status.motion_state = JMOVE_FAULT;
                        status.active_seq = motor_msg.seq;
                        MotionFramework_UpdateStatusSnapshot(&status);

                        App_OnMotionFault(motor_msg.seq);
                    }
                    else
                    {
                        MotionFramework_ReadStatusSnapshot(&status);
                        status.feedback_valid = 1U;
                        FillArrayCopy(status.q_fb, q_current);
                        FillArrayCopy(status.q_target, motor_msg.q_target);
                        status.motion_state = arm.state;
                        status.active_seq = motor_msg.seq;
                        MotionFramework_UpdateStatusSnapshot(&status);
                    }
                }
            }
        }

        /* 1ms 周期推进轨迹 + 下发 pos_ctrl() */
        if (arm.state == JMOVE_RUNNING)
        {
            if (!ArmMotion_Tick(&arm, NULL, joint_motor, MOTOR_TASK_DT_SEC))
            {
                arm.state = JMOVE_FAULT;
            }
        }

        /* 更新反馈和状态快照 */
        MotionFramework_ReadStatusSnapshot(&status);

        if (App_ReadJointFeedback(q_current))
        {
            status.feedback_valid = 1U;
            FillArrayCopy(status.q_fb, q_current);
        }
        else
        {
            status.feedback_valid = 0U;
        }

        FillArrayCopy(status.q_cmd, arm.last_q_cmd);
        FillArrayCopy(status.v_cmd, arm.last_v_cmd);
        status.motion_state = arm.state;

        /* 检测完成/故障边沿 */
        if ((prev_state != JMOVE_DONE) && (arm.state == JMOVE_DONE))
        {
            status.completed_seq = status.active_seq;
            MotionFramework_UpdateStatusSnapshot(&status);
            App_OnMotionDone(status.completed_seq);
        }
        else if ((prev_state != JMOVE_FAULT) && (arm.state == JMOVE_FAULT))
        {
            MotionFramework_UpdateStatusSnapshot(&status);
            App_OnMotionFault(status.active_seq);
        }
        else
        {
            MotionFramework_UpdateStatusSnapshot(&status);
        }

        prev_state = arm.state;

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
    }
}

/* ================= 弱符号：你在自己工程里覆盖 ================= */

__attribute__((weak)) bool App_ReadJointFeedback(float q_fb[JOINT_NUM])
{
    int i;

    if (q_fb == NULL)
    {
        return false;
    }

    /* 这里只是默认占位。
     * 你需要在自己的工程里覆盖它，从实际电机反馈中读 6 个关节角（rad） */
    for (i = 0; i < JOINT_NUM; i++)
    {
        q_fb[i] = 0.0f;
    }

    return false;
}

__attribute__((weak)) void App_OnIKFailed(uint32_t seq, const Pose6D_t *target_pose)
{
    (void)seq;
    (void)target_pose;
}

__attribute__((weak)) void App_OnMotionDone(uint32_t seq)
{
    (void)seq;
}

__attribute__((weak)) void App_OnMotionFault(uint32_t seq)
{
    (void)seq;
}