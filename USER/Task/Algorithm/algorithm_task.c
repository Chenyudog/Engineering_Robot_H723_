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
#include <stdio.h>
#include "algorithm_task.h"
#include "cmsis_os.h"
#include "KalmanFilterOne.h"
#include "drv_dwt.h"
#include "KinematicSDHdsp.h"
#include "usart_task.h"



/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
//static struct chassis_cmd_msg chassis_cmd;
//static struct chassis_fdb_msg chassis_fdb;
//static struct trans_fdb_msg trans_fdb;
//static struct ins_msg ins_data;
//
//static publisher_t *pub_chassis;
//static subscriber_t *sub_cmd,*sub_ins,*sub_trans;
//
//static void chassis_pub_init(void);
//static void chassis_sub_init(void);
//static void chassis_pub_push(void);
//static void chassis_sub_pull(void);
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
/* -------------------------------- 调试监测线程相关 --------------------------------- */
static uint32_t algorithm_task_dwt = 0;   // 毫秒监测
static float algorithm_task_dt = 0;       // 线程实际运行时间dt
static float algorithm_task_delta = 0;    // 监测线程运行时间
static float algorithm_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

static mat_type_t filtered_data[NUM_JOINTS];
static float angles[6] = {0}; // 从队列中读取的角度值（度数）

extern QueueHandle_t xKalmanOneQueue;
extern QueueHandle_t xControlQueue; // 队列句柄



#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD(x) ((x) * (float)M_PI / 180.0f)
#define RAD2DEG(x) ((x) * 180.0f / (float)M_PI)

#define TRAJ_POINTS 100

static void PrintQDegOneLine(const float q[6])
{
    //USART7_DebugPrintf("[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] deg",
//           RAD2DEG(q[0]), RAD2DEG(q[1]), RAD2DEG(q[2]),
//           RAD2DEG(q[3]), RAD2DEG(q[4]), RAD2DEG(q[5]));
}
static void BuildLinearPose(const Pose6D_t *start_pose,
                            const Pose6D_t *end_pose,
                            float t,
                            Pose6D_t *target_pose)
{
    int i;

    target_pose->X = start_pose->X + t * (end_pose->X - start_pose->X);
    target_pose->Y = start_pose->Y + t * (end_pose->Y - start_pose->Y);
    target_pose->Z = start_pose->Z + t * (end_pose->Z - start_pose->Z);

    /* 这里先固定姿态不变：整条轨迹共用起点姿态 */
    target_pose->hasR = 1;
    for (i = 0; i < 9; i++) {
        target_pose->R[i] = start_pose->R[i];
    }

    target_pose->YAW   = start_pose->YAW;
    target_pose->PITCH = start_pose->PITCH;
    target_pose->ROLL  = start_pose->ROLL;
}

int IKSOLVE(void)
{
    const SDH_Param_t *table = arm_sdh_table;
    float wrist_offset[3] = {0.0f, 0.0f, 0.0f};

    float pos_tol = POSITION_TOLERANCE;
    float ori_tol = ORIENTATION_TOLERANCE;

    float q_init[6] = {0};
    float q_last[6] = {0};
    Pose6D_t start_pose, end_pose, target_pose, verify_pose;
    int i, pass_count = 0;
    float max_pos_err = 0.0f;
    float max_ori_err = 0.0f;

    //USART7_DebugPrintf("\n");
    //USART7_DebugPrintf("PC端直线轨迹运动学验证程序\n");
    //USART7_DebugPrintf("============================================================\n");
    //USART7_DebugPrintf("初始关节角固定为 [0 0 0 0 0 0]\n");
    //USART7_DebugPrintf("沿笛卡尔空间直线插值 100 个目标点，逐点做 IK + FK 回代验证\n");
    //USART7_DebugPrintf("============================================================\n\n");

    if (!SDH_FK_ToPose6D(table, q_init, &start_pose)) {
        //USART7_DebugPrintf("初始关节角的正向运动学求解失败。\n");
        return 1;
    }

    end_pose = start_pose;
    end_pose.X += 0.300f;
    end_pose.Y += 0.300f;
    end_pose.Z -= 0.150f;

    //USART7_DebugPrintf("起点位姿:\n");
    //USART7_DebugPrintf("  P_start = [%.6f, %.6f, %.6f]\n", start_pose.X, start_pose.Y, start_pose.Z);
    //USART7_DebugPrintf("终点位姿:\n");
    //USART7_DebugPrintf("  P_end   = [%.6f, %.6f, %.6f]\n", end_pose.X, end_pose.Y, end_pose.Z);
    //USART7_DebugPrintf("\n");

    //USART7_DebugPrintf("开始轨迹验证...\n");
    //USART7_DebugPrintf("------------------------------------------------------------\n");

    for (i = 0; i < TRAJ_POINTS; i++) {
        float t = (float)i / (float)(TRAJ_POINTS - 1);
        float q_best[6];
        int ik_ok;
        float pos_err, ori_err;

        BuildLinearPose(&start_pose, &end_pose, t, &target_pose);

        ik_ok = IK_Solve_All(table,
                             wrist_offset,
                             &target_pose,
                             q_last,
                             &limit,
                             pos_tol,
                             ori_tol,
                             q_best,
                             NULL,
                             NULL);

        if (!ik_ok) {
            //USART7_DebugPrintf("点 %3d | t=%.3f | IK失败 | 目标位置=[%.6f, %.6f, %.6f]\n",
//                               i + 1, t, target_pose.X, target_pose.Y, target_pose.Z);
            continue;
        }

//        if (!SDH_FK_ToPose6D(table, q_best, &verify_pose)) {
//            //USART7_DebugPrintf("点 %3d | t=%.3f | FK回代失败\n", i + 1, t);
//            continue;
//        }

        if (!IK_Evaluate_Solution_Error(table, q_best, &target_pose, &pos_err, &ori_err)) {
            //USART7_DebugPrintf("点 %3d | t=%.3f | 误差评估失败\n", i + 1, t);
            continue;
        }

        if (pos_err > max_pos_err) max_pos_err = pos_err;
        if (ori_err > max_ori_err) max_ori_err = ori_err;

        //USART7_DebugPrintf("点 %3d | t=%.3f | 最优解 = ", i + 1, t);
        PrintQDegOneLine(q_best);
        //USART7_DebugPrintf(" | pos_err=%.9f | ori_err=%.9f rad\n", pos_err, ori_err);

        if (pos_err < pos_tol && ori_err < ori_tol) {
            pass_count++;
        }

        memcpy(q_last, q_best, sizeof(float) * 6);
    }

    //USART7_DebugPrintf("------------------------------------------------------------\n");
    //USART7_DebugPrintf("轨迹验证结束\n");
    //USART7_DebugPrintf("通过点数: %d / %d\n", pass_count, TRAJ_POINTS);
    //USART7_DebugPrintf("最大位置误差: %.9f\n", max_pos_err);
    //USART7_DebugPrintf("最大姿态误差: %.9f rad (%.6f deg)\n", max_ori_err, RAD2DEG(max_ori_err));

    if (pass_count == TRAJ_POINTS) {
        //USART7_DebugPrintf("结论: 100个目标点全部验证通过。\n");
    } else {
        //USART7_DebugPrintf("结论: 存在未通过点，请检查对应点的IK可达性、限位或分支跳变。\n");
    }

    return 0;
}



/* -------------------------------- 线程入口 ------------------------------- */
void AlgorithmTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    Init_KalmanFiltersOne(KALMAN_F, KALMAN_H, KALMAN_Q, KALMAN_R);

/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
    algorithm_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        algorithm_task_delta = dwt_get_time_ms() - algorithm_task_start_dt;
        algorithm_task_start_dt = dwt_get_time_ms();
        algorithm_task_dt = dwt_get_delta(&algorithm_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */

        if (xQueueReceive(xKalmanOneQueue, angles, 0) == pdTRUE) {
            // 对接收的数据进行滤波处理
            KalmanFilterOne_Data(angles, filtered_data);
            xQueueSend(xControlQueue, filtered_data, 0);
        }

        IKSOLVE();


        // dof6_kinematic_solve_ik(&kin, &input_pose, &last_joints, &ik_solves);
//printf("AlgorithmTask_Entry: algorithm_task_dt = %f\n", algorithm_task_dt);
//printf("AlgorithmTask_Entry: algorithm_task_delta = %f\n", algorithm_task_delta);
/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(1);
    }
}
/* -------------------------------- 线程结束 ------------------------------- */

/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */
///**
// * @brief chassis 线程中所有发布者初始化
// */
//static void chassis_pub_init(void)
//{
//    pub_chassis = pub_register("chassis_fdb",sizeof(struct chassis_fdb_msg));
//}
//
///**
// * @brief chassis 线程中所有订阅者初始化
// */
//static void chassis_sub_init(void)
//{
//    sub_cmd = sub_register("chassis_cmd", sizeof(struct chassis_cmd_msg));
//    sub_trans= sub_register("trans_fdb", sizeof(struct trans_fdb_msg));
//    sub_ins = sub_register("ins_msg", sizeof(struct ins_msg));
//}
//
///**
// * @brief chassis 线程中所有发布者推送更新话题
// */
//static void chassis_pub_push(void)
//{
//    pub_push_msg(pub_chassis,&chassis_fdb);
//}
///**
// * @brief chassis 线程中所有订阅者获取更新话题
// */
//static void chassis_sub_pull(void)
//{
//    sub_get_msg(sub_cmd, &chassis_cmd);
//    sub_get_msg(sub_trans, &trans_fdb);
//    sub_get_msg(sub_ins, &ins_data);
//}
/* -------------------------------- 线程间通讯Topics相关 ------------------------------- */