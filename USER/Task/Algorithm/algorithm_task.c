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



//#ifndef M_PI
//#define M_PI 3.14159265358979323846
//#endif
//
//#define DEG2RAD(x) ((x) * (float)M_PI / 180.0f)
//#define RAD2DEG(x) ((x) * 180.0f / (float)M_PI)
//
//#define TRAJ_POINTS 100
//
//static void PrintQDegOneLine(const float q[6])
//{
//    //USART7_DebugPrintf("[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] deg",
////           RAD2DEG(q[0]), RAD2DEG(q[1]), RAD2DEG(q[2]),
////           RAD2DEG(q[3]), RAD2DEG(q[4]), RAD2DEG(q[5]));
//}
//static void BuildLinearPose(const Pose6D_t *start_pose,
//                            const Pose6D_t *end_pose,
//                            float t,
//                            Pose6D_t *target_pose)
//{
//    int i;
//
//    target_pose->X = start_pose->X + t * (end_pose->X - start_pose->X);
//    target_pose->Y = start_pose->Y + t * (end_pose->Y - start_pose->Y);
//    target_pose->Z = start_pose->Z + t * (end_pose->Z - start_pose->Z);
//
//    /* 这里先固定姿态不变：整条轨迹共用起点姿态 */
//    target_pose->hasR = 1;
//    for (i = 0; i < 9; i++) {
//        target_pose->R[i] = start_pose->R[i];
//    }
//
//    target_pose->YAW   = start_pose->YAW;
//    target_pose->PITCH = start_pose->PITCH;
//    target_pose->ROLL  = start_pose->ROLL;
//}
//
//int IKSOLVE(void)
//{
//    const SDH_Param_t *table = arm_sdh_table;
//    float wrist_offset[3] = {0.0f, 0.0f, 0.0f};
//
//    float pos_tol = POSITION_TOLERANCE;
//    float ori_tol = ORIENTATION_TOLERANCE;
//
//    float q_init[6] = {0};
//    float q_last[6] = {0};
//    Pose6D_t start_pose, end_pose, target_pose, verify_pose;
//    int i, pass_count = 0;
//    float max_pos_err = 0.0f;
//    float max_ori_err = 0.0f;
//
//    //USART7_DebugPrintf("\n");
//    //USART7_DebugPrintf("PC端直线轨迹运动学验证程序\n");
//    //USART7_DebugPrintf("============================================================\n");
//    //USART7_DebugPrintf("初始关节角固定为 [0 0 0 0 0 0]\n");
//    //USART7_DebugPrintf("沿笛卡尔空间直线插值 100 个目标点，逐点做 IK + FK 回代验证\n");
//    //USART7_DebugPrintf("============================================================\n\n");
//
//    if (!SDH_FK_ToPose6D(table, q_init, &start_pose)) {
//        //USART7_DebugPrintf("初始关节角的正向运动学求解失败。\n");
//        return 1;
//    }
//
//    end_pose = start_pose;
//    end_pose.X += 0.300f;
//    end_pose.Y += 0.300f;
//    end_pose.Z -= 0.150f;
//
//    //USART7_DebugPrintf("起点位姿:\n");
//    //USART7_DebugPrintf("  P_start = [%.6f, %.6f, %.6f]\n", start_pose.X, start_pose.Y, start_pose.Z);
//    //USART7_DebugPrintf("终点位姿:\n");
//    //USART7_DebugPrintf("  P_end   = [%.6f, %.6f, %.6f]\n", end_pose.X, end_pose.Y, end_pose.Z);
//    //USART7_DebugPrintf("\n");
//
//    //USART7_DebugPrintf("开始轨迹验证...\n");
//    //USART7_DebugPrintf("------------------------------------------------------------\n");
//
//    for (i = 0; i < TRAJ_POINTS; i++) {
//        float t = (float)i / (float)(TRAJ_POINTS - 1);
//        float q_best[6];
//        int ik_ok;
//        float pos_err, ori_err;
//
//        BuildLinearPose(&start_pose, &end_pose, t, &target_pose);
//
//        ik_ok = IK_Solve_All(table,
//                             wrist_offset,
//                             &target_pose,
//                             q_last,
//                             &limit,
//                             pos_tol,
//                             ori_tol,
//                             q_best,
//                             NULL,
//                             NULL);
//
//        if (!ik_ok) {
//            //USART7_DebugPrintf("点 %3d | t=%.3f | IK失败 | 目标位置=[%.6f, %.6f, %.6f]\n",
////                               i + 1, t, target_pose.X, target_pose.Y, target_pose.Z);
//            continue;
//        }
//
////        if (!SDH_FK_ToPose6D(table, q_best, &verify_pose)) {
////            //USART7_DebugPrintf("点 %3d | t=%.3f | FK回代失败\n", i + 1, t);
////            continue;
////        }
//
//        if (!IK_Evaluate_Solution_Error(table, q_best, &target_pose, &pos_err, &ori_err)) {
//            //USART7_DebugPrintf("点 %3d | t=%.3f | 误差评估失败\n", i + 1, t);
//            continue;
//        }
//
//        if (pos_err > max_pos_err) max_pos_err = pos_err;
//        if (ori_err > max_ori_err) max_ori_err = ori_err;
//
//        //USART7_DebugPrintf("点 %3d | t=%.3f | 最优解 = ", i + 1, t);
//        PrintQDegOneLine(q_best);
//        //USART7_DebugPrintf(" | pos_err=%.9f | ori_err=%.9f rad\n", pos_err, ori_err);
//
//        if (pos_err < pos_tol && ori_err < ori_tol) {
//            pass_count++;
//        }
//
//        memcpy(q_last, q_best, sizeof(float) * 6);
//    }
//
//    //USART7_DebugPrintf("------------------------------------------------------------\n");
//    //USART7_DebugPrintf("轨迹验证结束\n");
//    //USART7_DebugPrintf("通过点数: %d / %d\n", pass_count, TRAJ_POINTS);
//    //USART7_DebugPrintf("最大位置误差: %.9f\n", max_pos_err);
//    //USART7_DebugPrintf("最大姿态误差: %.9f rad (%.6f deg)\n", max_ori_err, RAD2DEG(max_ori_err));
//
//    if (pass_count == TRAJ_POINTS) {
//        //USART7_DebugPrintf("结论: 100个目标点全部验证通过。\n");
//    } else {
//        //USART7_DebugPrintf("结论: 存在未通过点，请检查对应点的IK可达性、限位或分支跳变。\n");
//    }
//
//    return 0;
//}

#include "joint_movej_planner.h"   // 里面会包含 trajectory_time_synchronization.h
#include "KinematicSDHdsp.h"       // 这里假设 JointLimit_t 在这里定义

#define DT              (0.01f)      // 10ms 控制周期
#define POS_TOL         (1e-4f)
#define VEL_TOL         (1e-4f)
#define ACC_TOL         (5e-2f)      // 差分估算加速度，给一点容差
#define SYNC_TOL        (1e-4f)
#define MAX_STEPS       (100000)

static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static void print_vec6(const char *name, const float a[JOINT_NUM])
{
    int i;
    USART7_DebugPrintf("%s = [", name);
    for (i = 0; i < JOINT_NUM; i++)
    {
        USART7_DebugPrintf("% .6f", a[i]);
        if (i < JOINT_NUM - 1) USART7_DebugPrintf(", ");
    }
    USART7_DebugPrintf("]\n");
}

static int find_first_moving_joint(const JointMoveJPlanner_t *jp)
{
    int i;
    for (i = 0; i < JOINT_NUM; i++)
    {
        if (fabsf(jp->dq[i]) > JOINT_EPS_MOVE)
        {
            return i;
        }
    }
    return -1;
}

static float calc_s_from_joint(const JointMoveJPlanner_t *jp, int idx)
{
    if (idx < 0 || idx >= JOINT_NUM) return 0.0f;
    if (fabsf(jp->dq[idx]) < JOINT_EPS_MOVE) return 0.0f;
    return (jp->q_ref[idx] - jp->q0[idx]) / jp->dq[idx];
}

static float calc_sync_err(const JointMoveJPlanner_t *jp)
{
    int i;
    int ref = find_first_moving_joint(jp);
    float s_ref, s_i, max_err = 0.0f;

    if (ref < 0) return 0.0f;

    s_ref = calc_s_from_joint(jp, ref);

    for (i = 0; i < JOINT_NUM; i++)
    {
        if (fabsf(jp->dq[i]) < JOINT_EPS_MOVE)
        {
            continue;
        }

        s_i = calc_s_from_joint(jp, i);
        if (fabsf(s_i - s_ref) > max_err)
        {
            max_err = fabsf(s_i - s_ref);
        }
    }

    return max_err;
}

static bool check_step_constraints(const JointMoveJPlanner_t *jp,
                                   const float vmax[JOINT_NUM],
                                   const float amax[JOINT_NUM],
                                   const float prev_q[JOINT_NUM],
                                   const float prev_v[JOINT_NUM],
                                   float dt,
                                   int step)
{
    int i;
    bool ok = true;

    for (i = 0; i < JOINT_NUM; i++)
    {
        float a_est = 0.0f;

        /* 静止关节：位置应保持不变，速度应接近0 */
        if (fabsf(jp->dq[i]) < JOINT_EPS_MOVE)
        {
            if (fabsf(jp->q_ref[i] - jp->q0[i]) > POS_TOL)
            {
                USART7_DebugPrintf("  [ERR] joint %d should stay still, q_ref=%f q0=%f\n",
                       i + 1, jp->q_ref[i], jp->q0[i]);
                ok = false;
            }
            if (fabsf(jp->v_ref[i]) > VEL_TOL)
            {
                USART7_DebugPrintf("  [ERR] joint %d should have zero velocity, v_ref=%f\n",
                       i + 1, jp->v_ref[i]);
                ok = false;
            }
            continue;
        }

        /* 速度限幅检查 */
        if (fabsf(jp->v_ref[i]) > vmax[i] + 1e-3f)
        {
            USART7_DebugPrintf("  [ERR] joint %d exceeds vmax: |v|=%f > vmax=%f\n",
                   i + 1, fabsf(jp->v_ref[i]), vmax[i]);
            ok = false;
        }

        /* 单调性 / 不越界检查 */
        if (jp->dq[i] > 0.0f)
        {
            if (jp->q_ref[i] + POS_TOL < prev_q[i])
            {
                USART7_DebugPrintf("  [ERR] joint %d position goes backward\n", i + 1);
                ok = false;
            }
            if (jp->q_ref[i] > jp->q1[i] + POS_TOL)
            {
                USART7_DebugPrintf("  [ERR] joint %d overshoots target\n", i + 1);
                ok = false;
            }
            if (jp->v_ref[i] < -VEL_TOL)
            {
                USART7_DebugPrintf("  [ERR] joint %d velocity sign wrong\n", i + 1);
                ok = false;
            }
        }
        else
        {
            if (jp->q_ref[i] - POS_TOL > prev_q[i])
            {
                USART7_DebugPrintf("  [ERR] joint %d position goes backward (negative move)\n", i + 1);
                ok = false;
            }
            if (jp->q_ref[i] < jp->q1[i] - POS_TOL)
            {
                USART7_DebugPrintf("  [ERR] joint %d overshoots target (negative move)\n", i + 1);
                ok = false;
            }
            if (jp->v_ref[i] > VEL_TOL)
            {
                USART7_DebugPrintf("  [ERR] joint %d velocity sign wrong (negative move)\n", i + 1);
                ok = false;
            }
        }

        /* 用差分估算加速度 */
        if (step > 0)
        {
            a_est = (jp->v_ref[i] - prev_v[i]) / dt;
            if (fabsf(a_est) > amax[i] + ACC_TOL)
            {
                USART7_DebugPrintf("  [ERR] joint %d exceeds amax: |a|=%f > amax=%f\n",
                       i + 1, fabsf(a_est), amax[i]);
                ok = false;
            }
        }
    }

    /* 同步误差检查 */
    if (calc_sync_err(jp) > SYNC_TOL)
    {
        USART7_DebugPrintf("  [ERR] sync error too large: %f\n", calc_sync_err(jp));
        ok = false;
    }

    return ok;
}

static void run_normal_case(void)
{
    JointMoveJPlanner_t jp;
    JointLimit_t limit;

    /* 一组正常测试数据 */
    float q0[JOINT_NUM]   = { 0.0f,  0.2f, -0.3f, 0.0f,  0.5f, -0.2f };
    float q1[JOINT_NUM]   = { 1.0f, -0.6f,  0.4f, 0.0f, -0.8f,  0.3f };
    float vmax[JOINT_NUM] = { 1.2f,  1.0f,  0.8f, 0.5f,  1.5f,  1.0f };
    float amax[JOINT_NUM] = { 2.0f,  1.5f,  1.2f, 1.0f,  2.0f,  1.5f };

    float prev_q[JOINT_NUM];
    float prev_v[JOINT_NUM];
    bool all_ok = true;
    int step;
    int ref_joint;
    float s, sdot, t;

    /* 假设关节限位 +/- pi，可按你的机械臂真实限位修改 */
    for (step = 0; step < JOINT_NUM; step++)
    {
        limit.min[step] = -3.1415926f;
        limit.max[step] =  3.1415926f;
    }

    JointMoveJ_Init(&jp);

    if (!JointMoveJ_Start(&jp, q0, q1, vmax, amax, &limit))
    {
        USART7_DebugPrintf("run_normal_case: JointMoveJ_Start failed\n");
        return;
    }

    USART7_DebugPrintf("\n================ NORMAL CASE ================\n");
    print_vec6("q0", q0);
    print_vec6("q1", q1);
    print_vec6("vmax", vmax);
    print_vec6("amax", amax);

    USART7_DebugPrintf("moving_joint_count = %u\n", (unsigned int)jp.moving_joint_count);
    USART7_DebugPrintf("vs_max             = %.6f\n", jp.vs_max);
    USART7_DebugPrintf("as_max             = %.6f\n", jp.as_max);
    USART7_DebugPrintf("use_triangle       = %u\n", (unsigned int)jp.prof.use_triangle_profile);
    USART7_DebugPrintf("accel_time         = %.6f\n", jp.prof.accel_time);
    USART7_DebugPrintf("cruise_time        = %.6f\n", jp.prof.cruise_time);
    USART7_DebugPrintf("total_time         = %.6f\n", jp.prof.total_time);

    memcpy(prev_q, jp.q_ref, sizeof(prev_q));
    memcpy(prev_v, jp.v_ref, sizeof(prev_v));

    USART7_DebugPrintf("\nstep,t,s,sdot,sync_err,q1,q2,q3,q4,q5,q6,v1,v2,v3,v4,v5,v6\n");

    for (step = 0; step < MAX_STEPS; step++)
    {
        t = step * DT;

        if (!JointMoveJ_Update(&jp, DT))
        {
            USART7_DebugPrintf("JointMoveJ_Update failed at step=%d\n", step);
            all_ok = false;
            break;
        }

        ref_joint = find_first_moving_joint(&jp);
        if (ref_joint >= 0)
        {
            s = calc_s_from_joint(&jp, ref_joint);
            sdot = jp.v_ref[ref_joint] / jp.dq[ref_joint];
        }
        else
        {
            s = 1.0f;
            sdot = 0.0f;
        }

        USART7_DebugPrintf("%d,%.4f,%.6f,%.6f,%.8f,"
               "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
               "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
               step, t, s, sdot, calc_sync_err(&jp),
               jp.q_ref[0], jp.q_ref[1], jp.q_ref[2],
               jp.q_ref[3], jp.q_ref[4], jp.q_ref[5],
               jp.v_ref[0], jp.v_ref[1], jp.v_ref[2],
               jp.v_ref[3], jp.v_ref[4], jp.v_ref[5]);

        if (!check_step_constraints(&jp, vmax, amax, prev_q, prev_v, DT, step))
        {
            all_ok = false;
        }

        memcpy(prev_q, jp.q_ref, sizeof(prev_q));
        memcpy(prev_v, jp.v_ref, sizeof(prev_v));

        if (jp.state == JP_DONE)
        {
            break;
        }
    }

    /* 终点检查 */
    if (jp.state != JP_DONE)
    {
        USART7_DebugPrintf("[FAIL] planner did not reach JP_DONE\n");
        all_ok = false;
    }

    for (step = 0; step < JOINT_NUM; step++)
    {
        if (fabsf(jp.q_ref[step] - q1[step]) > POS_TOL)
        {
            USART7_DebugPrintf("[FAIL] final q_ref[%d]=%f, target=%f\n",
                   step, jp.q_ref[step], q1[step]);
            all_ok = false;
        }
        if (fabsf(jp.v_ref[step]) > VEL_TOL)
        {
            USART7_DebugPrintf("[FAIL] final v_ref[%d]=%f, should be 0\n",
                   step, jp.v_ref[step]);
            all_ok = false;
        }
    }

    if (all_ok)
    {
        USART7_DebugPrintf("\n[PASS] normal case passed.\n");
    }
    else
    {
        USART7_DebugPrintf("\n[FAIL] normal case failed.\n");
    }
}

static void run_no_move_case(void)
{
    JointMoveJPlanner_t jp;
    JointLimit_t limit;
    float q0[JOINT_NUM]   = { 0, 0, 0, 0, 0, 0 };
    float q1[JOINT_NUM]   = { 0, 0, 0, 0, 0, 0 };
    float vmax[JOINT_NUM] = { 1, 1, 1, 1, 1, 1 };
    float amax[JOINT_NUM] = { 1, 1, 1, 1, 1, 1 };
    int i;

    for (i = 0; i < JOINT_NUM; i++)
    {
        limit.min[i] = -3.1415926f;
        limit.max[i] =  3.1415926f;
    }

    JointMoveJ_Init(&jp);

    USART7_DebugPrintf("\n================ NO MOVE CASE ================\n");
    if (!JointMoveJ_Start(&jp, q0, q1, vmax, amax, &limit))
    {
        USART7_DebugPrintf("[FAIL] no-move case start failed\n");
        return;
    }

    USART7_DebugPrintf("state = %d (expect JP_DONE=%d)\n", jp.state, JP_DONE);
    print_vec6("q_ref", jp.q_ref);
    print_vec6("v_ref", jp.v_ref);
}

static void run_limit_fault_case(void)
{
    JointMoveJPlanner_t jp;
    JointLimit_t limit;
    float q0[JOINT_NUM]   = { 0, 0, 0, 0, 0, 0 };
    float q1[JOINT_NUM]   = { 4.0f, 0, 0, 0, 0, 0 };  // 故意超限
    float vmax[JOINT_NUM] = { 1, 1, 1, 1, 1, 1 };
    float amax[JOINT_NUM] = { 1, 1, 1, 1, 1, 1 };
    int i;
    bool ok;

    for (i = 0; i < JOINT_NUM; i++)
    {
        limit.min[i] = -3.1415926f;
        limit.max[i] =  3.1415926f;
    }

    JointMoveJ_Init(&jp);

    USART7_DebugPrintf("\n================ LIMIT FAULT CASE ================\n");
    ok = JointMoveJ_Start(&jp, q0, q1, vmax, amax, &limit);

    USART7_DebugPrintf("start_return = %d (expect 0)\n", (int)ok);
    USART7_DebugPrintf("state        = %d (expect JP_FAULT=%d)\n", jp.state, JP_FAULT);
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

//        IKSOLVE();
        run_normal_case();
        run_no_move_case();
        run_limit_fault_case();

        // dof6_kinematic_solve_ik(&kin, &input_pose, &last_joints, &ik_solves);
//USART7_DebugPrintf("AlgorithmTask_Entry: algorithm_task_dt = %f\n", algorithm_task_dt);
//USART7_DebugPrintf("AlgorithmTask_Entry: algorithm_task_delta = %f\n", algorithm_task_delta);
/* -------------------------------- 线程代码编写段落 ------------------------------- */

/* -------------------------------- 线程发布Topics信息 ------------------------------- */
//        chassis_pub_push();
/* -------------------------------- 线程发布Topics信息 ------------------------------- */
        vTaskDelay(1000);
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