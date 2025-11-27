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
#include <math.h>
#include "DMmotor_task.h"
#include "drv_dwt.h"
#include "PID.h"
#include "cmd_task.h"

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
static uint32_t DMmotor_task_dwt = 0;   // 毫秒监测
static float DMmotor_task_dt = 0;       // 线程实际运行时间dt
static float DMmotor_task_delta = 0;    // 监测线程运行时间
static float DMmotor_task_start_dt = 0; // 监测线程开始时间
/* -------------------------------- 调试监测线程相关 --------------------------------- */

static float current_angle[6] = {0.0f};        // 实际的关节输出角度，也是需要滤波的值
static float dm_angles[6] = {0.0f};   // 队列读取值
static float dm_motor_angles[6] = {0.0f};   // 期望角度值

extern QueueHandle_t xControlQueue;
extern Gripper_mode_e Gripper_mode;

DMmotorControl motor_controls[6] = {
        { MOTOR_1_MIN_LIMIT, MOTOR_1_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 0 (FDCAN3)
        { MOTOR_2_MIN_LIMIT, MOTOR_2_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 1 (FDCAN2)
        { MOTOR_3_MIN_LIMIT, MOTOR_3_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 2 (FDCAN2)
        { MOTOR_4_MIN_LIMIT, MOTOR_4_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 3 (FDCAN2)
        { MOTOR_5_MIN_LIMIT, MOTOR_5_MAX_LIMIT, 0.0f, 0.0f, 0 }, // Motor 4 (FDCAN2)
        { -M_PI, M_PI, 0.0f, 0.0f, 0 }  // Motor 5 (FDCAN2)
};

extern motor_t motor[num];//读取到的电机数据  //不可以修改

struct arm_cmd_msg arm_cmd = {
        .ctrl_mode = ARM_DISABLE,
        .last_mode = ARM_DISABLE
};

void arm_cmd_enable(void) {
    if (arm_cmd.last_mode == ARM_DISABLE && arm_cmd.ctrl_mode == ARM_ENABLE) {
        dm_motor_enable(&hfdcan3, &motor[Motor1]);
        for(int i=1;i<6;i++)
        {
            dm_motor_enable(&hfdcan2, &motor[i]);
            vTaskDelay(1);
        }
        arm_cmd.last_mode = ARM_ENABLE;
    }
}
void arm_cmd_disable(void) {
    if (arm_cmd.last_mode == ARM_ENABLE && arm_cmd.ctrl_mode == ARM_DISABLE) {
        dm_motor_disable(&hfdcan3, &motor[Motor1]);
        for(int i=1;i<6;i++)
        {
            dm_motor_disable(&hfdcan2, &motor[i]);
            vTaskDelay(1);
        }
        arm_cmd.last_mode = ARM_DISABLE;
    }
}

void arm_cmd_init(void) {
    if (arm_cmd.last_mode == ARM_ENABLE && arm_cmd.ctrl_mode == ARM_INIT) {
        pos_ctrl(&hfdcan3, motor[Motor1].id, 0, 0.7f); // 发送控制命令
        vTaskDelay(200); // 延时，等待电机稳定

        for(int i=1;i<6;i++)
        {
            dm_motor_enable(&hfdcan2, &motor[i]);
            pos_ctrl(&hfdcan2, motor[i].id, 0, 0.7f); // 发送控制命令
            vTaskDelay(200); // 延时，等待电机稳定
        }
        arm_cmd.last_mode = ARM_ENABLE;  //TODO:BUG一个
    }
}

void arm_cmd_state_machine(void) {

    switch (arm_cmd.ctrl_mode) {
        case ARM_ENABLE:
            arm_cmd_enable();
            break;
        case ARM_DISABLE:
            arm_cmd_disable();
            break;
//        case ARM_INIT:
//            arm_cmd_init();
//            break;
        default:
//            arm_cmd_disable();
            break;
    }
}



/* -------------------------------- 线程入口 ------------------------------- */
void DMmotorTask_Entry(void const * argument)
{
/* -------------------------------- 外设初始化段落 ------------------------------- */
    for (int i = 0; i < 6; i++) {
        motor_controls[i].current_angle_rad = 0.0f;
        motor_controls[i].last_angle_rad = 0.0f;
        motor_controls[i].initial_offset_rad = 0.0f;
        motor_controls[i].calibrated = 0;
    }

    dm_motor_enable(&hfdcan3, &motor[Motor1]);
    vTaskDelay(200); // 延时，等待电机稳定
    pos_ctrl(&hfdcan3, motor[Motor1].id, 0, 0.7f); // 发送控制命令
    vTaskDelay(200); // 延时，等待电机稳定

    for(int i=1;i<6;i++)
    {
        dm_motor_enable(&hfdcan2, &motor[i]);
        vTaskDelay(200); // 延时，等待电机稳定
        pos_ctrl(&hfdcan2, motor[i].id, 0, 0.7f); // 发送控制命令
        vTaskDelay(200); // 延时，等待电机稳定
    }

    dm_motor_enable(&hfdcan2, &motor[Motor7]);//不用校准//开始发送夹爪初始化控制指令
    vTaskDelay(200); // 延时，等待电机稳定

    arm_cmd.ctrl_mode = ARM_ENABLE; // 使能机械臂
    arm_cmd.last_mode = ARM_ENABLE;

/* -------------------------------- 外设初始化段落 ------------------------------- */

/* -------------------------------- 线程间Topics初始化 ------------------------------- */
//    chassis_pub_init();
//    chassis_sub_init();
/* -------------------------------- 线程间Topics初始化 ------------------------------- */
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    DMmotor_task_dt = dwt_get_delta(&DMmotor_task_dwt);
    DMmotor_task_start_dt = dwt_get_time_ms();
/* -------------------------------- 调试监测线程调度 --------------------------------- */
    for(;;)
    {
/* -------------------------------- 调试监测线程调度 --------------------------------- */
        DMmotor_task_delta = dwt_get_time_ms() - DMmotor_task_start_dt;
        DMmotor_task_start_dt = dwt_get_time_ms();
        DMmotor_task_dt = dwt_get_delta(&DMmotor_task_dwt);
/* -------------------------------- 调试监测线程调度 --------------------------------- */
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */
//        chassis_sub_pull();
/* -------------------------------- 线程订阅Topics信息 ------------------------------- */

/* -------------------------------- 线程代码编写段落 ------------------------------- */
        if (xQueueReceive(xControlQueue, dm_angles, 0) == pdPASS) {
            for(uint8_t i=0;i<6;i++){
                dm_motor_angles[i] = dm_angles[i];
            }
            DMcontrol_motor_1(&hfdcan3, &motor_controls[Motor1], dm_motor_angles[Motor1]);
            DMcontrol_motor_2(&hfdcan2, &motor_controls[Motor2], dm_motor_angles[Motor2]);
            DMcontrol_motor_3(&hfdcan2, &motor_controls[Motor3], dm_motor_angles[Motor3]);
            DMcontrol_motor_4(&hfdcan2, &motor_controls[Motor4], dm_motor_angles[Motor4]);
            DMcontrol_motor_5(&hfdcan2, &motor_controls[Motor5], dm_motor_angles[Motor5]);
            DMcontrol_motor_6(&hfdcan2, &motor_controls[Motor6], dm_motor_angles[Motor6]);
            DMcontrol_motor_7(&hfdcan2);//夹爪控制//一键夹取功能
        }
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

float clamp_radians(float radians, float min_limit, float max_limit) {
    if (radians > max_limit) return max_limit;
    if (radians < min_limit) return min_limit;
    return radians;
}

void smooth_motion_1(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[0] = target_angle;  // 直接使用目标角度，无需插值
    pos_ctrl(hcan, motor->id, -current_angle[0], 10.0f);  // 符号处理保留
}

void smooth_motion_2(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[1] = target_angle * GEAR_RATIO_2;  // 保留齿轮比转换
    pos_ctrl(hcan, motor->id, -current_angle[1], 10.0f);
}

void smooth_motion_3(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[2] = target_angle;
    pos_ctrl(hcan, motor->id, -current_angle[2], 10.0f);
}

void smooth_motion_4(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[3] = target_angle;
    pos_ctrl(hcan, motor->id, -current_angle[3], 10.0f);
}

void smooth_motion_5(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[4] = target_angle;
    pos_ctrl(hcan, motor->id, -current_angle[4], 10.0f);
}

void smooth_motion_6(hcan_t* hcan, motor_t* motor, float target_angle) {
    current_angle[5] = target_angle;
    pos_ctrl(hcan, motor->id, -current_angle[5], 10.0f);
}

void smooth_motion_7(hcan_t* hcan, motor_t* motor, float target_rad,float target_torque,float target_vel,float kp,float kd) {
    mit_ctrl(hcan,&motor[Motor7],motor->id,target_rad, target_vel, kp, kd, target_torque);
}

void DMcontrol_motor_1(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor1] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor1]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1) {
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad,motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_1(hcan, &motor[Motor1], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_2(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor2] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor2]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_2(hcan, &motor[Motor2], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_3(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor3] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor3]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_3(hcan, &motor[Motor3], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_4(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor4] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor4]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_4(hcan, &motor[Motor4], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_5(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor5] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor5]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_5(hcan, &motor[Motor5], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;
    }
}

void DMcontrol_motor_6(hcan_t* hcan, DMmotorControl* motor_control, float target_angle) {
    if (!motor_control->calibrated) {
        if (dm_motor_angles[Motor6] == 0) {
            motor_control->calibrated = 0;
        }else{
            motor_control->initial_offset_rad = DEG_TO_RAD(dm_motor_angles[Motor6]);
            motor_control->calibrated = 1;
        }
    }else if(motor_control->calibrated == 1){
        motor_control->current_angle_rad = DEG_TO_RAD(target_angle);

        float angle = clamp_radians(motor_control->current_angle_rad, motor_control->motor_min_limit, motor_control->motor_max_limit);

        smooth_motion_6(hcan, &motor[Motor6], angle);

        motor_control->last_angle_rad = motor_control->current_angle_rad;

    }
}

void DMcontrol_motor_7(hcan_t* hcan)
{
    float target_rad,target_torque,target_vel,target_kp,target_kd;
    if(Gripper_mode == Gripper_OPEN)//一键抓取模式,在这里调参
    {
        target_rad = 0.0f;
        target_torque = 0.0f;
        target_vel = 0.0f;
        target_kp = 0.0f;
        target_kd = 0.0f;
    }
    else
    {
        target_rad = 0.0f;
        target_torque = 0.0f;
        target_vel = 0.0f;
        target_kp = 0.0f;
        target_kd = 0.0f;
    }

    smooth_motion_7(hcan, &motor[Motor7],target_rad,target_torque,target_vel,target_kp,target_kd);
}


