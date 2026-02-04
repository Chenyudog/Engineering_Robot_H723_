//
// Created by 刘嘉俊 on 25-1-6.
//

#ifndef CTRBOARD_H7_ALL_CMD_TASK_H
#define CTRBOARD_H7_ALL_CMD_TASK_H
#include "cmsis_os.h"

void remote_to_cmd_sbus(void);
void Gripper_ctrl(void);



typedef enum
{
    Gripper_CLOSE,
    Gripper_OPEN,
} Gripper_mode_e;

typedef enum
{
    Store_NO1,
    Store_NO2,
    Store_NO3,
} Store_mode_e;

#endif //CTRBOARD_H7_ALL_CMD_TASK_H
