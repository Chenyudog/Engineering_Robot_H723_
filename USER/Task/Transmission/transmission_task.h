//
// Created by Áõ¼Î¿¡ on 25-3-31.
//

#ifndef CTRBOARD_H7_ALL_TRANSMISSION_TASK_H
#define CTRBOARD_H7_ALL_TRANSMISSION_TASK_H


#include <stdint.h>

struct pc_cmd_arm_msg
{
    float joint1_pos;
    float joint2_pos;
    float joint3_pos;
    float joint4_pos;
    float joint5_pos;
    float joint6_pos;
    uint8_t auto_state;
    uint8_t gripper_ctrl;
};

#endif //CTRBOARD_H7_ALL_TRANSMISSION_TASK_H
