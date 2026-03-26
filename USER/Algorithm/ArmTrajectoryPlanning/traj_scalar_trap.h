//
// Created by 刘嘉俊 on 26-3-26.
//

#ifndef CTRBOARD_H7_ALL_TRAJ_SCALAR_TRAP_H
#define CTRBOARD_H7_ALL_TRAJ_SCALAR_TRAP_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    float v_max;      // 期望最大速度（标量 s 的速度上限）
    float a_max;      // 期望最大加速度（标量 s 的加速度上限）

    float t_acc;      // 加速时间
    float t_flat;     // 匀速时间
    float t_total;    // 总时间

    float s_acc;      // 加速段结束时的位移
    float v_peak;     // 实际峰值速度（梯形时= v_max，三角形时< v_max）

    float t;          // 当前内部时间
    uint8_t is_triangle;
    uint8_t active;
} ScalarTrapProfile_t;

/**
 * @brief 启动一个从 s=0 到 s=1 的标量梯形/三角速度轨迹
 * @param p       轨迹对象
 * @param v_max   最大速度，必须 > 0
 * @param a_max   最大加速度，必须 > 0
 * @return true   启动成功
 * @return false  参数非法
 */
bool ScalarTrap_Start(ScalarTrapProfile_t *p, float v_max, float a_max);

/**
 * @brief 更新轨迹，输出当前时刻的 s 和 sdot
 *
 * 说明：
 * - 本函数返回的是“当前 p->t 对应的采样值”
 * - 然后内部时间再推进 dt
 * - 这样 Start 后第一次调用 Update，就会先输出 s=0, sdot=0
 *
 * @param p       轨迹对象
 * @param dt      控制周期，建议 > 0
 * @param s       当前位移进度 [0,1]
 * @param sdot    当前速度
 * @return true   输出有效
 * @return false  参数非法
 */
bool ScalarTrap_Update(ScalarTrapProfile_t *p, float dt, float *s, float *sdot);


#endif //CTRBOARD_H7_ALL_TRAJ_SCALAR_TRAP_H
