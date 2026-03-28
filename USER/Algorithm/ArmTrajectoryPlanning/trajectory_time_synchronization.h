#ifndef CTRBOARD_H7_ALL_TRAJECTORY_TIME_SYNCHRONIZATION_H
#define CTRBOARD_H7_ALL_TRAJECTORY_TIME_SYNCHRONIZATION_H

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    /* 统一标量进度 s(t) 的速度/加速度约束 */
    float progress_vel_max;        // 进度速度上限
    float progress_acc_max;        // 进度加速度上限

    /* 时间参数 */
    float accel_time;              // 加速段时间
    float cruise_time;             // 匀速段时间（梯形轨迹时有效）
    float total_time;              // 总时间

    /* 关键进度/速度参数 */
    float accel_end_progress;      // 加速段结束时的进度
    float peak_progress_vel;       // 实际峰值进度速度

    /* 运行状态 */
    float current_time;            // 当前内部时间
    uint8_t use_triangle_profile;  // 1=三角速度, 0=梯形速度
    uint8_t running;               // 1=轨迹进行中, 0=轨迹结束
} SpeedTimeSYNC_t;

/**
 * @brief 启动一个从 s=0 到 s=1 的标量梯形/三角速度轨迹
 * @param profile   轨迹对象
 * @param v_max     进度最大速度，必须 > 0
 * @param a_max     进度最大加速度，必须 > 0
 * @return true     启动成功
 * @return false    参数非法
 */
bool TrajTimeSync_Start(SpeedTimeSYNC_t *profile, float v_max, float a_max);

/**
 * @brief 更新轨迹，输出当前时刻的 s 和 sdot
 *
 * 说明：
 * - 本函数返回的是“当前 current_time 对应的采样值”
 * - 然后内部时间再推进 dt
 * - 所以 Start 后第一次调用 Update，会先输出 s=0, sdot=0
 *
 * @param profile       轨迹对象
 * @param dt            控制周期，单位秒，建议 > 0
 * @param progress      当前进度 s，范围 [0,1]
 * @param progress_vel  当前进度速度 sdot
 * @return true         输出有效
 * @return false        参数非法
 */
bool TrajTimeSync_Update(SpeedTimeSYNC_t *profile,
                       float dt,
                       float *progress,
                       float *progress_vel);

#endif