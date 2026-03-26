#include "traj_scalar_trap.h"
#include <math.h>

#define SCALAR_TRAP_DIST   (1.0f)
#define SCALAR_TRAP_EPS    (1e-6f)

static float clampf(float x, float xmin, float xmax)
{
    if (x < xmin) return xmin;
    if (x > xmax) return xmax;
    return x;
}

bool ScalarTrap_Start(ScalarTrapProfile_t *p, float v_max, float a_max)
{
    float t_acc_try;
    float s_acc_try;

    if ((p == 0) || (v_max <= SCALAR_TRAP_EPS) || (a_max <= SCALAR_TRAP_EPS))
    {
        return false;
    }

    /* 清零关键状态 */
    p->v_max = v_max;
    p->a_max = a_max;
    p->t = 0.0f;
    p->is_triangle = 0;
    p->active = 1;

    /* 先按“能跑到最大速度”的梯形情况尝试 */
    t_acc_try = v_max / a_max;
    s_acc_try = 0.5f * a_max * t_acc_try * t_acc_try;

    if ((2.0f * s_acc_try) < SCALAR_TRAP_DIST)
    {
        /* 标准梯形速度 */
        p->is_triangle = 0;
        p->t_acc = t_acc_try;
        p->s_acc = s_acc_try;
        p->v_peak = v_max;
        p->t_flat = (SCALAR_TRAP_DIST - 2.0f * p->s_acc) / p->v_peak;
        p->t_total = 2.0f * p->t_acc + p->t_flat;
    }
    else
    {
        /* 三角速度：距离太短，来不及跑到 v_max */
        p->is_triangle = 1;
        p->t_acc = sqrtf(SCALAR_TRAP_DIST / a_max);
        p->s_acc = 0.5f * a_max * p->t_acc * p->t_acc;   /* 理论上 = 0.5 */
        p->v_peak = a_max * p->t_acc;
        p->t_flat = 0.0f;
        p->t_total = 2.0f * p->t_acc;
    }

    return true;
}

bool ScalarTrap_Update(ScalarTrapProfile_t *p, float dt, float *s, float *sdot)
{
    float t_now;
    float td;

    if ((p == 0) || (s == 0) || (sdot == 0) || (dt < 0.0f))
    {
        return false;
    }

    /* 如果轨迹已经结束，持续输出终点 */
    if (p->active == 0)
    {
        *s = 1.0f;
        *sdot = 0.0f;
        return true;
    }

    /* 取当前时刻做输出 */
    t_now = p->t;

    if (t_now <= 0.0f)
    {
        *s = 0.0f;
        *sdot = 0.0f;
    }
    else if (t_now < p->t_acc)
    {
        /* 加速段 */
        *s = 0.5f * p->a_max * t_now * t_now;
        *sdot = p->a_max * t_now;
    }
    else if ((p->is_triangle == 0) && (t_now < (p->t_acc + p->t_flat)))
    {
        /* 匀速段，仅梯形存在 */
        *s = p->s_acc + p->v_peak * (t_now - p->t_acc);
        *sdot = p->v_peak;
    }
    else if (t_now < p->t_total)
    {
        /* 减速段 */
        if (p->is_triangle)
        {
            td = t_now - p->t_acc;
            *s = p->s_acc + p->v_peak * td - 0.5f * p->a_max * td * td;
            *sdot = p->v_peak - p->a_max * td;
        }
        else
        {
            td = t_now - (p->t_acc + p->t_flat);
            *s = p->s_acc
                 + p->v_peak * p->t_flat
                 + p->v_peak * td
                 - 0.5f * p->a_max * td * td;
            *sdot = p->v_peak - p->a_max * td;
        }
    }
    else
    {
        /* 完成段 */
        *s = 1.0f;
        *sdot = 0.0f;
        p->active = 0;
        return true;
    }

    /* 数值保护 */
    *s = clampf(*s, 0.0f, 1.0f);
    if (*sdot < 0.0f)
    {
        *sdot = 0.0f;
    }

    /* 推进到下一个周期 */
    p->t += dt;
    if (p->t >= p->t_total)
    {
        p->t = p->t_total;
    }

    return true;
}