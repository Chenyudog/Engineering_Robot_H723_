#include "joint_movej_planner.h"
#include <string.h>
#include <math.h>

static float minf_local(float a, float b)
{
    return (a < b) ? a : b;
}

void JointMoveJ_Init(JointMoveJPlanner_t *jp)
{
    if (jp == 0)
    {
        return;
    }

    memset(jp, 0, sizeof(JointMoveJPlanner_t));
    jp->state = JP_IDLE;
}

void JointMoveJ_Stop(JointMoveJPlanner_t *jp)
{
    int i;

    if (jp == 0)
    {
        return;
    }

    jp->prof.active = 0;
    jp->state = JP_ABORT;

    for (i = 0; i < JOINT_NUM; i++)
    {
        jp->v_ref[i] = 0.0f;
    }
}

bool JointMoveJ_Start(JointMoveJPlanner_t *jp,
                      const float q0[JOINT_NUM],
                      const float q1[JOINT_NUM],
                      const float vmax[JOINT_NUM],
                      const float amax[JOINT_NUM])
{
    int i;
    float adq;
    bool found_moving_joint = false;

    if ((jp == 0) || (q0 == 0) || (q1 == 0) || (vmax == 0) || (amax == 0))
    {
        return false;
    }

    memset(jp, 0, sizeof(JointMoveJPlanner_t));

    jp->vs_max = 1e30f;
    jp->as_max = 1e30f;
    jp->moving_joint_count = 0;

    for (i = 0; i < JOINT_NUM; i++)
    {
        jp->q0[i] = q0[i];
        jp->q1[i] = q1[i];
        jp->dq[i] = q1[i] - q0[i];

        jp->vmax[i] = vmax[i];
        jp->amax[i] = amax[i];

        jp->q_ref[i] = q0[i];
        jp->v_ref[i] = 0.0f;

        adq = fabsf(jp->dq[i]);

        /* 静止关节不参与全局约束 */
        if (adq < JOINT_EPS_MOVE)
        {
            continue;
        }

        /* 有运动的关节，其 vmax/amax 必须合法 */
        if ((vmax[i] <= 0.0f) || (amax[i] <= 0.0f))
        {
            jp->state = JP_FAULT;
            return false;
        }

        jp->vs_max = minf_local(jp->vs_max, vmax[i] / adq);
        jp->as_max = minf_local(jp->as_max, amax[i] / adq);

        jp->moving_joint_count++;
        found_moving_joint = true;
    }

    /* 所有关节都不用动：直接完成 */
    if (!found_moving_joint)
    {
        for (i = 0; i < JOINT_NUM; i++)
        {
            jp->q_ref[i] = jp->q1[i];
            jp->v_ref[i] = 0.0f;
        }

        jp->state = JP_DONE;
        return true;
    }

    if ((jp->vs_max <= 0.0f) || (jp->as_max <= 0.0f))
    {
        jp->state = JP_FAULT;
        return false;
    }

    if (!ScalarTrap_Start(&jp->prof, jp->vs_max, jp->as_max))
    {
        jp->state = JP_FAULT;
        return false;
    }

    jp->state = JP_RUNNING;
    return true;
}

bool JointMoveJ_Update(JointMoveJPlanner_t *jp, float dt)
{
    int i;
    float s, sdot;

    if (jp == 0)
    {
        return false;
    }

    if (jp->state == JP_DONE)
    {
        for (i = 0; i < JOINT_NUM; i++)
        {
            jp->q_ref[i] = jp->q1[i];
            jp->v_ref[i] = 0.0f;
        }
        return true;
    }

    if (jp->state != JP_RUNNING)
    {
        return false;
    }

    if (!ScalarTrap_Update(&jp->prof, dt, &s, &sdot))
    {
        jp->state = JP_FAULT;
        return false;
    }

    for (i = 0; i < JOINT_NUM; i++)
    {
        jp->q_ref[i] = jp->q0[i] + jp->dq[i] * s;
        jp->v_ref[i] = jp->dq[i] * sdot;
    }

    if (jp->prof.active == 0)
    {
        for (i = 0; i < JOINT_NUM; i++)
        {
            jp->q_ref[i] = jp->q1[i];
            jp->v_ref[i] = 0.0f;
        }
        jp->state = JP_DONE;
    }

    return true;
}