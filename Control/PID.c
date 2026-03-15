
#include <stdint.h>
#include "main.h"
#include "PID.h"

int16_t PID_Compute(PID_t *p, float target, float measured)
{
    p->Target = target;
    p->Actual = measured;
    PID_Update(p);
    return (int16_t)(p->Out);
}

void PID_Init(PID_t *p)
{
    p->Target = 0;
    p->Actual = 0;
    p->Out = 0;
    
    p->Error0 = 0;
    p->Error1 = 0;
    p->ErrorInt = 0;
    
    // 限幅
    p->OutMax = 500;
    p->OutMin = -500;
}


void PID_Update(PID_t *p)
{

    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;
    
    if (p->Ki != 0)
    {
        p->ErrorInt += p->Error0;

    }
    else
    {
        p->ErrorInt = 0;
    }

    p->Out = p->Kp * p->Error0                  // 比例项
           + p->Ki * p->ErrorInt                // 积分项
           + p->Kd * (p->Error0 - p->Error1);   // ：误差变化率（阻尼作用）

    if (p->Out > p->OutMax) p->Out = p->OutMax;
    if (p->Out < p->OutMin) p->Out = p->OutMin;
}