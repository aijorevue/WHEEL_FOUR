
#ifndef __PID_H
#define __PID_H
#include <stdint.h>

#include <stdint.h>

// 定义 PID 结构体
typedef struct {
    // 目标值与实际值
    float Target;      // 我们想要达到的目标（比如误差为0）
    float Actual;      // 当前实际的值（比如传感器读到的误差）
    float Out;         // PID计算出的输出值（给电机的差速）
    
    // PID 三大参数
    float Kp;          // 比例系数 (反应快慢)
    float Ki;          // 积分系数 (消除静差，循迹通常给0)
    float Kd;          // 微分系数 (预判趋势，减少震荡)
    
    // 误差记忆（用于计算积分和微分）
    float Error0;      // 当前误差
    float Error1;      // 上一次误差
    float ErrorInt;    // 误差积分累加值
    
    // 输出限幅（防止电机疯转）
    float OutMax;      // 输出上限
    float OutMin;      // 输出下限

} PID_t;

// 函数声明
void PID_Init(PID_t *p);
void PID_Update(PID_t *p);
int16_t PID_Compute(PID_t *p, float target, float measured);

#endif