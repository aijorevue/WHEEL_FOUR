#ifndef __ECHO_H
#define __ECHO_H

#include "main.h"

// 测距函数，返回值为厘米(cm)
float Echo_GetDistance(void);

// 触发测距的信号发送函数
void Echo_Trig(void);

// 内部使用的中断处理逻辑（会被main.c的中断回调调用）
void Echo_EXTI_Callback(uint16_t GPIO_Pin);
int Echo_Should_Stop(float threshold);
void DWT_Init(void);
void Echo_Trig(void);
int Echo_Should_Stop(float threshold);

#endif