#ifndef __MOTO_H
#define __MOTO_H

#include "main.h"


void Motor_Init(void);

/**
  * @brief  设置电机速度
  * @param  speed_A: 电机A（通常为左电机）速度，范围 -999 到 999
  *                  正数为前进，负数为后退，0为刹车停止
  * @param  speed_B: 电机B（通常为右电机）速度，范围 -999 到 999
  *                  正数为前进，负数为后退，0为刹车停止
  * @retval 无
  */
void Motor_SetSpeed(int speed_A, int speed_B);

#endif /* __MOTOR_H */