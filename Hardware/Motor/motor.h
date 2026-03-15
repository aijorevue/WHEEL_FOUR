#ifndef __MOTO_H
#define __MOTO_H

#include "main.h"

// �����/��ת����
void moto(int mode);

// ���ҵ���ջ�����
int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);

// �����߼���װ����
void Motor_Stop(void);       // ֹͣ���
void Motor_Forward(void);    // ǰ��
void Motor_Back(void);       // ����
void Motor_Left(void);       // ��ת
void Motor_Right(void);      // ��ת

#endif
