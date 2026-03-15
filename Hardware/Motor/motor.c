#include "motor.h"
#include "tim.h"   // �������ã����򱨴� &htim3

// ��ʼ������
void Motor_Init(void)
{
    // ����PWMͨ��
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

// PB14(AIN1), PB15(AIN2)

static void Set_MotorA_Speed(int speed)
{
    if (speed == 0) // 强制刹车状态
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); // PWM清零
        return;
    }
    else if (speed > 0) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
    else 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        speed = -speed;
    }
    
    if(speed > 999) speed = 999;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
}

// �ڲ������������ҵ�� (Motor B)
// PB13(BIN1), PB12(BIN2)
static void Set_MotorB_Speed(int speed)
{
    if (speed == 0) // 强制刹车状态
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0); // PWM清零
        return;
    }
    else if (speed > 0) 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
    else 
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        speed = -speed;
    }

    if(speed > 999) speed = 999;

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
}

// �ⲿ���ýӿ�
void Motor_SetSpeed(int speed_A, int speed_B)
{
    Set_MotorA_Speed(speed_A);
    Set_MotorB_Speed(speed_B);
}