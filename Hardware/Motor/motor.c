#include "motor.h"
#include "tim.h"   // 必须引用，否则报错 &htim3

// 初始化函数
void Motor_Init(void)
{
    // 开启PWM通道
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

// 内部函数：设置左电机 (Motor A)
// PB14(AIN1), PB15(AIN2)
static void Set_MotorA_Speed(int speed)
{
    if (speed >= 0) // 前进
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   // High
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // Low
    }
    else // 后退
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Low
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   // High
        speed = -speed; // 取绝对值
    }
    
    // 限制最大PWM (ARR=999)
    if(speed > 999) speed = 999;
    
    // 设置PWM占空比 (PB1 -> TIM3 Channel 4)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
}

// 内部函数：设置右电机 (Motor B)
// PB13(BIN1), PB12(BIN2)
static void Set_MotorB_Speed(int speed)
{
    if (speed >= 0) // 前进
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   // High
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // Low
    }
    else // 后退
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Low
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   // High
        speed = -speed;
    }

    if(speed > 999) speed = 999;

    // 设置PWM占空比 (PB0 -> TIM3 Channel 3)
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
}

// 外部调用接口
void Motor_SetSpeed(int speed_A, int speed_B)
{
    Set_MotorA_Speed(speed_A);
    Set_MotorB_Speed(speed_B);
}