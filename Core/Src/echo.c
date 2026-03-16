#include "echo.h"
#include "tim.h" // 需要用到定时器的计数器来记录时间

// 私有变量
static uint32_t start_time = 0;
static uint32_t end_time = 0;
static uint8_t  edge_flag = 0; // 0: 等待上升沿, 1: 等待下降沿
static float    last_distance = 0;
static uint32_t last_echo_tick = 0; // 内部记录上次触发时间
/**
 * @brief  向PB9发送10us以上的高电平触发信号
 */
void Echo_Trig(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    
    // 简单的微秒延时 (72MHz下循环约500次接近10-15us)
    for(uint32_t i=0; i<500; i++) __NOP();
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

/**
 * @brief  外部中断回调的具体实现
 * @note   在 PB10 的双边沿中断中记录时间
 */
void Echo_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_10)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) // 上升沿
        {
            // 记录当前定时器的数值（借用电机用的 TIM3 或 TIM4 的 CNT）
            // 假设你的定时器频率是 1MHz (即1us计1次)
            start_time = __HAL_TIM_GET_COUNTER(&htim2); 
            edge_flag = 1;
        }
        else // 下降沿
        {
            if (edge_flag == 1)
            {
                end_time = __HAL_TIM_GET_COUNTER(&htim2);
                
                uint32_t diff = 0;
                if (end_time > start_time) 
                    diff = end_time - start_time;
                else 
                    // 考虑定时器溢出回零的情况
                    diff = (__HAL_TIM_GET_AUTORELOAD(&htim2) - start_time) + end_time;
                
                // 距离 = 时间(us) * 声速(0.034 cm/us) / 2
                last_distance = diff * 0.017f;
                edge_flag = 0;
            }
        }
    }
}

/**
 * @brief  获取最后一次测量的距离值
 */
float Echo_GetDistance(void)
{
    return last_distance;
}
int Echo_Should_Stop(float threshold)
{
    // 1. 每隔 60ms 自动触发一次测距信号
    if (HAL_GetTick() - last_echo_tick > 60) 
    {
        Echo_Trig();
        last_echo_tick = HAL_GetTick();
    }
    
    // 2. 获取当前最新的距离
    float dist = Echo_GetDistance();
    
    // 3. 判断是否满足避障条件
    // 排除掉 1.0cm 以下的异常值（通常是盲区或未收到回响）
    if (dist < threshold && dist > 1.0f) 
    {
        return 1; // 发现障碍物
    }
    
    return 0; // 道路安全
}
