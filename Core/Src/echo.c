/* 在 echo.c 的顶部添加 */
#include "main.h"

// 初始化 DWT 计数器
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 开启 Trace
    DWT->CYCCNT = 0;                                // 计数器清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;           // 开启循环计数器
}

// 获取当前微秒数 (针对 72MHz 系统频率)
uint32_t DWT_GetUs(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000);
}

// 微秒级阻塞延时
void DWT_Delay_us(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - startTick) < delayTicks);
}
static uint32_t start_cycles = 0;
static float last_distance = 0;
static uint32_t last_echo_tick = 0;

/**
 * @brief  触发超声波
 */
void Echo_Trig(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    DWT_Delay_us(15); // 使用 DWT 进行精准 15us 延时
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

/**
 * @brief  PB10 双边沿中断回调
 */
void Echo_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_10) {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) {
            // 上升沿：记录当前的周期数
            start_cycles = DWT->CYCCNT;
        } 
        else {
            // 下降沿：计算周期差
            uint32_t end_cycles = DWT->CYCCNT;
            uint32_t diff = 0;
            
            if (end_cycles > start_cycles)
                diff = end_cycles - start_cycles;
            else
                diff = (0xFFFFFFFF - start_cycles) + end_cycles; // 考虑32位溢出

            // 距离计算：
            // 1. 先将 diff 周期数转为微秒：diff / 72.0
            // 2. 距离 = 时间(us) * 0.034 / 2
            last_distance = (diff / 72.0f) * 0.017f;
        }
    }
}

float Echo_GetDistance(void) {
    return last_distance;
}

// 判断避障函数保持不变...
int Echo_Should_Stop(float threshold) {
    if (HAL_GetTick() - last_echo_tick > 60) {
        Echo_Trig();
        last_echo_tick = HAL_GetTick();
    }
    
    float dist = Echo_GetDistance();
    // 过滤掉 HC-SR04 常见的异常值 0 或 超过 400cm
    if (dist > 2.0f && dist < threshold) {
        return 1;
    }
    return 0;
}