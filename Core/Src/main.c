/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "sensors.h"
#include "PID.h"  // 确保你的PID.h中定义了 PID_t 类型
#include "echo.h" // 超声波测距相关函数
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STOP_FLAG 99
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
PID_t linePID;  // <<-- 修正 1: 定义PID结构体变量
int8_t last_error = 0; // 用于记录上一次有效的误差
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
int8_t Get_Line_Error(uint8_t state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * 误差提取逻辑（针对传感器间距不均优化）
 * 状态定义: L2 L1 R1 R2 (从左到右)
 * 灰线返回 1
 */

int8_t Get_Line_Error(uint8_t state) {
    switch (state) {
        /* ---------- 第一类：正中心 (0 误差) ---------- */
        case 0x06: return 0;   // 0110：中间两路在白线上

        /* ---------- 第二类：左偏 (误差为负，需要右转) ---------- */
        case 0x04: return -1;  // 0100：中心稍左
        case 0x0C: return -2;  // 1100：中度左偏
        case 0x08: return -3;  // 1000：严重左偏（极左）
        case 0x0E: return -4;  // 1110：大面积偏左（可能在急弯边缘）
        case 0x0D: return -5;  // 1101：非典型左偏

        /* ---------- 第三类：右偏 (误差为正，需要左转) ---------- */
        case 0x02: return 1;   // 0010：中心稍右
        case 0x03: return 2;   // 0011：中度右偏
        case 0x01: return 3;   // 0001：严重右偏（极右）
        case 0x07: return 4;   // 0111：大面积偏右（可能在急弯边缘）
        case 0x0B: return 5;   // 1011：非典型右偏

        /* ---------- 第四类：特殊状态 (丢线/特殊形状) ---------- */
        case 0x05: return 0;   // 0101：中间跳跃，可能是干扰，维持原状
        case 0x0A: return 0;   // 1010：中间跳跃，可能是干扰，维持原状
        case 0x09: return 0;   // 1001：两边亮中间灭，可能是十字路口初期

        /* ---------- 第五类：停止/丢线状态 (STOP_FLAG) ---------- */
        case 0x00: return STOP_FLAG; // 0000：全白，可能是横线或彻底丢线
        case 0x0F: return STOP_FLAG; // 1111：全黑，可能是彻底丢线

        default: return 0;
    }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  Motor_Init(); // 内部会开启TIM3 PWM
   HAL_TIM_Base_Start(&htim2); 
  // <<-- 修正 2: 初始化PID参数
  // 根据你的实际硬件调整 Kp, Ki, Kd
  PID_Init(&linePID); 
  linePID.Kp = 50.0f;   // 基础转向灵敏度
  linePID.Ki = 0.0f;    // 循线通常设为0
  linePID.Kd = 20.0f;   // 阻尼，防止摆动
  linePID.OutMax = 200; // 最大转向补偿
  linePID.OutMin = -200;
  /* USER CODE END 2 */

  /* Infinite loop */
  
 /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. 获取传感器状态和误差
    uint8_t sensor_status = Sensor_Read_Tracking();
    int8_t error = Get_Line_Error(sensor_status);
    if(error==-4){
      Motor_SetSpeed(120, -100);
      HAL_Delay(1600);
    }
    
    if(error==3){
      Motor_SetSpeed(100, 120);
      HAL_Delay(1000);
    }
    // 2. 第一层：避障优先级最高
    if (Echo_Should_Stop(15.0f)) 
    {
        Motor_SetSpeed(0, 0);
        HAL_Delay(10000); // 停止10秒，等待障碍物移开
    }
    
    // 3. 第二层：停止或丢线处理
    else if (error == STOP_FLAG) 
    {
        // 如果是全白(0x0F)丢线，根据最后一次误差记忆，原地自旋找线
        // 这可以防止小车过弯太猛冲出赛道后就“死”在那边
        if (sensor_status == 0x00) 
        {
            if (last_error > 0)      Motor_SetSpeed(-150, 150); // 向右找线
            else if (last_error < 0) Motor_SetSpeed(150, -150); // 向左找线
            else                     Motor_SetSpeed(0, 0);
        }
        else // 全黑或其他停止状态
        {
            Motor_SetSpeed(0, 0);
            
        }
    }

    // 4. 第三层：硬编码处理急弯 (当误差绝对值 >= 2 时)
    // 包含状态：0x0C, 0x08, 0x0E, 0x0D (左偏) 和 0x03, 0x01, 0x07, 0x0B (右偏)
    
    else 
    {
        last_error = error; // 记录最后一次有效误差
        
        int16_t base_speed = 150; // 直道可以给高一点的基础速度
        
        // PID计算
        int16_t turn_offset = (int16_t)PID_Compute(&linePID, 0.0f, (float)error);
        
        // 限制转向补偿范围，防止在直道猛晃
        Motor_SetSpeed(base_speed + turn_offset, base_speed - turn_offset);
    }

    // 6. 循环小延迟，保持 200Hz 的采样频率
    HAL_Delay(5); 
  }


}


/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 调用我们单开的文件里的处理函数
    Echo_EXTI_Callback(GPIO_Pin);
}
/* USER CODE END 4 */
