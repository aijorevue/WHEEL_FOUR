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
        case 0x06: return 0;   // 0110 正中心
        
        // 稍微偏移
        case 0x04: return -1;  // 0100 
        case 0x02: return 1;   // 0010 
        
        // 中度偏移
        case 0x0C: return -2;  // 1100
        case 0x03: return 2;   // 0011
        
        // 严重偏移（外侧触发）
        // 因为外侧远，赋予大权重(5)，强制PID输出大动作防止丢线
        case 0x08: return -5;  // 1000 极左
        case 0x01: return 5;   // 0001 极右
        
        // 停止状态
        case 0x00: return STOP_FLAG; // 全白：停止
        case 0x0F: return STOP_FLAG; // 全黑：停止
        
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
  linePID.Kp = 55.0f;   // 基础转向灵敏度
  linePID.Ki = 0.0f;    // 循线通常设为0
  linePID.Kd = 20.0f;   // 阻尼，防止摆动
  linePID.OutMax = 400; // 最大转向补偿
  linePID.OutMin = -400;
  /* USER CODE END 2 */

  /* Infinite loop */
  
  while (1)
  {
      // 1. 调用封装好的避障判断函数 (设置15cm为阈值)
      if (Echo_Should_Stop(15.0f)) 
      {
          Motor_SetSpeed(0, 0); // 优先级最高：停车
      }
      else 
      {
          // 2. 只有安全时才执行循线
          uint8_t sensor_status = Sensor_Read_Tracking();
          int8_t error = Get_Line_Error(sensor_status);
          
          if (error == STOP_FLAG) 
          {
              Motor_SetSpeed(0, 0); 
          }
          else 
          {
              int16_t base_speed = 300; 
              if (error >= 5 || error <= -5) base_speed = 150; 
              
              int16_t turn_offset = (int16_t)PID_Compute(&linePID, 0.0f, (float)error);
              Motor_SetSpeed(base_speed + turn_offset, base_speed - turn_offset);
          }
      }
      
      HAL_Delay(5); // 给 PID 留 5ms 采样间隔
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
