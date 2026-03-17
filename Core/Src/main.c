/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 整合了状态机、PID和避障的优化版本
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
#include "PID.h"  
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STOP_FLAG 99
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
PID_t linePID;  
int8_t last_error = 0; 

// 定义状态枚举
typedef enum {
    STAGE_1_SEARCH_TURN = 0, // 第一阶段：循线并寻找转弯点
    STAGE_2_SEARCH_AVOID,    // 第二阶段：循线并寻找障碍物
    STAGE_3_FINAL_FOLLOW,     // 第三阶段：最后纯循线
    STAGE_4_servo
} RunStage_t;

RunStage_t current_stage = STAGE_1_SEARCH_TURN;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
int8_t Get_Line_Error(uint8_t state);
void Do_Line_Follow(uint8_t status, int8_t err);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * 误差提取逻辑
 */
int8_t Get_Line_Error(uint8_t state) {
    switch (state) {
        case 0x06: return 0;   // 0110
        case 0x04: return -1;  
        case 0x0C: return -2;  
        case 0x08: return -3;  
        case 0x0E: return -4;  
        case 0x0D: return -5;  
        case 0x02: return 1;   
        case 0x03: return 2;   
        case 0x01: return 3;   
        case 0x07: return 4;   
        case 0x0B: return 5;   
        case 0x05: return 0;   
        case 0x0A: return 0;   
        case 0x09: return 0;   
        case 0x00: return STOP_FLAG; 
        case 0x0F: return STOP_FLAG; 
        default: return 0;
    }
}

/**
 * 封装好的 PID 循线动作函数
 */
void Do_Line_Follow(uint8_t status, int8_t err) {
    if (err == STOP_FLAG) {
        if (status == 0x00) {
            // 彻底丢线，根据上一次误差原地找线
            if (last_error > 0)      Motor_SetSpeed(-130, 130);
            else if (last_error < 0) Motor_SetSpeed(130, -130);
            else                     Motor_SetSpeed(0, 0);
        } else {
            // 这里处理 0x0F (全黑/交叉口)
            // 在循线过程中遇到全黑，保持直行一段距离，而不是停车
            Motor_SetSpeed(0, 0); 
        }
    } else {
        last_error = err; 
        int16_t base_speed = 150; 
        int16_t turn_offset = (int16_t)PID_Compute(&linePID, 0.0f, (float)err);
        Motor_SetSpeed(base_speed + turn_offset, base_speed - turn_offset);
    }
}
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
  Motor_Init(); 
  HAL_TIM_Base_Start(&htim2); 

  PID_Init(&linePID); 
  linePID.Kp = 50.0f;   
  linePID.Ki = 0.0f;    
  linePID.Kd = 20.0f;   
  linePID.OutMax = 200; 
  linePID.OutMin = -200;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint8_t sensor_status = Sensor_Read_Tracking();
    int8_t error = Get_Line_Error(sensor_status);

    // --- 核心状态机逻辑 ---
    switch (current_stage) {
        
        case STAGE_1_SEARCH_TURN:
            // 触发条件：识别到 -4 (左偏大) 或 0x0F (路口黑块)
            if (sensor_status == 0x0E || (sensor_status == 0x0F && error == -4)) {
                Motor_SetSpeed(200, 0); // 执行大左转
                HAL_Delay(1000);           // 强制执行 1.6秒
                current_stage = STAGE_2_SEARCH_AVOID; // 完成后切到避障阶段
            } else {
                Do_Line_Follow(sensor_status, error); 
            }
            break;

        case STAGE_2_SEARCH_AVOID:
            // 实时检查避障传感器
            if (HAL_GPIO_ReadPin(GPIOC, Avoid_L_Pin) == GPIO_PIN_RESET || 
                HAL_GPIO_ReadPin(GPIOC, Avoid_R_Pin) == GPIO_PIN_RESET) {
                
                Motor_SetSpeed(0, 200); // 避障动作 A
                HAL_Delay(1700);
                Motor_SetSpeed(200, 0); // 避障动作 B
                HAL_Delay(1700);
                Motor_SetSpeed(180, 180); // 避障动作 C
                HAL_Delay(1800);
                Motor_SetSpeed(200, 0);
                HAL_Delay(1700);
                Motor_SetSpeed(180, 180); // 避障动作 C
                HAL_Delay(1800);
                Motor_SetSpeed(0, 200);
                HAL_Delay(1700);
                current_stage = STAGE_3_FINAL_FOLLOW; // 避障完切到最后阶段
            } else {
                Do_Line_Follow(sensor_status, error);
            }
            break;

        case STAGE_3_FINAL_FOLLOW:
            if(sensor_status == 0x0F) {
                Motor_SetSpeed(0, 0); // 停车
                current_stage = STAGE_4_servo; // 切到伺服阶段
            } else {
                Do_Line_Follow(sensor_status, error);
            }
            break;
    }

    HAL_Delay(5); 
  }
  /* USER CODE END WHILE */
}

/* 后续 SystemClock_Config 等函数保持不变... */
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
// 如果不需要其他外部中断，此回调函数可留空或删除内容
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}
/* USER CODE END 4 */