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
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STOP_FLAG 99
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
PID_t linePID;  
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
 * 误差提取逻辑
 */
int8_t Get_Line_Error(uint8_t state) {
    switch (state) {
        /* ---------- 第一类：正中心 (0 误差) ---------- */
        case 0x06: return 0;   // 0110

        /* ---------- 第二类：左偏 (误差为负) ---------- */
        case 0x04: return -1;  
        case 0x0C: return -2;  
        case 0x08: return -3;  
        case 0x0E: return -4;  
        case 0x0D: return -5;  

        /* ---------- 第三类：右偏 (误差为正) ---------- */
        case 0x02: return 1;   
        case 0x03: return 2;   
        case 0x01: return 3;   
        case 0x07: return 4;   
        case 0x0B: return 5;   

        /* ---------- 第四类：特殊状态 ---------- */
        case 0x05: return 0;   
        case 0x0A: return 0;   
        case 0x09: return 0;   

        /* ---------- 第五类：停止/丢线状态 ---------- */
        case 0x00: return STOP_FLAG; 
        case 0x0F: return STOP_FLAG; 

        default: return 0;
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
  Motor_Init(); // 内部会开启TIM3 PWM
  HAL_TIM_Base_Start(&htim2); 

  // 初始化PID参数
  PID_Init(&linePID); 
  linePID.Kp = 50.0f;   
  linePID.Ki = 0.0f;    
  linePID.Kd = 20.0f;   
  linePID.OutMax = 200; 
  linePID.OutMin = -200;
  /* USER CODE END 2 */
  int flag_1=0;
  int flag_2=0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* Infinite loop */
  while (1)
  {
    uint8_t sensor_status = Sensor_Read_Tracking();
    int8_t error = Get_Line_Error(sensor_status);

    // --- 第一阶段：优先转弯触发 ---
    // 只有在 flag_1 还没用过，且正好误差是 -4 时，才执行特殊动作
    if (flag_1 == 0 && error == -4) {
        Motor_SetSpeed(120, -100);
        HAL_Delay(1600);
        flag_1 = 1; // 标记转弯已完成
        // 执行完特殊动作后，不需要 continue，让它继续往下走或者进入下一轮读取
    }

    // --- 第二阶段：避障触发 ---
    // 只有在转弯完成(flag_1==1)且避障还没做过(flag_2==0)时，才检查避障
    else if (flag_1 == 1 && flag_2 == 0) {
        if (HAL_GPIO_ReadPin(GPIOC, Avoid_L_Pin) == GPIO_PIN_RESET || 
            HAL_GPIO_ReadPin(GPIOC, Avoid_R_Pin) == GPIO_PIN_RESET) {
            Motor_SetSpeed(0, 200);
            HAL_Delay(2000);
            Motor_SetSpeed(200, 0);
            HAL_Delay(2000);
            flag_2 = 1; // 标记避障已完成
        }
        // 如果没有避障，就正常往下走，执行下面的 PID 循线
    }

    // --- 默认逻辑：正常循线 ---
    // 只要没有在执行上面的特殊动作，每一轮循环都会跑到这里执行 PID
    if (error == STOP_FLAG) 
    {
        if (sensor_status == 0x00) {
            if (last_error > 0)      Motor_SetSpeed(-150, 150);
            else if (last_error < 0) Motor_SetSpeed(150, -150);
            else                     Motor_SetSpeed(0, 0);
        } else {
            Motor_SetSpeed(0, 0);
        }
    }
    else 
    {
        last_error = error; 
        int16_t base_speed = 150; 
        int16_t turn_offset = (int16_t)PID_Compute(&linePID, 0.0f, (float)error);
        Motor_SetSpeed(base_speed + turn_offset, base_speed - turn_offset);
    }

    HAL_Delay(5); 
  }                                                                                                                                                             
    /* USER CODE END WHILE */
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
// 如果不需要其他外部中断，此回调函数可留空或删除内容
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
}
/* USER CODE END 4 */