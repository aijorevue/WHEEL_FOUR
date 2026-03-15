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
#include "OLED.h"     
#include "PID.h"   
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 定义一个全黑停车的特殊标志位
#define STOP_FLAG 99

// 1. 状态压缩：直接调用传感器模块接口
uint8_t Get_Sensor_State(void)
{
    // 直接调用Sensor_Read_Tracking()，返回4位状态
    return Sensor_Read_Tracking();
}

// 2. 误差计算（精细化分类 + 识别直角 + 识别全黑）
int8_t Calculate_Error(uint8_t state)
{
    static int8_t last_error = 0; 
    int8_t error = 0;

    switch (state)
    {
        // === 第一梯队：平稳走线（微调） ===
        case 0x06: error = 0;  break; // 0110：完美居中（中间两探头压线）
        case 0x04: error = -1; break; // 0100：偏右一点
        case 0x02: error = 1;  break; // 0010：偏左一点
        case 0x0C: error = -2; break; // 1100：偏右较多
        case 0x03: error = 2;  break; // 0011：偏左较多

        // === 第二梯队：直角转弯先兆===
        case 0x08: error = -4; break; // 1000：极度偏右
        case 0x01: error = 4;  break; // 0001：极度偏左
        case 0x0E: error = -5; break; // 1110：标准的左直角弯
        case 0x07: error = 5;  break; // 0111：标准的右直角弯

        // === 第三梯队：全黑指示线停车 ===
        case 0x0F: return STOP_FLAG;  // 1111：四路全亮，遇到全黑的垂直指示线

        // === 第四梯队：完全脱线（依赖记忆补救） ===
        case 0x00: 
            if (last_error > 0)
                error = 5;   // 极限右转找线
            else if (last_error < 0)
                error = -5;  // 极限左转找线
            else
                error = 0;   
            break;

        // 其他未知噪点状态
        default:
            error = last_error; 
            break;
    }
    
    
    if (state != 0x00 && state != 0x0F) {
        last_error = error; 
    }
    
    return error;
}

// 3. 主控制流：加入动态降速与停车逻辑
void Track_Process(PID_t *pid, int16_t Base_Speed)
{
    uint8_t sensor_state = Get_Sensor_State(); 
    int8_t error = Calculate_Error(sensor_state); 

    // --- 1. 停车逻辑 ---
    if (error == STOP_FLAG) 
    {
        Motor_SetSpeed(0, 0); // 彻底刹停电机
        
        // 注意：如果你希望小车停下后就彻底罢工不再动了，请把下面这行代码的注释(//)去掉
        //while(1) { Motor_SetSpeed(0, 0); } 
        
        return; // 直接退出，不再进行PID计算
    }

    // --- 2. 直角转弯“动态降速”逻辑 ---
    int16_t current_base_speed = Base_Speed;
    if (error >= 4 || error <= -4) 
    {
        current_base_speed = Base_Speed / 2; // 遇到急弯/直角，基础速度减半，防冲出
    }

    // --- 3. PID 计算与输出 ---
    int16_t Turn_Compensate = PID_Compute(pid, 0, error); 
    
    int16_t speed_L = current_base_speed + Turn_Compensate; 
    int16_t speed_R = current_base_speed - Turn_Compensate; 

    Motor_SetSpeed(speed_L, speed_R); 
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  
  /* USER CODE BEGIN 2 */

  // 1. 硬件初始化
  // ---------------------------------------------------------
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE(); 
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(200); 
  // ---------------------------------------------------------

  Motor_Init();
  OLED_Init();
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  
  // 2. PID 初始化
  PID_t trackPID;
  PID_Init(&trackPID);
  
  // 设置 PID 参数 (还没有调试)
  trackPID.Kp = 50.0f;  // 初始P值
  trackPID.Ki = 0.0f;   // 循迹通常不需要I
  trackPID.Kd = 20.0f;   // 初始D值
  trackPID.OutMax = 400; // 最大差速限制
  trackPID.OutMin = -400;

  // 3. 初始界面显示
  OLED_Clear();
  OLED_ShowString(0, 0,  "Track:", OLED_8X16);
  OLED_ShowString(0, 20, "PID:",   OLED_8X16);
  OLED_ShowString(0, 40, "Mode:",  OLED_8X16);
  OLED_Update();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    // 主循环只用核心函数，结构更简洁
    while (1)
    {
        Track_Process(&trackPID, 300); // 基础速度300
        HAL_Delay(10);
    }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
