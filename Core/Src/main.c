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
#include "PID.h"   
#include "oled.h"    // <<-- 修正 1：必须包含 OLED 头文件
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_PORT GPIOB
#define TRIG_PIN  GPIO_PIN_9
#define ECHO_PORT GPIOB
#define ECHO_PIN  GPIO_PIN_10
#define SERVO_PIN GPIO_PIN_8
#define SERVO_PORT GPIOB

// 避障距离阈值 (厘米)
#define OBSTACLE_LIMIT 5.0f
#define STOP_FLAG 99
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
PID_t trackPID; 
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* --- 硬件驱动函数声明 --- */
void DWT_Init(void);
void DWT_DelayUs(uint32_t us);
float Get_Distance(void);
void Servo_SetAngle(float angle);

/* --- 循线与控制函数声明 --- */
uint8_t Get_Sensor_State(void);
int8_t Calculate_Error(uint8_t state);
void Track_Process(PID_t *pid, int16_t Base_Speed);
void Detour_Sequence(void);

/* 注意：这里的参数类型 int16_t 必须与 motor.c 中完全一致 */
void Motor_SetSpeed(int16_t Left, int16_t Right);
void Motor_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; 
    DWT->CYCCNT = 0;                                
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            
}

void DWT_DelayUs(uint32_t us) {
    uint32_t startTicks = DWT->CYCCNT;
    uint32_t targetTicks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - startTicks) < targetTicks);
}

float Get_Distance(void) {
    uint32_t ticks;
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    DWT_DelayUs(15);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

    uint32_t timeout = 100000;
    while(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET && timeout--);
    if(timeout == 0) return 999.0f;

    uint32_t start_ticks = DWT->CYCCNT;
    while(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET);
    ticks = DWT->CYCCNT - start_ticks;

    return (float)ticks * 0.034f / 2.0f / (SystemCoreClock / 1000000);
}

void Servo_SetAngle(float angle) {
    uint32_t pulse_us = (uint32_t)(500 + (angle / 180.0f) * 2000);
    HAL_GPIO_WritePin(SERVO_PORT, SERVO_PIN, GPIO_PIN_SET);
    DWT_DelayUs(pulse_us);
    HAL_GPIO_WritePin(SERVO_PORT, SERVO_PIN, GPIO_PIN_RESET);
}

uint8_t Get_Sensor_State(void) {
    return Sensor_Read_Tracking(); 
}

int8_t Calculate_Error(uint8_t state) {
    static int8_t last_error = 0; 
    int8_t error = 0;

    switch (state) {
        case 0x06: error = 0;  break; 
        case 0x04: error = -1; break; 
        case 0x02: error = 1;  break; 
        case 0x0C: error = -2; break; 
        case 0x03: error = 2;  break; 
        case 0x08: error = -4; break; 
        case 0x01: error = 4;  break; 
        case 0x0F: return STOP_FLAG;  
        case 0x00: 
            if (last_error > 0) error = 5; 
            else if (last_error < 0) error = -5;
            else error = 0;
            break;
        default: error = last_error; break;
    }
    if (state != 0x00 && state != 0x0F) last_error = error;
    return error;
}

void Track_Process(PID_t *pid, int16_t Base_Speed) {
    uint8_t sensor_state = Get_Sensor_State(); 
    int8_t error = Calculate_Error(sensor_state); 

    if (error == STOP_FLAG) {
        Motor_SetSpeed(0, 0);
        return; 
    }

    int16_t current_base_speed = Base_Speed;
    if (error >= 4 || error <= -4) current_base_speed = Base_Speed / 2; 

    int16_t Turn_Compensate = (int16_t)PID_Compute(pid, 0, (float)error); 
    Motor_SetSpeed(current_base_speed + Turn_Compensate, current_base_speed - Turn_Compensate); 
}

void Detour_Sequence(void) {
    Motor_SetSpeed(0, 0);
    Servo_SetAngle(0); 
    HAL_Delay(500);
    Motor_SetSpeed(350, -350); 
    HAL_Delay(450); 
    Motor_SetSpeed(300, 300);  
    HAL_Delay(800);
    Motor_SetSpeed(-350, 350); 
    HAL_Delay(900);
    Motor_SetSpeed(300, 300);  

    HAL_Delay(1000);
    Motor_SetSpeed(-350, 350); 
    HAL_Delay(400);
    Servo_SetAngle(90); 
    while(1) {
        Motor_SetSpeed(200, 200); 
        if(Get_Sensor_State() != 0x00) break; 
        HAL_Delay(10);
    }
}
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  DWT_Init(); 
  Motor_Init();
  OLED_Init();
  
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  PID_Init(&trackPID);
  trackPID.Kp = 60.0f;  
  trackPID.Ki = 0.0f;   
  trackPID.Kd = 25.0f;   
  trackPID.OutMax = 400; 
  trackPID.OutMin = -400;
  /* USER CODE END 2 */

  while (1)
  {
      float dist = Get_Distance();

      if (dist > 0 && dist < OBSTACLE_LIMIT) 
      {
          Detour_Sequence(); 
      }
      else 
      {
          Track_Process(&trackPID, 300); 
      }
      HAL_Delay(10); 
  }
}

/* 后面的 SystemClock_Config 和 Error_Handler 保持不变 */

/* USER CODE BEGIN 4 */
/* 这里可以放一些回调函数，如果没有就空着 */
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  * @retval None
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
}