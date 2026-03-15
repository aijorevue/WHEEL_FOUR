#include "sensors.h"
#include "main.h"  




uint8_t Sensor_Read_Tracking(void)
{
    uint8_t status = 0;
   
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)  status |= (1 << 3); // L2
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET)  status |= (1 << 2); // L1
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_SET) status |= (1 << 1); // R1
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET) status |= (1 << 0); // R2
    return status;
}