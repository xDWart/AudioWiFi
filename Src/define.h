#include "stm32f1xx_hal.h"
#include "string.h"
#include "math.h"
#include "stdbool.h"
#include "stdio.h"

#define nop() __no_operation();

#define Led_off HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET)
#define Led_on HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET)
#define Led_inv HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8)