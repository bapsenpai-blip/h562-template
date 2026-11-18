#ifndef BOARD_H
#define BOARD_H

#include "stm32h5xx_hal.h"

/*
 * Định nghĩa phần cứng của board WeAct STM32H562VGT6.
 * !!! Kiểm tra lại với schematic của WeAct trước khi dùng !!!
 */

/* LED trên board — đổi port/pin theo schematic */
#define LED_GPIO_PORT        GPIOE
#define LED_PIN              GPIO_PIN_2
#define LED_GPIO_CLK_ENABLE() __HAL_RCC_GPIOE_CLK_ENABLE()
#define LED_ACTIVE_LOW       0      /* 1 nếu LED sáng khi chân ở mức 0 */

#endif /* BOARD_H */
