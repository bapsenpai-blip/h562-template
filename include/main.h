#ifndef MAIN_H
#define MAIN_H

#include "stm32h5xx_hal.h"
#include "board.h"

/* Không include driver ở đây: include "icm45686_fc.h" trong file .c nào dùng IMU. */

void SystemClock_Config(void);
void Error_Handler(void);

#endif /* MAIN_H */
