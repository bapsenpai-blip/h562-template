#ifndef MAIN_H
#define MAIN_H

#include "stm32h5xx_hal.h"
#include "board.h"
#include "mavlink_send.h"

#include "biquadfilter.h"
#include "mahony.h"
#include "quaternion.h"

#include "config.h"

#include "qmc5883l.h"
#include "ubx_gps.h"
#include "bmp388.h"

void SystemClock_Config(void);
void Error_Handler(void);

#endif /* MAIN_H */
