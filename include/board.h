#ifndef BOARD_H
#define BOARD_H

#include "stm32h5xx_hal.h"
#include "icm45686_fc.h"
#include "config.h"

void GPIOS_Init(void); void SPI1_Init(void); void TIMER5_Init(void); int ICM45686_Init(void);
extern SPI_HandleTypeDef hspi1; extern DMA_HandleTypeDef hdma_spi1_rx, hdma_spi1_tx; extern icm45686_t imu;
#ifdef USE_MAG
void I2C1_Init(void);
extern I2C_HandleTypeDef hi2c1; extern DMA_HandleTypeDef hdma_i2c1_rx;
#endif
#ifdef USE_GPS
void USART3_Init(void);
extern UART_HandleTypeDef huart3; extern DMA_HandleTypeDef hdma_usart3_rx;
#endif
#endif /* BOARD_H */
