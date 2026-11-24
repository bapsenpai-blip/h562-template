#ifndef BOARD_H
#define BOARD_H

#include "stm32h5xx_hal.h"
#include "icm45686_fc.h"
#include "bmp388.h"
#include "mavlink_send.h"
#include "config.h"

typedef struct {
    int rc;
    uint8_t whoami;
} SensorCheck_t;

void GPIOS_Init(void); void SPI1_Init(void); void TIMER5_Init(void); int ICM45686_Init(void); SensorCheck_t ICM45686_Check(void);
extern SPI_HandleTypeDef hspi1; extern DMA_HandleTypeDef hdma_spi1_rx, hdma_spi1_tx; extern icm45686_t imu;

void I2C1_Init(void); int BMP388_Init(void); SensorCheck_t BMP388_Check(void);
extern I2C_HandleTypeDef hi2c1; extern DMA_HandleTypeDef hdma_i2c1_rx; extern bmp388_t bmp;

#ifdef USE_GPS
void USART3_Init(void);
extern UART_HandleTypeDef huart3; extern DMA_HandleTypeDef hdma_usart3_rx;
#endif

void USART1_Init(void); int MAVLINK_Init(void);
extern UART_HandleTypeDef huart1; extern DMA_HandleTypeDef hdma_usart1_tx, hdma_usart1_rx; extern mavlink_tx_t mav;
#endif /* BOARD_H */
