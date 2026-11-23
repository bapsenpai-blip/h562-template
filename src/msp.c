#include "stm32h5xx_hal.h"
#include "main.h"

#include "config.h"

#ifdef USE_MAG
extern DMA_HandleTypeDef hdma_i2c1_rx;
#endif
#ifdef USE_GPS
extern DMA_HandleTypeDef hdma_usart3_rx;
#endif

/* Cấu hình chung cho kênh GPDMA ngoại vi -> RAM, byte, normal mode */
static void dma_rx_init(DMA_HandleTypeDef *h, DMA_Channel_TypeDef *ch, uint32_t request, IRQn_Type irq)
{
    __HAL_RCC_GPDMA1_CLK_ENABLE();
    h->Instance                   = ch;
    h->Init.Request               = request;
    h->Init.BlkHWRequest          = DMA_BREQ_SINGLE_BURST;
    h->Init.Direction             = DMA_PERIPH_TO_MEMORY;
    h->Init.SrcInc                = DMA_SINC_FIXED;
    h->Init.DestInc               = DMA_DINC_INCREMENTED;
    h->Init.SrcDataWidth          = DMA_SRC_DATAWIDTH_BYTE;
    h->Init.DestDataWidth         = DMA_DEST_DATAWIDTH_BYTE;
    h->Init.Priority              = DMA_LOW_PRIORITY_LOW_WEIGHT;
    h->Init.SrcBurstLength        = 1;
    h->Init.DestBurstLength       = 1;
    h->Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
    h->Init.TransferEventMode     = DMA_TCEM_BLOCK_TRANSFER;
    h->Init.Mode                  = DMA_NORMAL;
    if (HAL_DMA_Init(h) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(irq, 5, 0);
    HAL_NVIC_EnableIRQ(irq);
}
static void dma_tx_init(DMA_HandleTypeDef *h, DMA_Channel_TypeDef *ch, uint32_t request, IRQn_Type irq)
{
    __HAL_RCC_GPDMA1_CLK_ENABLE();
    h->Instance                   = ch;
    h->Init.Request               = request;
    h->Init.BlkHWRequest          = DMA_BREQ_SINGLE_BURST;
    h->Init.Direction             = DMA_MEMORY_TO_PERIPH;
    h->Init.SrcInc                = DMA_SINC_INCREMENTED;
    h->Init.DestInc               = DMA_DINC_FIXED;
    h->Init.SrcDataWidth          = DMA_SRC_DATAWIDTH_BYTE;
    h->Init.DestDataWidth         = DMA_DEST_DATAWIDTH_BYTE;
    h->Init.Priority              = DMA_LOW_PRIORITY_LOW_WEIGHT;
    h->Init.SrcBurstLength        = 1;
    h->Init.DestBurstLength       = 1;
    h->Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT1 | DMA_DEST_ALLOCATED_PORT0;
    h->Init.TransferEventMode     = DMA_TCEM_BLOCK_TRANSFER;
    h->Init.Mode                  = DMA_NORMAL;
    if (HAL_DMA_Init(h) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(irq, 5, 0);
    HAL_NVIC_EnableIRQ(irq);
}
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hspi->Instance==SPI1)
    {
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**SPI1 GPIO Configuration
        PA5     ------> SPI1_SCK
        PA6     ------> SPI1_MISO
        PA7     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        HAL_NVIC_SetPriority(SPI1_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);    

        dma_rx_init(&hdma_spi1_rx, GPDMA1_Channel2, GPDMA1_REQUEST_SPI1_RX, GPDMA1_Channel2_IRQn);
        __HAL_LINKDMA(hspi, hdmarx, hdma_spi1_rx);
        dma_tx_init(&hdma_spi1_tx, GPDMA1_Channel3, GPDMA1_REQUEST_SPI1_TX, GPDMA1_Channel3_IRQn);
        __HAL_LINKDMA(hspi, hdmatx, hdma_spi1_tx);
    }   
}
#ifdef USE_MAG
DMA_HandleTypeDef hdma_i2c1_rx;

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) return;

    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    /* PB6 SCL, PB7 SDA */
    g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);

    dma_rx_init(&hdma_i2c1_rx, GPDMA1_Channel0, GPDMA1_REQUEST_I2C1_RX, GPDMA1_Channel0_IRQn);
    __HAL_LINKDMA(hi2c, hdmarx, hdma_i2c1_rx);

    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
}
#endif

#ifdef USE_GPS
DMA_HandleTypeDef hdma_usart3_rx;

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) return;

    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART3_CLK_ENABLE();
    /* PB10 TX, PB11 RX */
    g.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &g);

    dma_rx_init(&hdma_usart3_rx, GPDMA1_Channel1, GPDMA1_REQUEST_USART3_RX, GPDMA1_Channel1_IRQn);
    __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);

    HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}
#endif
