#include "main.h"

/* Các handler ngắt lõi Cortex-M33. Thêm handler ngoại vi (USARTx_IRQHandler, ...) tại đây. */

void NMI_Handler(void)
{
    while (1)
    {
    }
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

/* Bắt buộc: HAL_Delay() / HAL_GetTick() dựa vào ngắt này */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

#include "config.h"

/* IMU: SPI1 + GPDMA CH2 (RX) / CH3 (TX) + INT1 trên PA0 */
void SPI1_IRQHandler(void)            { HAL_SPI_IRQHandler(&hspi1); }
void GPDMA1_Channel2_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi1_rx); }
void GPDMA1_Channel3_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_spi1_tx); }
void EXTI0_IRQHandler(void)           { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }

#ifdef USE_MAG
void GPDMA1_Channel0_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_i2c1_rx); }
void I2C1_EV_IRQHandler(void)         { HAL_I2C_EV_IRQHandler(&hi2c1); }
void I2C1_ER_IRQHandler(void)         { HAL_I2C_ER_IRQHandler(&hi2c1); }
#endif

#ifdef USE_GPS
void GPDMA1_Channel1_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart3_rx); }
void USART3_IRQHandler(void)          { HAL_UART_IRQHandler(&huart3); }
#endif
