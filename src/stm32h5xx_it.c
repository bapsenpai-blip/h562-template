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
