#include "main.h"

/*
 * SYSCLK = 250 MHz (tối đa của STM32H5), nguồn HSI 64 MHz, không phụ thuộc thạch anh.
 *
 *   HSI 64 MHz / PLLM 16 = 4 MHz   (VCI range 2: 4..8 MHz)
 *   4 MHz * PLLN 125     = 500 MHz (VCO wide: 192..836 MHz)
 *   500 / PLLP 2         = 250 MHz -> SYSCLK
 *
 * Khi muốn dùng HSE (chính xác hơn, cần cho USB/CAN), đổi nguồn PLL sang HSE
 * và tính lại PLLM. Nên đối chiếu với Clock Configuration trong STM32CubeMX.
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    /* VOS0 bắt buộc để chạy 250 MHz */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
    {
    }

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSIDiv              = RCC_HSI_DIV1;           /* 64 MHz */
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

    osc.PLL.PLLState  = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLL1_SOURCE_HSI;
    osc.PLL.PLLM      = 16;
    osc.PLL.PLLN      = 125;
    osc.PLL.PLLP      = 2;
    osc.PLL.PLLQ      = 2;
    osc.PLL.PLLR      = 2;
    osc.PLL.PLLRGE    = RCC_PLL1_VCIRANGE_2;
    osc.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
    osc.PLL.PLLFRACN  = 0;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        Error_Handler();
    }

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                         RCC_CLOCKTYPE_PCLK3;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    clk.APB3CLKDivider = RCC_HCLK_DIV1;

    /* 250 MHz @ VOS0 cần 5 wait state */
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}
