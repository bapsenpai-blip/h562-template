#include "main.h"
TIM_HandleTypeDef htim = {0};
GPIO_InitTypeDef gpio = {0};
static void LED_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin   = GPIO_PIN_2;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
}
static void TIMER_Init(void)
{
    TIM_PWM_InitTypeDef sConfig = {0};
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim.Instance = TIM2;
    htim.Init.Prescaler = 25000 - 1; // 1 kHz
    htim.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim.Init.Period = 1000 - 1; // 1 second
    htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim);
    HAL_TIM_Base_Start_IT(&htim);
    
}

int main(void)
{
    HAL_Init();             /* SysTick 1 ms, NVIC priority grouping */
    SystemClock_Config();   /* 250 MHz từ HSI + PLL1 */
    LED_Init();
    TIMER_Init();
    led_brightness = 50; // Set initial brightness to 50%
    while (1)   
    {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        HAL_Delay(500);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
