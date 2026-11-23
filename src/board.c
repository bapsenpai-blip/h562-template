#include "board.h"
#include "main.h"
SPI_HandleTypeDef hspi1;
icm45686_t imu;
static icm45686_config_t cfg;
void GPIOS_Init(void)
{
    static GPIO_InitTypeDef gpiob = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpiob.Pin   = GPIO_PIN_2;
    gpiob.Mode  = GPIO_MODE_OUTPUT_PP;
    gpiob.Pull  = GPIO_NOPULL;
    gpiob.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpiob);

    static GPIO_InitTypeDef gpioa = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpioa.Pin   = GPIO_PIN_2;
    gpioa.Mode  = GPIO_MODE_OUTPUT_PP;
    gpioa.Pull  = GPIO_NOPULL;
    gpioa.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpioa);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

    GPIO_InitTypeDef gpio_it = {0};
    gpio_it.Pin   = GPIO_PIN_0;
    gpio_it.Mode  = GPIO_MODE_IT_RISING;
    gpio_it.Pull  = GPIO_NOPULL;
    gpio_it.Speed = GPIO_SPEED_FREQ_HIGH;   
    HAL_GPIO_Init(GPIOA, &gpio_it);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
void SPI1_Init(void){
    __HAL_RCC_SPI1_CLK_ENABLE();
    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    /* Giữ SCK/MOSI ổn định khi HAL tắt/bật SPI giữa các lần truyền lúc CS đang thấp */
    hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}
int ICM45686_Init(void)
{
    icm45686_default_config(&cfg);
    cfg.hspi    = &hspi1;
    cfg.cs_port = GPIOA;
    cfg.cs_pin  = GPIO_PIN_2;
    cfg.gyro_odr  = GYRO_CONFIG0_GYRO_ODR_1600_HZ;
    cfg.gyro_fsr  = GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS;   /* tên enum: kiểm tra trong inv_imu_defs.h */
    cfg.accel_fsr = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G;
    cfg.spi_prescaler_init = SPI_BAUDRATEPRESCALER_256;    /* xem mục C */
    cfg.spi_prescaler_run  = SPI_BAUDRATEPRESCALER_16;
    return icm45686_init(&imu, &cfg);
}

#ifdef USE_MAG
I2C_HandleTypeDef hi2c1;
void I2C1_Init(void)
{
    hi2c1.Instance              = I2C1;
    hi2c1.Init.Timing           = 0xF0310F15;   /* ~400 kHz @ I2C kernel clk 250 MHz (PCLK1); kiểm tra lại bằng CubeMX/scope */
    hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
    HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
}
#endif

#ifdef USE_GPS
UART_HandleTypeDef huart3;
void USART3_Init(void)
{
    huart3.Instance            = USART3;
    huart3.Init.BaudRate       = 9600;          /* GPS_Init tự dò baud rồi chuyển sang GPS_BAUD */
    huart3.Init.WordLength     = UART_WORDLENGTH_8B;
    huart3.Init.StopBits       = UART_STOPBITS_1;
    huart3.Init.Parity         = UART_PARITY_NONE;
    huart3.Init.Mode           = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling   = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
}
#endif

TIM_HandleTypeDef htim5;
void TIMER5_Init(void){
    /* TIM5 32-bit free-running, 250MHz / 250 = 1MHz (1 tick = 1us) */
    __HAL_RCC_TIM5_CLK_ENABLE();
    htim5.Instance         = TIM5;
    htim5.Init.Prescaler   = 249;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period      = 0xFFFFFFFF;
    if(HAL_TIM_Base_Init(&htim5) != HAL_OK) Error_Handler();
    HAL_TIM_Base_Start(&htim5);
}
