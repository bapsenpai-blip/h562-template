/*
 * icm45686_port_stm32.c
 *
 * Port STM32 HAL cho driver eMD ICM-45686 của TDK.
 */
#include "icm45686_port_stm32.h"

/* ---------------------------------------------------------------------------
 * Thời gian (DWT)
 * ------------------------------------------------------------------------- */
static uint32_t s_cycles_per_us = 1;

void icm_port_timer_init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(__CORTEX_M) && (__CORTEX_M == 7U)
	/* Một số chip M7 (F7/H7) khóa DWT sau reset: mở khóa bằng LAR. */
	DWT->LAR = 0xC5ACCE55;
#endif
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	s_cycles_per_us = SystemCoreClock / 1000000U;
	if (s_cycles_per_us == 0)
		s_cycles_per_us = 1;
}

uint32_t icm_port_micros(void)
{
	/* CYCCNT tràn mỗi 2^32 chu kỳ (~8.9 s @480 MHz). Ta tích lũy phần tràn
	 * để micros() đơn điệu, miễn là hàm được gọi ít nhất một lần mỗi vài giây
	 * (vòng PID của FC gọi liên tục nên luôn thỏa). */
	static uint32_t last_cyc  = 0;
	static uint32_t us_accum  = 0;
	static uint32_t cyc_rem   = 0;

	uint32_t primask = __get_PRIMASK();
	__disable_irq();

	uint32_t now   = DWT->CYCCNT;
	uint32_t delta = now - last_cyc; /* tự xử lý tràn */
	last_cyc       = now;

	cyc_rem += delta;
	us_accum += cyc_rem / s_cycles_per_us;
	cyc_rem %= s_cycles_per_us;

	uint32_t result = us_accum;
	if (!primask)
		__enable_irq();

	return result;
}

void icm_port_sleep_us(uint32_t us)
{
	/* Chia nhỏ để không vượt nửa vòng CYCCNT. */
	while (us) {
		uint32_t chunk = (us > 1000000U) ? 1000000U : us;
		uint32_t start = DWT->CYCCNT;
		uint32_t ticks = chunk * s_cycles_per_us;
		while ((DWT->CYCCNT - start) < ticks) {
		}
		us -= chunk;
	}
}

/* ---------------------------------------------------------------------------
 * SPI
 * ------------------------------------------------------------------------- */
int icm_port_spi_read_reg(void *context, uint8_t reg, uint8_t *buf, uint32_t len)
{
	const icm_port_bus_t *b    = (const icm_port_bus_t *)context;
	uint8_t               addr = (uint8_t)(reg | ICM_SPI_READ_BIT);
	HAL_StatusTypeDef     st;

	if (b == NULL || buf == NULL || len == 0 || len > 0xFFFFU)
		return -1;

	icm_port_cs_low(b);
	st = HAL_SPI_Transmit(b->hspi, &addr, 1, b->timeout_ms);
	if (st == HAL_OK)
		st = HAL_SPI_Receive(b->hspi, buf, (uint16_t)len, b->timeout_ms);
	icm_port_cs_high(b);

	return (st == HAL_OK) ? 0 : -1;
}

int icm_port_spi_write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len)
{
	const icm_port_bus_t *b    = (const icm_port_bus_t *)context;
	uint8_t               addr = (uint8_t)(reg & (uint8_t)~ICM_SPI_READ_BIT);
	HAL_StatusTypeDef     st;

	if (b == NULL || buf == NULL || len == 0 || len > 0xFFFFU)
		return -1;

	icm_port_cs_low(b);
	st = HAL_SPI_Transmit(b->hspi, &addr, 1, b->timeout_ms);
	if (st == HAL_OK)
		st = HAL_SPI_Transmit(b->hspi, (uint8_t *)buf, (uint16_t)len, b->timeout_ms);
	icm_port_cs_high(b);

	return (st == HAL_OK) ? 0 : -1;
}

int icm_port_spi_set_prescaler(SPI_HandleTypeDef *hspi, uint32_t prescaler)
{
	if (hspi->Init.BaudRatePrescaler == prescaler)
		return 0;

	hspi->Init.BaudRatePrescaler = prescaler;
	/* HAL_SPI_Init chỉ gọi MspInit khi State == RESET, nên gọi lại ở đây
	 * chỉ ghi lại thanh ghi cấu hình, không đụng tới GPIO/DMA. */
	return (HAL_SPI_Init(hspi) == HAL_OK) ? 0 : -1;
}
