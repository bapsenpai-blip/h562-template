/*
 * icm45686_fc.c
 *
 * Lớp driver ICM-45686 cho flight controller, xây trên driver eMD của TDK.
 */
#include "icm45686_fc.h"

#include <string.h>

/* Địa chỉ thanh ghi dùng ở đường nóng (khớp inv_imu_regmap_le.h) */
#define REG_ACCEL_DATA_X1_UI 0x00u
#define REG_INT1_STATUS0     0x19u

#define BURST_DATA_LEN   14u /* accel 6 + gyro 6 + temp 2 */
#define BURST_STATUS_LEN 26u /* 0x00 .. 0x19 */

/* ---------------------------------------------------------------------------
 * Bảng tra
 * ------------------------------------------------------------------------- */
static float gyro_fsr_dps(gyro_config0_gyro_ui_fs_sel_t fsr)
{
	switch (fsr) {
#if INV_IMU_HIGH_FSR_SUPPORTED
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_4000_DPS: return 4000.0f;
#endif
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS: return 2000.0f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_1000_DPS: return 1000.0f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_500_DPS: return 500.0f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_250_DPS: return 250.0f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_125_DPS: return 125.0f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_62_5_DPS: return 62.5f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_31_25_DPS: return 31.25f;
	case GYRO_CONFIG0_GYRO_UI_FS_SEL_15_625_DPS: return 15.625f;
	default: return 0.0f;
	}
}

static float accel_fsr_g(accel_config0_accel_ui_fs_sel_t fsr)
{
	switch (fsr) {
#if INV_IMU_HIGH_FSR_SUPPORTED
	case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_32_G: return 32.0f;
#endif
	case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G: return 16.0f;
	case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_8_G: return 8.0f;
	case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G: return 4.0f;
	case ACCEL_CONFIG0_ACCEL_UI_FS_SEL_2_G: return 2.0f;
	default: return 0.0f;
	}
}

static uint32_t gyro_odr_hz(gyro_config0_gyro_odr_t odr)
{
	switch (odr) {
	case GYRO_CONFIG0_GYRO_ODR_6400_HZ: return 6400;
	case GYRO_CONFIG0_GYRO_ODR_3200_HZ: return 3200;
	case GYRO_CONFIG0_GYRO_ODR_1600_HZ: return 1600;
	case GYRO_CONFIG0_GYRO_ODR_800_HZ: return 800;
	case GYRO_CONFIG0_GYRO_ODR_400_HZ: return 400;
	case GYRO_CONFIG0_GYRO_ODR_200_HZ: return 200;
	case GYRO_CONFIG0_GYRO_ODR_100_HZ: return 100;
	case GYRO_CONFIG0_GYRO_ODR_50_HZ: return 50;
	case GYRO_CONFIG0_GYRO_ODR_25_HZ: return 25;
	case GYRO_CONFIG0_GYRO_ODR_12_5_HZ: return 12;
	case GYRO_CONFIG0_GYRO_ODR_6_25_HZ: return 6;
	case GYRO_CONFIG0_GYRO_ODR_3_125_HZ: return 3;
	case GYRO_CONFIG0_GYRO_ODR_1_5625_HZ: return 1;
	default: return 0;
	}
}

/* ---------------------------------------------------------------------------
 * Xử lý mẫu
 * ------------------------------------------------------------------------- */
static inline int16_t neg16(int16_t v)
{
	return (v == INT16_MIN) ? INT16_MAX : (int16_t)-v;
}

/* Xoay vector cảm biến sang hệ thân máy bay. */
static void apply_align(icm45686_align_t a, const int16_t in[3], int16_t out[3])
{
	const int16_t x = in[0], y = in[1], z = in[2];

	switch (a) {
	default:
	case ICM45686_ALIGN_CW0:       out[0] = x;        out[1] = y;        out[2] = z;        break;
	case ICM45686_ALIGN_CW90:      out[0] = y;        out[1] = neg16(x); out[2] = z;        break;
	case ICM45686_ALIGN_CW180:     out[0] = neg16(x); out[1] = neg16(y); out[2] = z;        break;
	case ICM45686_ALIGN_CW270:     out[0] = neg16(y); out[1] = x;        out[2] = z;        break;
	case ICM45686_ALIGN_CW0_FLIP:  out[0] = neg16(x); out[1] = y;        out[2] = neg16(z); break;
	case ICM45686_ALIGN_CW90_FLIP: out[0] = y;        out[1] = x;        out[2] = neg16(z); break;
	case ICM45686_ALIGN_CW180_FLIP:out[0] = x;        out[1] = neg16(y); out[2] = neg16(z); break;
	case ICM45686_ALIGN_CW270_FLIP:out[0] = neg16(y); out[1] = neg16(x); out[2] = neg16(z); break;
	}
}

static inline bool is_invalid(const int16_t v[3])
{
	return (v[0] == INVALID_VALUE_FIFO) || (v[1] == INVALID_VALUE_FIFO) ||
	       (v[2] == INVALID_VALUE_FIFO);
}

/* Điền icm45686_sample_t từ dữ liệu thô (đã đúng endian). Trả về false nếu
 * mẫu không hợp lệ. */
static bool fill_sample(icm45686_t *imu, const inv_imu_sensor_data_t *d, uint32_t ts,
                        icm45686_sample_t *s)
{
	if (is_invalid(d->gyro_data) || is_invalid(d->accel_data)) {
		imu->stat_invalid++;
		return false;
	}

	apply_align(imu->cfg.align, d->gyro_data, s->gyro_raw);
	apply_align(imu->cfg.align, d->accel_data, s->accel_raw);
	s->temp_raw = d->temp_data;

	for (int i = 0; i < 3; i++) {
		s->gyro_dps[i] = (float)s->gyro_raw[i] * imu->gyro_dps_per_lsb - imu->gyro_bias_dps[i];
		s->accel_g[i]  = (float)s->accel_raw[i] * imu->accel_g_per_lsb;
	}
	/* Theo ví dụ TDK: hệ số 128 LSB/°C, offset 25 °C */
	s->temp_c       = 25.0f + (float)d->temp_data / 128.0f;
	s->timestamp_us = ts;
	s->dt_us        = 0;
	return true;
}

/* Parse burst DMA: rx[0] là byte rác lúc gửi địa chỉ, dữ liệu từ rx[1]. */
static void parse_burst(const icm45686_t *imu, const uint8_t *p, inv_imu_sensor_data_t *d)
{
	const uint8_t be = imu->dev.endianness_data;
	uint16_t *    o  = (uint16_t *)d;

	for (int i = 0; i < 7; i++)
		FORMAT_16_BITS_DATA(be, &p[2 * i], &o[i]);
}

/* ---------------------------------------------------------------------------
 * Cache (Cortex-M7)
 * ------------------------------------------------------------------------- */
static inline void cache_clean(void *addr, int32_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
	if (SCB->CCR & SCB_CCR_DC_Msk)
		SCB_CleanDCache_by_Addr((uint32_t *)addr, len);
#else
	(void)addr;
	(void)len;
#endif
}

static inline void cache_invalidate(void *addr, int32_t len)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
	if (SCB->CCR & SCB_CCR_DC_Msk)
		SCB_InvalidateDCache_by_Addr((uint32_t *)addr, len);
#else
	(void)addr;
	(void)len;
#endif
}

/* ---------------------------------------------------------------------------
 * API
 * ------------------------------------------------------------------------- */
void icm45686_default_config(icm45686_config_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->spi_prescaler_init = SPI_BAUDRATEPRESCALER_128;
	cfg->spi_prescaler_run  = SPI_BAUDRATEPRESCALER_8;

	cfg->gyro_odr = GYRO_CONFIG0_GYRO_ODR_6400_HZ;
	cfg->gyro_fsr = GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS;
	cfg->gyro_lpf = IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_8;

	cfg->accel_odr = ACCEL_CONFIG0_ACCEL_ODR_1600_HZ;
	cfg->accel_fsr = ACCEL_CONFIG0_ACCEL_UI_FS_SEL_16_G;
	cfg->accel_lpf = IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4;

	cfg->align           = ICM45686_ALIGN_CW0;
	cfg->read_int_status = false;
}

int icm45686_init(icm45686_t *imu, const icm45686_config_t *cfg)
{
	int                      rc     = 0;
	uint8_t                  whoami = 0;
	inv_imu_int_pin_config_t pin;
	inv_imu_fifo_config_t fifo;
	inv_imu_int_state_t      it;

	if (imu == NULL || cfg == NULL || cfg->hspi == NULL || cfg->cs_port == NULL)
		return ICM45686_ERR_ARG;

	memset(imu, 0, sizeof(*imu));
	imu->cfg = *cfg;

	imu->gyro_dps_per_lsb = gyro_fsr_dps(cfg->gyro_fsr) / 32768.0f;
	imu->accel_g_per_lsb  = accel_fsr_g(cfg->accel_fsr) / 32768.0f;
	imu->gyro_odr_hz      = gyro_odr_hz(cfg->gyro_odr);
	if (imu->gyro_dps_per_lsb == 0.0f || imu->accel_g_per_lsb == 0.0f || imu->gyro_odr_hz == 0)
		return ICM45686_ERR_ARG;

	/* Bus */
	imu->bus.hspi       = cfg->hspi;
	imu->bus.cs_port    = cfg->cs_port;
	imu->bus.cs_pin     = cfg->cs_pin;
	imu->bus.timeout_ms = 10;
	icm_port_cs_high(&imu->bus);

	icm_port_timer_init();
	if (icm_port_spi_set_prescaler(cfg->hspi, cfg->spi_prescaler_init) != 0)
		return INV_IMU_ERROR_TRANSPORT;

	/* Transport TDK */
	imu->dev.transport.context    = &imu->bus;
	imu->dev.transport.read_reg   = icm_port_spi_read_reg;
	imu->dev.transport.write_reg  = icm_port_spi_write_reg;
	imu->dev.transport.serif_type = UI_SPI4;
	imu->dev.transport.sleep_us   = icm_port_sleep_us;

	/* Chờ nguồn ổn định (theo ví dụ TDK: 3 ms) */
	icm_port_sleep_us(3000);

	/* Giao dịch SPI đầu tiên chốt chip vào chế độ SPI; thử lại vài lần */
	for (int i = 0; i < 5; i++) {
		rc = inv_imu_get_who_am_i(&imu->dev, &whoami);
		if (rc == 0 && whoami == INV_IMU_WHOAMI)
			break;
		icm_port_sleep_us(1000);
	}
	if (rc != 0)
		return rc;
	if (whoami != INV_IMU_WHOAMI)
		return ICM45686_ERR_WHOAMI;

	rc = inv_imu_soft_reset(&imu->dev); /* cũng đọc endianness */
	if (rc != 0)
		return rc;

	/* INT1: active high, pulse, push-pull -> EXTI cạnh lên */
	pin.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
	pin.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
	pin.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
	rc |= inv_imu_set_pin_config_int(&imu->dev, INV_IMU_INT1, &pin);

	memset(&it, INV_IMU_DISABLE, sizeof(it));
	it.INV_FIFO_THS = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu->dev, INV_IMU_INT1, &it);
		memset(&fifo, 0, sizeof(fifo));
	fifo.gyro_en    = INV_IMU_ENABLE;
	fifo.accel_en   = INV_IMU_ENABLE;
	fifo.hires_en   = INV_IMU_DISABLE;                 /* 16-bit, frame 16 byte */
	fifo.fifo_wm_th = 4;                               /* 4 frame = ~2.5 ms @1600 Hz */
	fifo.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_STREAM;
	fifo.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_MAX;
	
	rc |= inv_imu_set_fifo_config(&imu->dev, &fifo);
	rc |= inv_imu_flush_fifo(&imu->dev);
	
	rc |= inv_imu_set_gyro_fsr(&imu->dev, cfg->gyro_fsr);
	rc |= inv_imu_set_accel_fsr(&imu->dev, cfg->accel_fsr);
	rc |= inv_imu_set_gyro_frequency(&imu->dev, cfg->gyro_odr);
	rc |= inv_imu_set_accel_frequency(&imu->dev, cfg->accel_odr);
	rc |= inv_imu_set_gyro_ln_bw(&imu->dev, cfg->gyro_lpf);
	rc |= inv_imu_set_accel_ln_bw(&imu->dev, cfg->accel_lpf);

	/* Low-noise mode cho cả hai cảm biến */
	rc |= inv_imu_set_accel_mode(&imu->dev, PWR_MGMT0_ACCEL_MODE_LN);
	rc |= inv_imu_set_gyro_mode(&imu->dev, PWR_MGMT0_GYRO_MODE_LN);
	if (rc != 0)
		return rc;

	/* Gyro cần GYR_STARTUP_TIME_US (70 ms) để ổn định */
	icm_port_sleep_us(GYR_STARTUP_TIME_US);
	imu->discard = (uint16_t)(imu->gyro_odr_hz / 100U + 1U); /* bỏ ~10 ms đầu */

	if (icm_port_spi_set_prescaler(cfg->hspi, cfg->spi_prescaler_run) != 0)
		return INV_IMU_ERROR_TRANSPORT;

	return ICM45686_OK;
}

int icm45686_read_blocking(icm45686_t *imu, icm45686_sample_t *out)
{
	inv_imu_sensor_data_t d;
	int                   rc;

	if (imu->streaming || imu->dma_busy)
		return ICM45686_ERR_BUSY;

	rc = inv_imu_get_register_data(&imu->dev, &d);
	if (rc != 0)
		return rc;

	return fill_sample(imu, &d, icm_port_micros(), out) ? ICM45686_OK : ICM45686_ERR_NO_DATA;
}

int icm45686_calibrate_gyro(icm45686_t *imu, uint32_t samples, float max_spread_dps)
{
	float    sum[3] = { 0 }, mn[3], mx[3];
	uint32_t got = 0, tries = 0;
	uint32_t period_us = 1000000U / imu->gyro_odr_hz;

	if (samples == 0)
		return ICM45686_ERR_ARG;

	/* Đo trên dữ liệu chưa trừ bias */
	float saved[3];
	memcpy(saved, imu->gyro_bias_dps, sizeof(saved));
	memset(imu->gyro_bias_dps, 0, sizeof(imu->gyro_bias_dps));

	for (int i = 0; i < 3; i++) {
		mn[i] = 1e9f;
		mx[i] = -1e9f;
	}

	/* Bỏ 100 mẫu đầu: gyro vừa bật/cấu hình lại còn dao động */
	for (int i = 0; i < 100; i++) {
		icm45686_sample_t w;
		icm_port_sleep_us(period_us);
		(void)icm45686_read_blocking(imu, &w);
	}

	while (got < samples && tries < samples * 3U) {
		icm45686_sample_t s;
		tries++;
		icm_port_sleep_us(period_us);
		if (icm45686_read_blocking(imu, &s) != ICM45686_OK)
			continue;
		for (int i = 0; i < 3; i++) {
			sum[i] += s.gyro_dps[i];
			if (s.gyro_dps[i] < mn[i]) mn[i] = s.gyro_dps[i];
			if (s.gyro_dps[i] > mx[i]) mx[i] = s.gyro_dps[i];
		}
		got++;
	}

	if (got < samples) {
		memcpy(imu->gyro_bias_dps, saved, sizeof(saved));
		return ICM45686_ERR_NO_DATA;
	}

	for (int i = 0; i < 3; i++)
		imu->calib_spread_dps[i] = mx[i] - mn[i];

	for (int i = 0; i < 3; i++) {
		if ((mx[i] - mn[i]) > max_spread_dps) {
			memcpy(imu->gyro_bias_dps, saved, sizeof(saved));
			return ICM45686_ERR_MOVING;
		}
	}

	for (int i = 0; i < 3; i++)
		imu->gyro_bias_dps[i] = sum[i] / (float)got;

	return ICM45686_OK;
}

int icm45686_start(icm45686_t *imu)
{
	if (imu->streaming)
		return ICM45686_OK;
	inv_imu_flush_fifo(&imu->dev); /* bỏ frame cũ tích lại lúc calib (STREAM ghi đè) */
	imu->have_ts       = false;
	imu->fifo_state    = FIFO_IDLE;
	imu->dma_busy      = false;
	imu->last_read_seq = imu->seq;
	imu->streaming     = true;
	return ICM45686_OK;
}

void icm45686_stop(icm45686_t *imu)
{
	imu->streaming = false;
	/* Chờ DMA đang chạy (nếu có) xong, tối đa ~1 ms */
	uint32_t t0 = icm_port_micros();
	while (imu->dma_busy && (icm_port_micros() - t0) < 1000U) {
	}
	if (imu->dma_busy) {
		HAL_SPI_Abort(imu->bus.hspi);
		icm_port_cs_high(&imu->bus);
		imu->dma_busy = false;
	}
}
static void start_dma(icm45686_t *imu, uint16_t len)
{
        icm_port_cs_low(&imu->bus);
        if (HAL_SPI_TransmitReceive_DMA(imu->bus.hspi, imu->dma_tx, imu->dma_rx, len) != HAL_OK) {
                icm_port_cs_high(&imu->bus);
                imu->dma_busy   = false;
                imu->fifo_state = FIFO_IDLE;
                imu->stat_dma_error++;
        }
}
void icm45686_on_drdy(icm45686_t *imu)
{
	if (!imu->streaming)
		return;

	if (imu->fifo_state != FIFO_IDLE || imu->dma_busy) {
		imu->stat_overrun++;
		return;
	}

	imu->drdy_ts_us = icm_port_micros();
	imu->dma_busy   = true;
	imu->fifo_state = FIFO_READ_COUNT;

	imu->dma_tx[0] = REG_FIFO_COUNT_0 | ICM_SPI_READ_BIT;
	imu->dma_tx[1] = 0;
	imu->dma_tx[2] = 0;
	start_dma(imu, 3);
}

void icm45686_on_dma_complete(icm45686_t *imu)
{
	icm_port_cs_high(&imu->bus);

	if (imu->fifo_state == FIFO_READ_COUNT) {
		/* Errata AN-000364: đọc FIFO_COUNT hai lần, dùng lần hai */
		imu->fifo_state = FIFO_READ_COUNT2;
		start_dma(imu, 3); /* dma_tx vẫn giữ nguyên lệnh đọc 0x12 */
		return;
	}

	if (imu->fifo_state == FIFO_READ_COUNT2) {
		uint16_t n;
		FORMAT_16_BITS_DATA(imu->dev.endianness_data, &imu->dma_rx[1], &n); /* số frame */
		if (n == 0) {
			imu->dma_busy   = false;
			imu->fifo_state = FIFO_IDLE;
			return;
		}
		if (n > FIFO_MAX_FRAMES) {
			imu->stat_fifo_overflow++;
			n = FIFO_MAX_FRAMES; /* phần còn lại đọc ở ngắt sau (watermark vẫn còn vượt) */
		}

		imu->fifo_n     = n;
		imu->fifo_state = FIFO_READ_DATA;
		imu->dma_tx[0]  = FIFO_DATA | ICM_SPI_READ_BIT;
		memset(&imu->dma_tx[1], 0, n * FIFO_FRAME_SIZE);
		start_dma(imu, (uint16_t)(1U + n * FIFO_FRAME_SIZE));
		return;
	}

	/* FIFO_READ_DATA: parse từng frame */
	cache_invalidate(imu->dma_rx, sizeof(imu->dma_rx));
	for (uint16_t i = 0; i < imu->fifo_n; i++) {
		const uint8_t *f = &imu->dma_rx[1 + i * FIFO_FRAME_SIZE];
		inv_imu_sensor_data_t d;

		/* header hợp lệ: ACCEL_EN | GYRO_EN | TMST_FIELD_EN (0x68), không EXT_HEADER/HIRES */
		if ((f[0] & 0xF8U) != 0x68U) {
			imu->stat_invalid++;
			continue;
		}
		parse_burst(imu, &f[1], &d);          /* chỉ dùng accel+gyro (6 từ đầu) */
		d.temp_data = (int8_t)f[13];          /* temp trong FIFO chỉ 1 byte */

		/* Timestamp 16 bit (1 µs/tick, tràn ~65 ms): dt = hiệu hai frame liên tiếp.
		 * Phải cập nhật cả với frame bị bỏ để hiệu không bị cộng dồn. */
		uint16_t ts;
		FORMAT_16_BITS_DATA(imu->dev.endianness_data, &f[14], &ts);
		uint16_t dt = imu->have_ts ? (uint16_t)(ts - imu->last_fifo_ts) : 0U;
		imu->last_fifo_ts = ts;
		imu->have_ts      = true;

		if (imu->discard) { imu->discard--; continue; }
		imu->stat_frames++;

		uint8_t next = imu->latest ^ 1U;
		if (fill_sample(imu, &d, imu->drdy_ts_us, &imu->buf[next])) {
			imu->buf[next].seq = imu->seq + 1U;
			imu->buf[next].dt_us = dt;
			/* temp trong FIFO: 2 LSB/°C (datasheet 17.14), khác thanh ghi 128 LSB/°C */
			imu->buf[next].temp_c = 25.0f + (float)d.temp_data / 2.0f;
			imu->latest = next;
			__DMB();
			uint32_t h = imu->ring_head;
			if(h - imu->ring_tail >= IMU_RING_SIZE) imu->stat_ring_drop++;
			else {
				imu->ring[h & (IMU_RING_SIZE - 1u)] = imu->buf[next];
				__DMB();
				imu->ring_head = h +1u;
			}
			imu->seq++;
		}
	}
	imu->dma_busy   = false;
	imu->fifo_state = FIFO_IDLE;
	if (imu->on_sample)
		imu->on_sample(imu->on_sample_user);
}

void icm45686_on_dma_error(icm45686_t *imu)
{
	icm_port_cs_high(&imu->bus);
	imu->stat_dma_error++;
	imu->dma_busy = false;
	imu->fifo_state = FIFO_IDLE;
}

bool icm45686_get_latest(icm45686_t *imu, icm45686_sample_t *out)
{
	uint32_t s1, s2;

	/* Đọc kiểu seqlock: nếu ISR ghi đè trong lúc copy thì copy lại */
	do {
		s1 = imu->seq;
		__DMB();
		*out = imu->buf[imu->latest];
		__DMB();
		s2 = imu->seq;
	} while (s1 != s2);

	bool fresh         = (s1 != imu->last_read_seq);
	imu->last_read_seq = s1;
	return fresh;
}

bool icm45686_pop(icm45686_t *imu, icm45686_sample_t *out)
{
	uint32_t t = imu->ring_tail;
	if (t == imu->ring_head)
		return false;
	*out = imu->ring[t & (IMU_RING_SIZE - 1u)];
	__DMB(); /* copy xong rồi mới trả slot */
	imu->ring_tail = t + 1u;
	return true;
}

uint32_t icm45686_us_since_last_sample(const icm45686_t *imu)
{
	return icm_port_micros() - imu->buf[imu->latest].timestamp_us;
}

void icm45686_recovery_stage_1(icm45686_t *imu)
{	
	if(imu->recovery_stage != 1) return;
	if(imu->fifo_state != FIFO_IDLE || imu->dma_busy ==true)
	{
		icm45686_stop(imu);
		icm45686_start(imu);
	}else if(imu->dma_busy == false && imu->fifo_state == FIFO_IDLE){
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		icm45686_on_drdy(imu);
		HAL_NVIC_EnableIRQ(EXTI0_IRQn)
	}
}