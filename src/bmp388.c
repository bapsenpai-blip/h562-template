/*
 * bmp388.c — Driver BMP388/BMP390 cho STM32 HAL, tối ưu cho flight controller.
 * Dựa trên https://github.com/DuePonto/BMP388-STM32-HAL (MIT).
 */
#include "bmp388.h"

#include <math.h>
#include <string.h>

/* ---- Thanh ghi ------------------------------------------------------------ */
#define REG_CHIP_ID       0x00u
#define REG_ERR           0x02u
#define REG_STATUS        0x03u
#define REG_DATA_0        0x04u
#define REG_FIFO_LENGTH_0 0x12u
#define REG_FIFO_DATA     0x14u
#define REG_FIFO_CONFIG_1 0x17u
#define REG_FIFO_CONFIG_2 0x18u
#define REG_INT_CTRL      0x19u
#define REG_IF_CONF       0x1Au
#define REG_PWR_CTRL      0x1Bu
#define REG_OSR           0x1Cu
#define REG_ODR           0x1Du
#define REG_CONFIG        0x1Fu
#define REG_CALIB         0x31u
#define REG_CMD           0x7Eu

#define CALIB_LEN 21u

/* Bit */
#define ERR_FATAL      0x01u
#define ERR_CMD        0x02u
#define ERR_CONF       0x04u
#define STATUS_CMD_RDY 0x10u
#define STATUS_DRDY_P  0x20u
#define STATUS_DRDY_T  0x40u

#define PWR_PRESS_EN   0x01u
#define PWR_TEMP_EN    0x02u
#define PWR_MODE_SLEEP 0x00u
#define PWR_MODE_NORM  0x30u
#define PWR_MODE_MASK  0x30u

#define INT_OD      0x01u
#define INT_LEVEL   0x02u
#define INT_DRDY_EN 0x40u

#define IF_I2C_WDT_EN   0x02u
#define IF_I2C_WDT_40MS 0x04u

#define FIFO1_MODE      0x01u
#define FIFO1_STOP_FULL 0x02u
#define FIFO1_PRESS_EN  0x08u
#define FIFO1_TEMP_EN   0x10u
#define FIFO2_FILTERED  0x08u /* data_select = 01: dữ liệu đã qua IIR */

#define CMD_SOFTRESET  0xB6u
#define CMD_FIFO_FLUSH 0xB0u

/* FIFO frame header */
#define FH_PRESS_TEMP 0x94u
#define FH_TEMP       0x90u
#define FH_PRESS      0x84u
#define FH_TIME       0xA0u
#define FH_EMPTY      0x80u
#define FH_CFG_CHANGE 0x48u
#define FH_CFG_ERROR  0x44u

/* Burst đường nóng: STATUS (0x03) .. DATA_5 (0x09) */
#define BURST_LEN 7u

/* ---- Bus ------------------------------------------------------------------ */
static inline void cs_low(const bmp388_t *d)
{
	d->cfg.cs_port->BSRR = (uint32_t)d->cfg.cs_pin << 16U;
}

static inline void cs_high(const bmp388_t *d)
{
	d->cfg.cs_port->BSRR = (uint32_t)d->cfg.cs_pin;
}

static bmp388_status_t bus_read(bmp388_t *d, uint8_t reg, uint8_t *buf, uint16_t len)
{
	HAL_StatusTypeDef st;

	if (d->cfg.bus == BMP388_BUS_I2C) {
		st = HAL_I2C_Mem_Read(d->cfg.hi2c, (uint16_t)(d->cfg.i2c_addr << 1), reg,
		                      I2C_MEMADD_SIZE_8BIT, buf, len, d->cfg.timeout_ms);
	} else {
		/* SPI: byte địa chỉ | 0x80, sau đó chip trả MỘT byte dummy rồi mới tới dữ liệu */
		uint8_t hdr[2] = { (uint8_t)(reg | 0x80u), 0 };
		uint8_t dummy[2];
		cs_low(d);
		st = HAL_SPI_TransmitReceive(d->cfg.hspi, hdr, dummy, 2, d->cfg.timeout_ms);
		if (st == HAL_OK)
			st = HAL_SPI_Receive(d->cfg.hspi, buf, len, d->cfg.timeout_ms);
		cs_high(d);
	}
	return (st == HAL_OK) ? BMP388_OK : BMP388_ERR_BUS;
}

static bmp388_status_t bus_write(bmp388_t *d, uint8_t reg, uint8_t val)
{
	HAL_StatusTypeDef st;

	if (d->cfg.bus == BMP388_BUS_I2C) {
		st = HAL_I2C_Mem_Write(d->cfg.hi2c, (uint16_t)(d->cfg.i2c_addr << 1), reg,
		                       I2C_MEMADD_SIZE_8BIT, &val, 1, d->cfg.timeout_ms);
	} else {
		uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), val };
		cs_low(d);
		st = HAL_SPI_Transmit(d->cfg.hspi, tx, 2, d->cfg.timeout_ms);
		cs_high(d);
	}
	return (st == HAL_OK) ? BMP388_OK : BMP388_ERR_BUS;
}

#define TRY(x)                               \
	do {                                     \
		bmp388_status_t _r = (x);            \
		if (_r != BMP388_OK)                 \
			return _r;                       \
	} while (0)

/* ---- Cache (Cortex-M7) ----------------------------------------------------- */
static inline void cache_clean(void *a, int32_t n)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
	if (SCB->CCR & SCB_CCR_DC_Msk)
		SCB_CleanDCache_by_Addr((uint32_t *)a, n);
#else
	(void)a;
	(void)n;
#endif
}

static inline void cache_invalidate(void *a, int32_t n)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
	if (SCB->CCR & SCB_CCR_DC_Msk)
		SCB_InvalidateDCache_by_Addr((uint32_t *)a, n);
#else
	(void)a;
	(void)n;
#endif
}

/* ---- Tiện ích ------------------------------------------------------------- */
static inline uint32_t u24(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

uint32_t bmp388_measurement_time_us(uint8_t osr_p, uint8_t osr_t)
{
	/* T = 234 + (392 + 2^osr_p * 2020) + (163 + 2^osr_t * 2020)  [µs] */
	return 234u + 392u + (2020u << osr_p) + 163u + (2020u << osr_t);
}

uint8_t bmp388_fastest_odr(uint8_t osr_p, uint8_t osr_t)
{
	uint32_t t = bmp388_measurement_time_us(osr_p, osr_t);
	uint8_t  odr;

	for (odr = 0; odr < BMP388_ODR_MAX; odr++)
		if ((5000u << odr) >= t)
			break;
	return odr;
}

static bmp388_status_t parse_calib(bmp388_calib_t *c, const uint8_t *b)
{
	bool all0 = true, allF = true;
	for (unsigned i = 0; i < CALIB_LEN; i++) {
		all0 &= (b[i] == 0x00);
		allF &= (b[i] == 0xFF);
	}
	if (all0 || allF)
		return BMP388_ERR_CALIB;

	const uint16_t t1  = (uint16_t)(b[0] | (b[1] << 8));
	const uint16_t t2  = (uint16_t)(b[2] | (b[3] << 8));
	const int8_t   t3  = (int8_t)b[4];
	const int16_t  p1  = (int16_t)(b[5] | (b[6] << 8));
	const int16_t  p2  = (int16_t)(b[7] | (b[8] << 8));
	const int8_t   p3  = (int8_t)b[9];
	const int8_t   p4  = (int8_t)b[10];
	const uint16_t p5  = (uint16_t)(b[11] | (b[12] << 8));
	const uint16_t p6  = (uint16_t)(b[13] | (b[14] << 8));
	const int8_t   p7  = (int8_t)b[15];
	const int8_t   p8  = (int8_t)b[16];
	const int16_t  p9  = (int16_t)(b[17] | (b[18] << 8));
	const int8_t   p10 = (int8_t)b[19];
	const int8_t   p11 = (int8_t)b[20];

	/* Hệ số lượng tử theo datasheet mục 9.1. Dùng ldexpf thay cho hằng số
	 * khổng lồ (2^65) như bản gốc. */
	c->t1_raw = (int32_t)t1 << 8;                 /* / 2^-8 */
	c->t2     = ldexpf((float)t2, -30);
	c->t3     = ldexpf((float)t3, -48);
	c->p1     = ldexpf((float)(p1 - 16384), -20);
	c->p2     = ldexpf((float)(p2 - 16384), -29);
	c->p3     = ldexpf((float)p3, -32);
	c->p4     = ldexpf((float)p4, -37);
	c->p5     = (float)p5 * 8.0f;                 /* / 2^-3 */
	c->p6     = ldexpf((float)p6, -6);
	c->p7     = ldexpf((float)p7, -8);
	c->p8     = ldexpf((float)p8, -15);
	c->p9     = ldexpf((float)p9, -48);
	c->p10    = ldexpf((float)p10, -48);
	c->p11    = ldexpf((float)p11, -65);
	return BMP388_OK;
}

void bmp388_compensate(const bmp388_calib_t *c, uint32_t raw_p, uint32_t raw_t, float *pressure_pa,
                       float *temperature_c)
{
	/* Nhiệt độ: trừ PAR_T1 bằng số nguyên -> không mất bit nào khi đổi float */
	const float d = (float)((int32_t)raw_t - c->t1_raw);
	const float t = d * (c->t2 + d * c->t3);

	/* Áp suất: cùng công thức datasheet, viết lại dạng Horner.
	 * Sai số so với bản double < 0.03 Pa (~2.5 mm). */
	const float x  = (float)raw_p;
	const float o1 = c->p5 + t * (c->p6 + t * (c->p7 + t * c->p8));
	const float s  = c->p1 + t * (c->p2 + t * (c->p3 + t * c->p4));
	const float q  = (c->p9 + c->p10 * t) + x * c->p11;

	*temperature_c = t;
	*pressure_pa   = o1 + x * (s + x * q);
}

/* ---- Cấu hình ------------------------------------------------------------- */
void bmp388_default_config(bmp388_config_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->bus             = BMP388_BUS_I2C;
	cfg->i2c_addr        = BMP388_I2C_ADDR_SDO_LOW;
	cfg->timeout_ms      = 10;
	cfg->osr_p           = BMP388_OSR_8X;
	cfg->osr_t           = BMP388_OSR_1X;
	cfg->iir             = BMP388_IIR_3;
	cfg->odr             = BMP388_ODR_50_HZ;
	cfg->int_drdy_enable = false;
	cfg->int_active_high = true;
	cfg->int_open_drain  = false;
}

static bmp388_status_t soft_reset(bmp388_t *d)
{
	uint8_t  v;
	uint32_t t0 = HAL_GetTick();

	/* Chờ cmd_rdy (bản gốc bỏ qua reset một cách im lặng nếu chưa sẵn sàng) */
	for (;;) {
		TRY(bus_read(d, REG_STATUS, &v, 1));
		if (v & STATUS_CMD_RDY)
			break;
		if ((HAL_GetTick() - t0) > 10u)
			return BMP388_ERR_TIMEOUT;
	}

	TRY(bus_write(d, REG_CMD, CMD_SOFTRESET));
	HAL_Delay(3); /* datasheet: 2 ms; +1 vì HAL_Delay có thể ngắn hơn 1 tick */

	if (d->cfg.bus == BMP388_BUS_SPI)
		(void)bus_read(d, REG_CHIP_ID, &v, 1); /* chốt lại giao tiếp SPI */

	TRY(bus_read(d, REG_ERR, &v, 1));
	return (v & (ERR_CMD | ERR_FATAL)) ? BMP388_ERR_CMD : BMP388_OK;
}

static bmp388_status_t apply_config(bmp388_t *d)
{
	const bmp388_config_t *c = &d->cfg;
	uint8_t                v;

	if (c->osr_p > BMP388_OSR_32X || c->osr_t > BMP388_OSR_32X || c->iir > BMP388_IIR_127 ||
	    c->odr > BMP388_ODR_MAX)
		return BMP388_ERR_ARG;

	/* Kiểm tra trước để không rơi vào conf_err (chip tự về sleep) */
	d->meas_time_us = bmp388_measurement_time_us(c->osr_p, c->osr_t);
	if ((5000u << c->odr) < d->meas_time_us)
		return BMP388_ERR_CONF;

	/* Phải ở sleep khi đổi OSR/ODR/IIR */
	TRY(bus_write(d, REG_PWR_CTRL, PWR_MODE_SLEEP));
	TRY(bus_write(d, REG_OSR, (uint8_t)((c->osr_t << 3) | c->osr_p)));
	TRY(bus_write(d, REG_ODR, c->odr));
	TRY(bus_write(d, REG_CONFIG, (uint8_t)(c->iir << 1)));

	v = 0;
	if (c->int_drdy_enable)
		v |= INT_DRDY_EN;
	if (c->int_active_high)
		v |= INT_LEVEL;
	if (c->int_open_drain)
		v |= INT_OD;
	TRY(bus_write(d, REG_INT_CTRL, v));

	/* I2C watchdog 40 ms: chip tự nhả SDA nếu master bị treo giữa giao dịch */
	TRY(bus_write(d, REG_IF_CONF,
	              (c->bus == BMP388_BUS_I2C) ? (IF_I2C_WDT_EN | IF_I2C_WDT_40MS) : 0u));

	TRY(bus_write(d, REG_PWR_CTRL, PWR_PRESS_EN | PWR_TEMP_EN | PWR_MODE_NORM));
	HAL_Delay(2);

	TRY(bus_read(d, REG_ERR, &v, 1));
	if (v & ERR_CONF)
		return BMP388_ERR_CONF;
	TRY(bus_read(d, REG_PWR_CTRL, &v, 1));
	if ((v & PWR_MODE_MASK) != PWR_MODE_NORM)
		return BMP388_ERR_CONF;

	return BMP388_OK;
}

bmp388_status_t bmp388_init(bmp388_t *dev, const bmp388_config_t *cfg)
{
	uint8_t buf[CALIB_LEN];

	if (dev == NULL || cfg == NULL)
		return BMP388_ERR_ARG;
	if (cfg->bus == BMP388_BUS_I2C && cfg->hi2c == NULL)
		return BMP388_ERR_ARG;
	if (cfg->bus == BMP388_BUS_SPI && (cfg->hspi == NULL || cfg->cs_port == NULL))
		return BMP388_ERR_ARG;

	memset(dev, 0, sizeof(*dev));
	dev->cfg = *cfg;
	if (dev->cfg.timeout_ms == 0)
		dev->cfg.timeout_ms = 10;

	if (cfg->bus == BMP388_BUS_SPI) {
		cs_high(dev);
		(void)bus_read(dev, REG_CHIP_ID, buf, 1); /* lần đọc đầu chọn chế độ SPI */
	}

	TRY(bus_read(dev, REG_CHIP_ID, &dev->chip_id, 1));
	if (dev->chip_id != BMP388_CHIP_ID && dev->chip_id != BMP390_CHIP_ID)
		return BMP388_ERR_CHIP_ID; /* bản gốc trả HAL_OK ở đây */

	TRY(soft_reset(dev));
	TRY(bus_read(dev, REG_CALIB, buf, CALIB_LEN));
	TRY(parse_calib(&dev->calib, buf));
	TRY(apply_config(dev));

	return BMP388_OK;
}

bmp388_status_t bmp388_reconfigure(bmp388_t *dev, uint8_t osr_p, uint8_t osr_t, uint8_t iir,
                                   uint8_t odr)
{
	bmp388_config_t old = dev->cfg;
	bmp388_status_t rc;

	if (dev->busy)
		return BMP388_ERR_BUSY;

	dev->cfg.osr_p = osr_p;
	dev->cfg.osr_t = osr_t;
	dev->cfg.iir   = iir;
	dev->cfg.odr   = odr;

	rc = apply_config(dev);
	if (rc == BMP388_ERR_ARG || rc == BMP388_ERR_CONF) {
		dev->cfg = old;
		(void)apply_config(dev);
	}
	return rc;
}

/* ---- Đọc blocking --------------------------------------------------------- */
static void finish_sample(bmp388_t *d, uint32_t rp, uint32_t rt, uint32_t ts, uint32_t seq,
                          bmp388_sample_t *s)
{
	bmp388_compensate(&d->calib, rp, rt, &s->pressure_pa, &s->temperature_c);
	s->raw_pressure    = rp;
	s->raw_temperature = rt;
	s->timestamp_ms    = ts;
	s->seq             = seq;
}

bmp388_status_t bmp388_read_blocking(bmp388_t *dev, bmp388_sample_t *out)
{
	uint8_t b[BURST_LEN];

	if (dev->busy)
		return BMP388_ERR_BUSY;

	uint32_t ts = HAL_GetTick();
	TRY(bus_read(dev, REG_STATUS, b, BURST_LEN));

	/* drdy_press tự xóa khi đọc DATA -> burst này vừa kiểm tra vừa xóa cờ */
	if (!(b[0] & STATUS_DRDY_P)) {
		dev->stat_stale++;
		return BMP388_ERR_NO_DATA;
	}

	dev->raw_seq++;
	finish_sample(dev, u24(&b[1]), u24(&b[4]), ts, dev->raw_seq, out);
	dev->last     = *out;
	dev->proc_seq = dev->raw_seq;
	return BMP388_OK;
}

bmp388_status_t bmp388_wait_sample(bmp388_t *dev, bmp388_sample_t *out, uint32_t timeout_ms)
{
	uint32_t        t0 = HAL_GetTick();
	bmp388_status_t rc;

	for (;;) {
		rc = bmp388_read_blocking(dev, out);
		if (rc != BMP388_ERR_NO_DATA)
			return rc;
		if ((HAL_GetTick() - t0) >= timeout_ms)
			return BMP388_ERR_TIMEOUT;
		HAL_Delay(1);
	}
}

/* ---- Đọc DMA --------------------------------------------------------------- */
bmp388_status_t bmp388_trigger_read(bmp388_t *dev)
{
	HAL_StatusTypeDef st;

	if (dev->busy) {
		dev->stat_busy++;
		return BMP388_ERR_BUSY;
	}
	dev->busy       = true;
	dev->xfer_ts_ms = HAL_GetTick();

	if (dev->cfg.bus == BMP388_BUS_I2C) {
		st = HAL_I2C_Mem_Read_DMA(dev->cfg.hi2c, (uint16_t)(dev->cfg.i2c_addr << 1), REG_STATUS,
		                          I2C_MEMADD_SIZE_8BIT, dev->dma_rx, BURST_LEN);
	} else {
		memset(dev->dma_tx, 0, BURST_LEN + 2u);
		dev->dma_tx[0] = REG_STATUS | 0x80u;
		cache_clean(dev->dma_tx, sizeof(dev->dma_tx));
		cs_low(dev);
		st = HAL_SPI_TransmitReceive_DMA(dev->cfg.hspi, dev->dma_tx, dev->dma_rx,
		                                 (uint16_t)(BURST_LEN + 2u)); /* addr + dummy + 7 */
		if (st != HAL_OK)
			cs_high(dev);
	}

	if (st != HAL_OK) {
		dev->busy = false;
		dev->stat_bus_error++;
		return BMP388_ERR_BUS;
	}
	return BMP388_OK;
}

void bmp388_on_xfer_complete(bmp388_t *dev)
{
	const uint8_t *b;

	if (!dev->busy)
		return;

	if (dev->cfg.bus == BMP388_BUS_SPI) {
		cs_high(dev);
		b = &dev->dma_rx[2];
	} else {
		b = &dev->dma_rx[0];
	}
	cache_invalidate(dev->dma_rx, sizeof(dev->dma_rx));

	if (b[0] & STATUS_DRDY_P) {
		dev->raw_p  = u24(&b[1]);
		dev->raw_t  = u24(&b[4]);
		dev->raw_ts = dev->xfer_ts_ms;
		__DMB();
		dev->raw_seq++;
	} else {
		dev->stat_stale++;
	}
	dev->busy = false;
}

void bmp388_on_xfer_error(bmp388_t *dev)
{
	if (dev->cfg.bus == BMP388_BUS_SPI)
		cs_high(dev);
	dev->stat_bus_error++;
	dev->busy = false;
}

bool bmp388_get_latest(bmp388_t *dev, bmp388_sample_t *out)
{
	if (dev->raw_seq != dev->proc_seq) {
		uint32_t s, rp, rt, ts;
		do { /* seqlock: ISR ghi đè giữa chừng thì đọc lại */
			s  = dev->raw_seq;
			__DMB();
			rp = dev->raw_p;
			rt = dev->raw_t;
			ts = dev->raw_ts;
			__DMB();
		} while (s != dev->raw_seq);

		finish_sample(dev, rp, rt, ts, s, &dev->last);
		dev->proc_seq = s;
	}

	*out          = dev->last;
	bool fresh    = (dev->last.seq != dev->read_seq) && (dev->last.seq != 0);
	dev->read_seq = dev->last.seq;
	return fresh;
}

/* ---- Độ cao ---------------------------------------------------------------- */
float bmp388_pressure_to_altitude(float pressure_pa, float ref_pa)
{
	/* Khí quyển chuẩn: h = 44330 * (1 - (p/p0)^(1/5.255)). powf (float) thay cho
	 * pow (double) của bản gốc: nhanh hơn nhiều lần trên FPU đơn của F4/F7. */
	return 44330.0f * (1.0f - powf(pressure_pa / ref_pa, 0.190295f));
}

void bmp388_set_ground(bmp388_t *dev, float ground_pa)
{
	dev->ground_pa     = ground_pa;
	dev->inv_ground_pa = (ground_pa > 0.0f) ? 1.0f / ground_pa : 0.0f;
}

float bmp388_altitude_m(const bmp388_t *dev, float pressure_pa)
{
	if (dev->inv_ground_pa == 0.0f)
		return 0.0f;
	return 44330.0f * (1.0f - powf(pressure_pa * dev->inv_ground_pa, 0.190295f));
}

bmp388_status_t bmp388_calibrate_ground(bmp388_t *dev, uint16_t samples)
{
	bmp388_sample_t s;
	float           sum = 0.0f;
	uint32_t        period_ms = (5u << dev->cfg.odr);
	uint32_t        timeout   = 2u * period_ms + dev->meas_time_us / 1000u + 5u;

	if (samples == 0)
		return BMP388_ERR_ARG;

	/* Bỏ các mẫu khi IIR chưa hội tụ: ~3 hằng số thời gian */
	uint16_t discard = (uint16_t)(3u * (1u << dev->cfg.iir) + 2u);
	for (uint16_t i = 0; i < discard; i++)
		TRY(bmp388_wait_sample(dev, &s, timeout));

	for (uint16_t i = 0; i < samples; i++) {
		TRY(bmp388_wait_sample(dev, &s, timeout));
		sum += s.pressure_pa;
	}

	bmp388_set_ground(dev, sum / (float)samples);
	return BMP388_OK;
}

/* ---- FIFO ------------------------------------------------------------------ */
bmp388_status_t bmp388_fifo_start(bmp388_t *dev, bool stop_on_full)
{
	if (dev->busy)
		return BMP388_ERR_BUSY;

	/* Chọn dữ liệu ĐÃ LỌC IIR. Mặc định chip đẩy dữ liệu chưa lọc vào FIFO,
	 * nên bản gốc mất tác dụng của cài đặt IIR khi dùng FIFO. */
	TRY(bus_write(dev, REG_FIFO_CONFIG_2, FIFO2_FILTERED));
	TRY(bus_write(dev, REG_FIFO_CONFIG_1,
	              (uint8_t)(FIFO1_MODE | FIFO1_PRESS_EN | FIFO1_TEMP_EN |
	                        (stop_on_full ? FIFO1_STOP_FULL : 0u))));
	return bmp388_fifo_flush(dev);
}

bmp388_status_t bmp388_fifo_flush(bmp388_t *dev)
{
	return bus_write(dev, REG_CMD, CMD_FIFO_FLUSH);
}

bmp388_status_t bmp388_fifo_read(bmp388_t *dev, uint8_t *buf, uint16_t buf_size, uint16_t *len_out)
{
	uint8_t  l[2];
	uint16_t n;

	*len_out = 0;
	if (dev->busy)
		return BMP388_ERR_BUSY;

	TRY(bus_read(dev, REG_FIFO_LENGTH_0, l, 2));
	n = (uint16_t)((l[0] | (l[1] << 8)) & 0x01FFu); /* 9 bit, tối đa 512 */
	if (n == 0)
		return BMP388_OK;

	/* Không bao giờ đọc quá bộ đệm (bản gốc đọc len+4 và cắt len về uint8_t) */
	if (n > buf_size)
		n = (uint16_t)(buf_size - (buf_size % 7u)); /* giữ nguyên frame 7 byte */
	if (n == 0)
		return BMP388_ERR_ARG;

	TRY(bus_read(dev, REG_FIFO_DATA, buf, n));
	*len_out = n;
	return BMP388_OK;
}

uint16_t bmp388_fifo_parse(const bmp388_t *dev, const uint8_t *buf, uint16_t len,
                           bmp388_fifo_frame_t *out, uint16_t max_frames)
{
	uint16_t i = 0, n = 0;

	while (i < len && n < max_frames) {
		const uint8_t h = buf[i++];

		switch (h) {
		case FH_PRESS_TEMP: /* nhiệt độ 3 byte trước, áp suất 3 byte sau */
			if (i + 6u > len)
				return n;
			bmp388_compensate(&dev->calib, u24(&buf[i + 3]), u24(&buf[i]), &out[n].pressure_pa,
			                  &out[n].temperature_c);
			n++;
			i = (uint16_t)(i + 6u);
			break;
		case FH_TEMP:
		case FH_PRESS:
		case FH_TIME:
			i = (uint16_t)(i + 3u);
			break;
		case FH_CFG_CHANGE:
		case FH_CFG_ERROR:
			i = (uint16_t)(i + 1u);
			break;
		case FH_EMPTY:
		default:
			return n; /* hết dữ liệu hoặc header lạ */
		}
	}
	return n;
}
