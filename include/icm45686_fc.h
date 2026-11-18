/*
 * icm45686_fc.h
 *
 * Lớp driver ICM-45686 cho flight controller, xây trên driver eMD của TDK.
 *
 *  - Cấu hình (reset, FSR, ODR, bộ lọc UI, ngắt) đi qua API TDK (inv_imu_*).
 *  - Đường dữ liệu nóng: ngắt DRDY trên INT1 -> EXTI -> đọc burst 14 byte
 *    bằng SPI DMA -> parse trong callback hoàn tất DMA -> double buffer.
 *  - Vòng PID lấy mẫu mới nhất bằng icm45686_get_latest() (không chặn).
 *
 * Luồng sử dụng điển hình:
 *     icm45686_default_config(&cfg); cfg.hspi = &hspi1; ...
 *     icm45686_init(&imu, &cfg);
 *     icm45686_calibrate_gyro(&imu, 2000, 2.0f);  // máy bay đứng yên
 *     icm45686_start(&imu);
 *     // HAL_GPIO_EXTI_Callback  -> icm45686_on_drdy(&imu)
 *     // HAL_SPI_TxRxCpltCallback -> icm45686_on_dma_complete(&imu)
 *     // HAL_SPI_ErrorCallback   -> icm45686_on_dma_error(&imu)
 */
#ifndef ICM45686_FC_H
#define ICM45686_FC_H

#include <stdbool.h>
#include <stdint.h>

#include "imu/inv_imu_driver.h"
#include "icm45686_port_stm32.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Trên STM32H7, bộ đệm DMA phải nằm ở RAM mà DMA truy cập được (không phải
 * DTCM) và nên ở vùng non-cacheable. Định nghĩa macro này trước khi include
 * (hoặc trong compiler flags) để đặt instance icm45686_t vào section phù hợp,
 * ví dụ:  -DICM45686_DMA_ATTR='__attribute__((section(".ram_d2")))'          */
#ifndef ICM45686_DMA_ATTR
#define ICM45686_DMA_ATTR
#endif

/* Mã lỗi riêng (âm). Lỗi từ driver TDK (INV_IMU_ERROR_*) cũng được trả về
 * nguyên vẹn. */
#define ICM45686_OK              0
#define ICM45686_ERR_WHOAMI     -100
#define ICM45686_ERR_ARG        -101
#define ICM45686_ERR_BUSY       -102
#define ICM45686_ERR_MOVING     -103
#define ICM45686_ERR_NO_DATA    -104

/* Hướng gắn cảm biến so với thân máy bay (giống quy ước Betaflight/INAV). */
typedef enum {
	ICM45686_ALIGN_CW0 = 0,
	ICM45686_ALIGN_CW90,
	ICM45686_ALIGN_CW180,
	ICM45686_ALIGN_CW270,
	ICM45686_ALIGN_CW0_FLIP,
	ICM45686_ALIGN_CW90_FLIP,
	ICM45686_ALIGN_CW180_FLIP,
	ICM45686_ALIGN_CW270_FLIP,
} icm45686_align_t;

typedef struct {
	/* Phần cứng */
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *     cs_port;
	uint16_t           cs_pin;
	uint32_t           spi_prescaler_init; /* <= ~1 MHz khi cấu hình */
	uint32_t           spi_prescaler_run;  /* <= 24 MHz khi đọc dữ liệu */

	/* Cảm biến (dùng thẳng enum của TDK) */
	gyro_config0_gyro_odr_t                gyro_odr;
	gyro_config0_gyro_ui_fs_sel_t          gyro_fsr;
	ipreg_sys1_reg_172_gyro_ui_lpfbw_sel_t gyro_lpf;
	accel_config0_accel_odr_t              accel_odr;
	accel_config0_accel_ui_fs_sel_t        accel_fsr;
	ipreg_sys2_reg_131_accel_ui_lpfbw_t    accel_lpf;

	icm45686_align_t align;

	/* Nếu INT1 chỉ phát 1 xung rồi đứng (tùy silicon/cấu hình), bật cờ này:
	 * burst DMA sẽ kéo dài tới INT1_STATUS0 (0x19) để xóa cờ ngắt mỗi mẫu.
	 * Đọc 26 byte thay vì 14 byte. */
	bool read_int_status;
} icm45686_config_t;

typedef struct {
	int16_t  gyro_raw[3];  /* đã xoay theo align, chưa trừ bias */
	int16_t  accel_raw[3]; /* đã xoay theo align */
	int16_t  temp_raw;
	float    gyro_dps[3];  /* đã trừ bias */
	float    accel_g[3];
	float    temp_c;
	uint32_t timestamp_us; /* thời điểm cạnh lên DRDY */
	uint32_t seq;          /* tăng mỗi mẫu */
} icm45686_sample_t;

typedef struct {
	/* Driver TDK */
	inv_imu_device_t dev;
	icm_port_bus_t   bus;
	icm45686_config_t cfg;

	/* Hệ số đổi đơn vị */
	float gyro_dps_per_lsb;
	float accel_g_per_lsb;
	float gyro_bias_dps[3];
	uint32_t gyro_odr_hz;

	/* Trạng thái đường dữ liệu nóng */
	volatile bool     streaming;
	volatile bool     dma_busy;
	volatile uint32_t drdy_ts_us;
	volatile uint32_t seq;
	volatile uint8_t  latest;
	icm45686_sample_t buf[2];
	uint32_t          last_read_seq;
	uint16_t          discard; /* số mẫu bỏ qua sau khi bật gyro */

	/* Thống kê chẩn đoán */
	volatile uint32_t stat_overrun;   /* DRDY tới khi DMA trước chưa xong */
	volatile uint32_t stat_dma_error;
	volatile uint32_t stat_invalid;   /* mẫu = 0x8000 (chưa sẵn sàng) */

	/* Callback tùy chọn, gọi trong ngữ cảnh ngắt khi có mẫu mới
	 * (ví dụ: kích tác vụ PID). */
	void (*on_sample)(void *user);
	void *on_sample_user;

	/* Bộ đệm DMA, căn 32 byte cho cache line của Cortex-M7 */
	uint8_t dma_tx[32] __attribute__((aligned(32)));
	uint8_t dma_rx[32] __attribute__((aligned(32)));
} icm45686_t;

/* Cấu hình mặc định cho FC: gyro 6.4 kHz ±2000 dps, LPF = ODR/8 (~800 Hz),
 * accel 1.6 kHz ±16 g, LPF = ODR/4, CW0, SPI init ~1 MHz / run ~12-24 MHz
 * (prescaler phải tự chỉnh theo xung clock SPI của bạn). */
void icm45686_default_config(icm45686_config_t *cfg);

/* Reset + cấu hình cảm biến (blocking, ~100 ms). Gọi khi ngắt EXTI của INT1
 * chưa được dùng hoặc streaming đang tắt. */
int icm45686_init(icm45686_t *imu, const icm45686_config_t *cfg);

/* Đọc một mẫu blocking qua API TDK (dùng khi chưa start streaming). */
int icm45686_read_blocking(icm45686_t *imu, icm45686_sample_t *out);

/* Hiệu chuẩn bias gyro: lấy trung bình 'samples' mẫu, trả về
 * ICM45686_ERR_MOVING nếu độ lệch (max-min) trên trục nào > max_spread_dps.
 * Gọi trước icm45686_start(). */
int icm45686_calibrate_gyro(icm45686_t *imu, uint32_t samples, float max_spread_dps);

/* Bật/tắt đường dữ liệu DMA theo ngắt DRDY. */
int  icm45686_start(icm45686_t *imu);
void icm45686_stop(icm45686_t *imu);

/* Hook ngắt — gọi từ callback HAL tương ứng. */
void icm45686_on_drdy(icm45686_t *imu);
void icm45686_on_dma_complete(icm45686_t *imu);
void icm45686_on_dma_error(icm45686_t *imu);

/* Lấy mẫu mới nhất (an toàn khi bị ngắt chen ngang).
 * Trả về true nếu là mẫu mới kể từ lần gọi trước. */
bool icm45686_get_latest(icm45686_t *imu, icm45686_sample_t *out);

/* Thời gian từ mẫu cuối (để phát hiện IMU chết/treo trong FC). */
uint32_t icm45686_us_since_last_sample(const icm45686_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_FC_H */
