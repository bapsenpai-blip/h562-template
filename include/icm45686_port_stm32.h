/*
 * icm45686_port_stm32.h
 *
 * Lớp port STM32 HAL cho driver eMD ICM-45686 của TDK
 * (https://github.com/tdk-invn-oss/motion.mcu.icm45686.driver).
 *
 * Thay thế cho examples/system_interface.c (bản gốc chỉ là stub rỗng):
 *   - read_reg / write_reg qua SPI 4 dây (HAL, blocking)  -> dùng cho cấu hình
 *   - sleep_us dựa trên bộ đếm chu kỳ DWT
 *   - micros() 32-bit để đóng dấu thời gian mẫu
 *
 * Yêu cầu: lõi Cortex-M3/M4/M7/M33 (có DWT). Các dòng F4/F7/H7/G4/F405/F722/H743
 * dùng cho flight controller đều thỏa.
 */
#ifndef ICM45686_PORT_STM32_H
#define ICM45686_PORT_STM32_H

#include <stdint.h>
/* Include thẳng HAL của dòng chip, KHÔNG include "main.h": main.h thường
 * include ngược lại driver -> vòng include (icm_port_bus_t chưa khai báo). */
#if defined(ICM_PORT_HAL_HEADER)
#include ICM_PORT_HAL_HEADER
#elif __has_include("stm32h5xx_hal.h")
#include "stm32h5xx_hal.h"
#elif __has_include("stm32h7xx_hal.h")
#include "stm32h7xx_hal.h"
#elif __has_include("stm32f7xx_hal.h")
#include "stm32f7xx_hal.h"
#elif __has_include("stm32f4xx_hal.h")
#include "stm32f4xx_hal.h"
#elif __has_include("stm32g4xx_hal.h")
#include "stm32g4xx_hal.h"
#else
#error "Không tìm thấy HAL STM32: định nghĩa ICM_PORT_HAL_HEADER, ví dụ -DICM_PORT_HAL_HEADER=\"stm32xxxx_hal.h\""
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Bit đọc trong byte địa chỉ SPI. Driver TDK KHÔNG tự thêm bit này,
 * lớp port phải tự OR vào. */
#define ICM_SPI_READ_BIT 0x80u

/* Thông tin bus cho một con IMU (truyền vào transport.context). */
typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *     cs_port;
	uint16_t           cs_pin;
	uint32_t           timeout_ms; /* timeout cho các giao dịch blocking */
} icm_port_bus_t;

/* Khởi tạo DWT cycle counter. Gọi một lần sau SystemClock_Config(). */
void icm_port_timer_init(void);

/* Delay chặn (busy-wait) theo micro giây. Khớp chữ ký transport.sleep_us. */
void icm_port_sleep_us(uint32_t us);

/* Thời gian hiện tại tính bằng micro giây (tràn sau ~71 phút, dùng phép trừ
 * không dấu để tính khoảng thời gian). */
uint32_t icm_port_micros(void);

/* Callback cho inv_imu_transport_t. context = icm_port_bus_t*. */
int icm_port_spi_read_reg(void *context, uint8_t reg, uint8_t *buf, uint32_t len);
int icm_port_spi_write_reg(void *context, uint8_t reg, const uint8_t *buf, uint32_t len);

/* Đổi prescaler SPI lúc chạy (ví dụ chậm khi init, nhanh khi đọc dữ liệu).
 * prescaler là hằng HAL: SPI_BAUDRATEPRESCALER_2 ... _256. */
int icm_port_spi_set_prescaler(SPI_HandleTypeDef *hspi, uint32_t prescaler);

static inline void icm_port_cs_low(const icm_port_bus_t *b)
{
	b->cs_port->BSRR = (uint32_t)b->cs_pin << 16U;
}

static inline void icm_port_cs_high(const icm_port_bus_t *b)
{
	b->cs_port->BSRR = (uint32_t)b->cs_pin;
}

#ifdef __cplusplus
}
#endif

#endif /* ICM45686_PORT_STM32_H */
