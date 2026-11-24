/*
 * mavlink_send.h — Bộ phát MAVLink v2 không chặn (FIFO + UART IT/DMA) cho STM32 HAL.
 *
 * Thiết kế đồng bộ với icm45686_fc.h và bmp388.h:
 *   - Mọi trạng thái, bộ đệm FIFO và bộ đếm chẩn đoán nằm trong struct mavlink_tx_t.
 *   - Không phụ thuộc biến toàn cục cứng (như huart1); truyền UART qua mavlink_config_t.
 *   - Luồng sử dụng:
 *       mavlink_default_config(&mcfg);
 *       mcfg.huart = &huart1;
 *       mavlink_init(&mav, &mcfg);
 *       // Trong HAL_UART_TxCpltCallback -> mavlink_on_tx_complete(&mav, huart);
 *       // Trong HAL_UART_ErrorCallback  -> mavlink_on_tx_error(&mav, huart);
 *       // Trong vòng lặp main:
 *       mavlink_poll(&mav); // watchdog chống treo TX
 *       mavlink_send_heartbeat(&mav);
 *       mavlink_send_attitude(&mav, roll_rad, pitch_rad, yaw_rad, gx, gy, gz);
 *       mavlink_send_scaled_pressure(&mav, baro.pressure, baro.temp);
 */
#ifndef MAVLINK_SEND_H
#define MAVLINK_SEND_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32h5xx_hal.h"
#include "config.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <mavlink/common/mavlink.h>
#pragma GCC diagnostic pop

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Hằng số --------------------------------------------------------------- */
#define MAVLINK_TX_FIFO_SIZE 1024u /* phải là lũy thừa của 2 */
#define MAVLINK_TX_FIFO_MASK (MAVLINK_TX_FIFO_SIZE - 1u)

/* ---- Kiểu trả về (giống bmp388_status_t; tránh trùng mavlink_status_t của thư viện) */
typedef enum {
	MAVLINK_TX_OK       = 0,
	MAVLINK_TX_ERR_ARG  = -1,
	MAVLINK_TX_ERR_FULL = -2, /* FIFO không đủ chỗ chứa trọn gói -> bỏ gói */
	MAVLINK_TX_ERR_BUS  = -3, /* HAL UART trả lỗi */
} mavlink_tx_status_t;

/* ---- Cấu hình -------------------------------------------------------------- */
typedef struct {
	UART_HandleTypeDef *huart;
	uint8_t             sys_id;        /* Mặc định: 1 */
	uint8_t             comp_id;       /* Mặc định: MAV_COMP_ID_AUTOPILOT1 */
	uint8_t             mav_type;      /* Mặc định: MAV_TYPE_FIXED_WING */
	uint8_t             autopilot;     /* Mặc định: MAV_AUTOPILOT_GENERIC */
	uint8_t             base_mode;     /* Mặc định: 0 */
	uint32_t            custom_mode;   /* Mặc định: 0 */
	uint8_t             system_status; /* Mặc định: MAV_STATE_STANDBY */
	uint32_t            tx_timeout_ms; /* Mặc định: 250 ms */
	bool                use_dma;       /* true: HAL_UART_Transmit_DMA, false: HAL_UART_Transmit_IT */
} mavlink_config_t;

/* ---- Instance -------------------------------------------------------------- */
typedef struct {
	mavlink_config_t cfg;

	/* Hàng đợi vòng TX: chỉ main ghi tx_head, chỉ ISR (hoặc khi tắt ngắt) ghi tx_tail */
	uint8_t           tx_fifo[MAVLINK_TX_FIFO_SIZE] __attribute__((aligned(32)));
	volatile uint16_t tx_head;
	volatile uint16_t tx_tail;
	volatile uint16_t tx_chunk;
	volatile bool     tx_busy;
	volatile uint32_t tx_start_ms;

#ifdef USE_GPS
	bool    home_set;
	int32_t home_alt_mm;
#endif

	/* Thống kê (xem trong Live Watch giống bmp.stat_* / imu.stat_*) */
	volatile uint32_t stat_drop;      /* số gói bị bỏ vì FIFO đầy */
	volatile uint32_t stat_timeout;   /* số lần hủy truyền do quá hạn tx_timeout_ms */
	volatile uint32_t stat_bus_error; /* số lần UART báo lỗi */
	volatile uint32_t stat_hb_sent;
	volatile uint32_t stat_att_sent;
	volatile uint32_t stat_baro_sent;
	volatile uint32_t stat_gps_sent;
} mavlink_tx_t;

/* ---- API khởi tạo & callback HAL ------------------------------------------- */

/* Điền cấu hình mặc định cho UAV cánh bằng (SysID=1, MAV_TYPE_FIXED_WING, IT mode). */
void mavlink_default_config(mavlink_config_t *cfg);

/* Khởi tạo instance mavlink_tx_t. */
mavlink_tx_status_t mavlink_init(mavlink_tx_t *dev, const mavlink_config_t *cfg);

/* Gọi trong vòng lặp main: tự động hủy & kích lại nếu UART TX bị kẹt quá timeout. */
void mavlink_poll(mavlink_tx_t *dev);

/* Gọi từ HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart). */
void mavlink_on_tx_complete(mavlink_tx_t *dev, UART_HandleTypeDef *huart);

/* Gọi từ HAL_UART_ErrorCallback(UART_HandleTypeDef *huart). */
void mavlink_on_tx_error(mavlink_tx_t *dev, UART_HandleTypeDef *huart);

/* ---- API gửi gói tin ------------------------------------------------------- */

/* Đóng gói mavlink_message_t vào FIFO và kích UART TX nếu đang rảnh. */
mavlink_tx_status_t mavlink_send_msg(mavlink_tx_t *dev, const mavlink_message_t *msg);

/* HEARTBEAT (1 Hz): bắt buộc để QGroundControl / Mission Planner nhận diện kết nối. */
mavlink_tx_status_t mavlink_send_heartbeat(mavlink_tx_t *dev);

/* ATTITUDE (10..50 Hz): góc Euler (rad) và tốc độ góc (rad/s). */
mavlink_tx_status_t mavlink_send_attitude(mavlink_tx_t *dev, float roll_rad, float pitch_rad,
                                          float yaw_rad, float gx_rad_s, float gy_rad_s,
                                          float gz_rad_s);

/* SCALED_PRESSURE (5..10 Hz): gửi áp suất (Pa) và nhiệt độ (°C) từ BMP388. */
mavlink_tx_status_t mavlink_send_scaled_pressure(mavlink_tx_t *dev, float pressure_pa,
                                                 float temperature_c);

/* VFR_HUD (2..10 Hz): gửi độ cao (m), tốc độ leo (m/s), tốc độ bay, góc hướng cho HUD cánh bằng. */
mavlink_tx_status_t mavlink_send_vfr_hud(mavlink_tx_t *dev, float airspeed_m_s,
                                         float groundspeed_m_s, int16_t heading_deg,
                                         uint16_t throttle_pct, float alt_m, float climb_m_s);

#ifdef USE_GPS
/* GPS_RAW_INT & GLOBAL_POSITION_INT lấy dữ liệu từ biến toàn cục gps (ubx_gps.h). */
mavlink_tx_status_t mavlink_send_gps_raw(mavlink_tx_t *dev);
mavlink_tx_status_t mavlink_send_global_position(mavlink_tx_t *dev);

/* Gọi ngay khi GPS_Poll() trả về 1 (có gói NAV-PVT mới): tự chốt Home và gửi gói GPS. */
void mavlink_on_new_gps_fix(mavlink_tx_t *dev);
#endif

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_SEND_H */
