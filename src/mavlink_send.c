/*
 * mavlink_send.c — Bộ phát MAVLink v2 không chặn (FIFO + UART IT/DMA) cho STM32 HAL.
 */
#include "mavlink_send.h"
#include <string.h>

#ifdef USE_GPS
#include "ubx_gps.h"
#endif

/* ---- Nội bộ: kích truyền khúc liền mạch tiếp theo trong FIFO --------------- */
/* Gọi khi đã tắt ngắt hoặc từ trong ISR TxCplt. */
static void uart_tx_kick(mavlink_tx_t *d)
{
	if (d->tx_busy || d->tx_head == d->tx_tail)
		return;

	uint16_t head = d->tx_head;
	uint16_t tail = d->tx_tail;
	uint16_t n    = (head > tail) ? (uint16_t)(head - tail)
	                              : (uint16_t)(MAVLINK_TX_FIFO_SIZE - tail);

	d->tx_chunk    = n;
	d->tx_busy     = true;
	d->tx_start_ms = HAL_GetTick();

	HAL_StatusTypeDef st;
	if (d->cfg.use_dma) {
		/* STM32H5 định nghĩa USART_DMAREQUESTS_SW_WA nên HAL không tự xóa DMAT khi xong;
		 * xóa DMAT trước khi nạp lại GPDMA để luôn tạo sườn lên DMAT sạch cho lần truyền mới. */
		st = HAL_UART_Transmit_DMA(d->cfg.huart, &d->tx_fifo[tail], n);
	} else {
		st = HAL_UART_Transmit_IT(d->cfg.huart, &d->tx_fifo[tail], n);
	}

	if (st != HAL_OK) {
		d->tx_busy  = false;
		d->tx_chunk = 0;
		d->stat_bus_error++;
	}
}

/* ---- Khởi tạo & cấu hình --------------------------------------------------- */
void mavlink_default_config(mavlink_config_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->huart         = NULL;
	cfg->sys_id        = 1u;
	cfg->comp_id       = MAV_COMP_ID_AUTOPILOT1;
	cfg->mav_type      = MAV_TYPE_FIXED_WING;
	cfg->autopilot     = MAV_AUTOPILOT_GENERIC;
	cfg->base_mode     = 0u;
	cfg->custom_mode   = 0u;
	cfg->system_status = MAV_STATE_STANDBY;
	cfg->tx_timeout_ms = 250u;
	cfg->use_dma       = false;
}

mavlink_tx_status_t mavlink_init(mavlink_tx_t *dev, const mavlink_config_t *cfg)
{
	if (dev == NULL || cfg == NULL || cfg->huart == NULL)
		return MAVLINK_TX_ERR_ARG;

	memset(dev, 0, sizeof(*dev));
	dev->cfg = *cfg;
	if (dev->cfg.sys_id == 0u)
		dev->cfg.sys_id = 1u;
	if (dev->cfg.comp_id == 0u)
		dev->cfg.comp_id = MAV_COMP_ID_AUTOPILOT1;
	if (dev->cfg.tx_timeout_ms == 0u)
		dev->cfg.tx_timeout_ms = 250u;

	return MAVLINK_TX_OK;
}

/* ---- Callback HAL & Watchdog ----------------------------------------------- */
void mavlink_on_tx_complete(mavlink_tx_t *dev, UART_HandleTypeDef *huart)
{
	if (dev == NULL || huart != dev->cfg.huart || !dev->tx_busy)
		return;

	dev->tx_tail  = (uint16_t)((dev->tx_tail + dev->tx_chunk) & MAVLINK_TX_FIFO_MASK);
	dev->tx_chunk = 0;
	dev->tx_busy  = false;
	uart_tx_kick(dev);
}

void mavlink_on_tx_error(mavlink_tx_t *dev, UART_HandleTypeDef *huart)
{
	if (dev == NULL || huart != dev->cfg.huart)
		return;

	dev->stat_bus_error++;
	if (dev->tx_busy) {
		dev->tx_tail  = (uint16_t)((dev->tx_tail + dev->tx_chunk) & MAVLINK_TX_FIFO_MASK);
		dev->tx_chunk = 0;
		dev->tx_busy  = false;
		uart_tx_kick(dev);
	}
}

void mavlink_poll(mavlink_tx_t *dev)
{
	if (dev == NULL || dev->cfg.huart == NULL)
		return;

	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	if (dev->tx_busy && (HAL_GetTick() - dev->tx_start_ms) > dev->cfg.tx_timeout_ms) {
		HAL_UART_AbortTransmit(dev->cfg.huart);
		dev->tx_tail  = (uint16_t)((dev->tx_tail + dev->tx_chunk) & MAVLINK_TX_FIFO_MASK);
		dev->tx_chunk = 0;
		dev->tx_busy  = false;
		dev->stat_timeout++;
	}
	uart_tx_kick(dev);
	__set_PRIMASK(pm);
}

/* ---- Gửi gói tổng quát ----------------------------------------------------- */
mavlink_tx_status_t mavlink_send_msg(mavlink_tx_t *dev, const mavlink_message_t *msg)
{
	static uint8_t buf[MAVLINK_MAX_PACKET_LEN]; /* chỉ gọi từ main loop */

	if (dev == NULL || msg == NULL || dev->cfg.huart == NULL)
		return MAVLINK_TX_ERR_ARG;

	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	uint32_t pm = __get_PRIMASK();
	__disable_irq();
	/* Khi FIFO đang rỗng và UART rảnh, đưa head/tail về 0 để cả cụm gói tin
	 * (ATTITUDE + SCALED_PRESSURE + VFR_HUD) nằm liền mạch từ tx_fifo[0], không bị cắt đôi */
	if (!dev->tx_busy && dev->tx_head == dev->tx_tail) {
		dev->tx_head = 0;
		dev->tx_tail = 0;
	}
	uint16_t used = (uint16_t)((dev->tx_head - dev->tx_tail) & MAVLINK_TX_FIFO_MASK);
	uint16_t free = (uint16_t)(MAVLINK_TX_FIFO_SIZE - 1u - used);

	if (len > free) {
		__set_PRIMASK(pm);
		dev->stat_drop++;
		return MAVLINK_TX_ERR_FULL;
	}

	uint16_t h = dev->tx_head;
	for (uint16_t i = 0; i < len; i++) {
		dev->tx_fifo[h] = buf[i];
		h               = (uint16_t)((h + 1u) & MAVLINK_TX_FIFO_MASK);
	}
	dev->tx_head = h;
	__set_PRIMASK(pm);

	return MAVLINK_TX_OK;
}

/* ---- Các gói tin chuẩn ----------------------------------------------------- */
mavlink_tx_status_t mavlink_send_heartbeat(mavlink_tx_t *dev)
{
	mavlink_message_t msg;

	if (dev == NULL)
		return MAVLINK_TX_ERR_ARG;

	mavlink_msg_heartbeat_pack(dev->cfg.sys_id, dev->cfg.comp_id, &msg, dev->cfg.mav_type,
	                           dev->cfg.autopilot, dev->cfg.base_mode, dev->cfg.custom_mode,
	                           dev->cfg.system_status);

	mavlink_tx_status_t rc = mavlink_send_msg(dev, &msg);
	if (rc == MAVLINK_TX_OK)
		dev->stat_hb_sent++;
	return rc;
}

mavlink_tx_status_t mavlink_send_attitude(mavlink_tx_t *dev, float roll_rad, float pitch_rad,
                                          float yaw_rad, float gx_rad_s, float gy_rad_s,
                                          float gz_rad_s)
{
	mavlink_message_t msg;

	if (dev == NULL)
		return MAVLINK_TX_ERR_ARG;

	mavlink_msg_attitude_pack(dev->cfg.sys_id, dev->cfg.comp_id, &msg, HAL_GetTick(), roll_rad,
	                          pitch_rad, yaw_rad, gx_rad_s, gy_rad_s, gz_rad_s);

	mavlink_tx_status_t rc = mavlink_send_msg(dev, &msg);
	if (rc == MAVLINK_TX_OK)
		dev->stat_att_sent++;
	return rc;
}

mavlink_tx_status_t mavlink_send_scaled_pressure(mavlink_tx_t *dev, float pressure_pa,
                                                 float temperature_c)
{
	mavlink_message_t msg;

	if (dev == NULL)
		return MAVLINK_TX_ERR_ARG;

	/* SCALED_PRESSURE: press_abs tính bằng hPa (1 hPa = 100 Pa), temperature tính bằng 0.01 °C */
	mavlink_msg_scaled_pressure_pack(dev->cfg.sys_id, dev->cfg.comp_id, &msg, HAL_GetTick(),
	                                 pressure_pa * 0.01f, 0.0f,
	                                 (int16_t)(temperature_c * 100.0f), 0);

	mavlink_tx_status_t rc = mavlink_send_msg(dev, &msg);
	if (rc == MAVLINK_TX_OK)
		dev->stat_baro_sent++;
	return rc;
}

mavlink_tx_status_t mavlink_send_vfr_hud(mavlink_tx_t *dev, float airspeed_m_s,
                                         float groundspeed_m_s, int16_t heading_deg,
                                         uint16_t throttle_pct, float alt_m, float climb_m_s)
{
	mavlink_message_t msg;

	if (dev == NULL)
		return MAVLINK_TX_ERR_ARG;

	mavlink_msg_vfr_hud_pack(dev->cfg.sys_id, dev->cfg.comp_id, &msg, airspeed_m_s,
	                         groundspeed_m_s, heading_deg, throttle_pct, alt_m, climb_m_s);

	return mavlink_send_msg(dev, &msg);
}

#ifdef USE_GPS
static uint8_t gps_mav_fix_type(void)
{
	if (!GPS_IsAlive())
		return GPS_FIX_TYPE_NO_GPS;
	if (!gps.gnssFixOK)
		return GPS_FIX_TYPE_NO_FIX;
	if (gps.carrSoln == 2)
		return GPS_FIX_TYPE_RTK_FIXED;
	if (gps.carrSoln == 1)
		return GPS_FIX_TYPE_RTK_FLOAT;
	switch (gps.fixType) {
	case 2:
		return GPS_FIX_TYPE_2D_FIX;
	case 3:
	case 4:
		return GPS_FIX_TYPE_3D_FIX;
	default:
		return GPS_FIX_TYPE_NO_FIX;
	}
}

static inline int16_t clamp_i16(int32_t v)
{
	if (v > 32767)
		return 32767;
	if (v < -32767)
		return -32767;
	return (int16_t)v;
}

mavlink_tx_status_t mavlink_send_gps_raw(mavlink_tx_t *dev)
{
	mavlink_message_t msg;

	if (dev == NULL)
		return MAVLINK_TX_ERR_ARG;

	uint8_t  fix      = gps_mav_fix_type();
	uint8_t  has_data = (fix != GPS_FIX_TYPE_NO_GPS);
	uint32_t vel      = has_data ? (uint32_t)(gps.gSpeed / 10) : UINT16_MAX;
	if (has_data && vel > 65534u)
		vel = 65534u;

	uint16_t cog = UINT16_MAX;
	if (has_data && gps.gSpeed > 500)
		cog = (uint16_t)((uint32_t)gps.headMot / 1000u % 36000u);

	mavlink_msg_gps_raw_int_pack(dev->cfg.sys_id, dev->cfg.comp_id, &msg,
	                             (uint64_t)HAL_GetTick() * 1000ULL, fix, gps.lat, gps.lon,
	                             gps.hMSL, has_data ? gps.pDOP : UINT16_MAX, UINT16_MAX,
	                             (uint16_t)vel, cog, gps.numSV, gps.height, gps.hAcc, gps.vAcc,
	                             gps.sAcc, gps.headAcc, 0);

	mavlink_tx_status_t rc = mavlink_send_msg(dev, &msg);
	if (rc == MAVLINK_TX_OK)
		dev->stat_gps_sent++;
	return rc;
}

mavlink_tx_status_t mavlink_send_global_position(mavlink_tx_t *dev)
{
	mavlink_message_t msg;

	if (dev == NULL)
		return MAVLINK_TX_ERR_ARG;

	uint16_t hdg = UINT16_MAX;
	if (gps.gSpeed > 500)
		hdg = (uint16_t)((uint32_t)gps.headMot / 1000u % 36000u);

	mavlink_msg_global_position_int_pack(
	    dev->cfg.sys_id, dev->cfg.comp_id, &msg, HAL_GetTick(), gps.lat, gps.lon, gps.hMSL,
	    dev->home_set ? (gps.hMSL - dev->home_alt_mm) : 0, clamp_i16(gps.velN / 10),
	    clamp_i16(gps.velE / 10), clamp_i16(gps.velD / 10), hdg);

	return mavlink_send_msg(dev, &msg);
}

void mavlink_on_new_gps_fix(mavlink_tx_t *dev)
{
	if (dev == NULL)
		return;

	uint8_t fix = gps_mav_fix_type();
	if (!dev->home_set && fix >= GPS_FIX_TYPE_3D_FIX && gps.numSV >= 6 && gps.hAcc < 5000u) {
		dev->home_set    = true;
		dev->home_alt_mm = gps.hMSL;
	}
	(void)mavlink_send_gps_raw(dev);
	if (fix >= GPS_FIX_TYPE_2D_FIX)
		(void)mavlink_send_global_position(dev);
}
#endif /* USE_GPS */
