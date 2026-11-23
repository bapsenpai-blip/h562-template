#ifndef UBX_GPS_H
#define UBX_GPS_H

#include "stm32h5xx_hal.h"
#include <stdint.h>

/* ===================== CẤU HÌNH GPS =====================
   GPS_AUTOCONFIG = 1: lúc khởi động MCU tự dò baud của module (9600/38400/57600/115200)
   rồi gửi lệnh cấu hình vào RAM của module (không ghi flash, rút điện là về mặc định):
     - chỉ xuất UBX (tắt NMEA), bật UBX-NAV-PVT trên UART1 của module
     - tốc độ cập nhật GPS_RATE_MS
     - đổi baud của module sang GPS_BAUD
   Gửi cả lệnh kiểu M10 (CFG-VALSET) lẫn kiểu M8 (CFG-PRT/MSG/RATE), module nào không
   hiểu thì tự bỏ qua, nên dùng được cho cả NEO-M8N và M10.
   Đặt 0 nếu bạn đã cấu hình sẵn bằng u-center và lưu vào flash. */
#ifndef GPS_AUTOCONFIG
#define GPS_AUTOCONFIG   1
#endif
#define GPS_BAUD         115200UL  /* phải khớp huart3.Init.BaudRate */
#define GPS_RATE_MS      200U      /* 200 ms = 5 Hz (M10 nhiều chòm sao thường giới hạn ~5-10 Hz) */
#define GPS_TIMEOUT_MS   1500U     /* quá lâu không có NAV-PVT => coi như mất GPS */

/* Dữ liệu GPS đã giải mã từ UBX-NAV-PVT (đơn vị giữ nguyên như u-blox) */
typedef struct {
    uint32_t iTOW;          // ms, thời gian trong tuần GPS
    uint16_t year;
    uint8_t  month, day, hour, min, sec;
    uint8_t  fixType;       // 0 none, 1 dead-reckoning, 2 2D, 3 3D, 4 GNSS+DR, 5 time only
    uint8_t  gnssFixOK;     // flags bit0: fix hợp lệ
    uint8_t  carrSoln;      // flags bit6..7: 0 không RTK, 1 RTK float, 2 RTK fixed
    uint8_t  numSV;         // số vệ tinh đang dùng
    int32_t  lon, lat;      // 1e-7 độ
    int32_t  height, hMSL;  // mm (trên ellipsoid / trên mực nước biển)
    uint32_t hAcc, vAcc;    // mm
    int32_t  velN, velE, velD, gSpeed; // mm/s
    int32_t  headMot;       // 1e-5 độ (hướng chuyển động - course over ground)
    uint32_t sAcc;          // mm/s
    uint32_t headAcc;       // 1e-5 độ
    uint16_t pDOP;          // 0.01
    uint32_t rx_tick_ms;    // HAL_GetTick() lúc giải mã xong gói (0 = chưa có gói nào)
} GPS_Data_t;

/* Bộ đếm để debug (xem trong Watch) */
typedef struct {
    uint32_t bytes;     // tổng byte đã nhận từ GPS
    uint32_t pvt_ok;    // số gói NAV-PVT giải mã thành công
    uint32_t bad_ck;    // số gói sai checksum
    uint32_t overruns;  // số byte bị mất vì ring buffer đầy
    uint32_t uart_err;  // số lần UART báo lỗi
    uint32_t ack;       // số UBX-ACK-ACK nhận được (module chấp nhận lệnh cấu hình)
    uint32_t nak;       // số UBX-ACK-NAK (module từ chối lệnh - bình thường với lệnh khác đời)
} GPS_Stats_t;

extern GPS_Data_t gps;
extern volatile GPS_Stats_t gps_stats;

void GPS_Init(UART_HandleTypeDef *huart);   // gọi sau khi UART đã init (chặn ~0.5 s nếu AUTOCONFIG)
int  GPS_Poll(void);                        // gọi trong vòng lặp main; trả 1 nếu vừa có gói NAV-PVT mới
uint8_t GPS_IsAlive(void);                  // 1 nếu nhận NAV-PVT trong GPS_TIMEOUT_MS gần nhất

/* Gọi từ callback của HAL */
void GPS_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void GPS_OnError(UART_HandleTypeDef *huart);

#endif
