# Hướng dẫn dùng ICM-45686 với STM32H562 (SPI + GPDMA + FIFO)

Tài liệu mô tả đúng những gì firmware trong repo này đang làm, kèm các lưu ý rút ra khi viết và kiểm tra code.
Trạng thái: **build sạch, chưa kiểm chứng trên phần cứng** — các mục "Cần đo khi chạy" ở cuối là việc phải làm.

Nguồn số liệu: datasheet TDK DS-000577 (file `TDK_DS_000577_ICM_45686.pdf` trong repo) và code driver.

---

## 1. Tổng quan

| Thành phần | Giá trị |
|---|---|
| MCU | STM32H562, 250 MHz (HSE 8 MHz + PLL1), không có data cache |
| IMU | ICM-45686 trên SPI1 |
| Chân | SCK=PA5, MISO=PA6, MOSI=PA7, CS=PA2 (GPIO mềm), INT1=PA0 (EXTI0, cạnh lên) |
| SPI | Mode 0, MSB first, 8 bit. Init chậm (prescaler 256), chạy thường 16 (≈15,6 MHz nếu kernel clock 250 MHz; datasheet cho tối đa 24 MHz) |
| DMA | GPDMA1: SPI1 RX = kênh 2, SPI1 TX = kênh 3 |
| Cấu hình IMU | Gyro 1600 Hz ±500 dps, Accel ±4 g, FIFO STREAM, watermark 4 frame |
| dt | TIM5 32 bit đếm tự do 1 MHz (`Micros()`), và timestamp trong FIFO |

File chính:

- `src/icm45686_fc.c`, `include/icm45686_fc.h`: driver tầng trên (state machine DMA, ring, calib).
- `include/imu/`: driver TDK (`inv_imu_driver.c`, regmap) — không sửa.
- `src/board.c`: cấu hình GPIO/SPI/IMU/TIM5. `src/msp.c`: GPIO AF, DMA, NVIC. `src/stm32h5xx_it.c`: ISR.
- `src/main.c`: init, calib, vòng lặp tiêu thụ mẫu, Mahony.

---

## 2. Luồng khởi động (thứ tự quan trọng)

```
HAL_Init → SystemClock_Config → GPIOS_Init → SPI1_Init → TIMER5_Init
  → ICM45686_Init            (WHO_AM_I, soft reset, cấu hình INT1/FIFO/FSR/ODR)
  → kiểm tra g_imu_rc        (khác 0 thì Error_Handler)
  → icm45686_calibrate_gyro  (SPI blocking, board phải đứng yên)
  → kiểm tra g_calib_rc      (khác 0 thì Error_Handler)
  → IMU_FilterInit, Mahony_Init
  → icm45686_start           (flush FIFO, bật streaming)
  → vòng lặp: icm45686_pop → lọc → Mahony
```

Điểm cần nhớ:

1. **Calib trước, start sau.** Calib đọc thanh ghi dữ liệu bằng SPI blocking trên cùng SPI1. Nếu DMA FIFO đã chạy thì hai đường giẫm nhau trên một bus. Trước `start`, `streaming = false` nên `on_drdy` thoát ngay, ngắt không chen vào.
2. **`icm45686_start` flush FIFO.** Trong lúc calib (~0,3 s với 500 mẫu), FIFO vẫn nhận mẫu và STREAM ghi đè. Flush để ngắt đầu tiên không đọc phải frame cũ. Việc flush dùng SPI blocking nên chỉ an toàn khi chưa có DMA nào chạy (đúng ở thời điểm này).
3. **Kiểm tra lỗi init trước khi calib.** Nếu WHO_AM_I sai (`ICM45686_ERR_WHOAMI = -100`) thì dừng ngay, xem `g_imu_rc`, `g_imu_whoami` (kỳ vọng 0xE9) trong debugger.
4. **Calib thất bại thì dừng.** `ICM45686_ERR_MOVING` nghĩa là gyro rung quá `max_spread_dps` (2,0). Hệ quả: mỗi lần bật nguồn phải để board nằm yên, nếu không firmware treo ở `Error_Handler`.
5. `TIMER5_Init` phải chạy trước mọi chỗ gọi `Micros()`.

---

## 3. Cơ chế FIFO của ICM-45686

- FIFO mặc định 2 KB (`FIFO_DEPTH` 0x07). Driver đang dùng `FIFO_DEPTH_MAX` (0x1E) — xem mục 8.
- Chế độ STREAM: đầy thì ghi đè mẫu cũ. STREAM chỉ hỗ trợ frame 8/16/20 byte.
- `FIFO_COUNT` (0x12 = byte cao, 0x13 = byte thấp) đếm theo **frame**, không theo byte. `FIFO_DATA` = 0x14.
- **Errata AN-000364:** đọc `FIFO_COUNT` hai lần và dùng giá trị lần hai. Code làm đúng việc này.
- Watermark 16 bit; ngắt FIFO_THS phát khi số frame ≥ ngưỡng (`fifo_wr_wm_gt_th = EQ_OR_GT`).

### Frame 16 byte (accel + gyro + temp + timestamp)

| Offset | Nội dung |
|---|---|
| 0 | Header |
| 1–6 | Accel X/Y/Z (16 bit mỗi trục) |
| 7–12 | Gyro X/Y/Z |
| 13 | Nhiệt độ (**1 byte**) |
| 14–15 | Timestamp (16 bit) |

Header: bit7 EXT_HEADER, bit6 ACCEL_EN, bit5 GYRO_EN, bit4 HIRES_EN, bit3 TMST_FIELD_EN, bit2 FSYNC_TAG_EN, bit1 ACCEL_ODR, bit0 GYRO_ODR.
Header hợp lệ cho cấu hình này thỏa `(h & 0xF8) == 0x68`; code đếm `stat_invalid` và bỏ frame sai.

### Nhiệt độ — hai công thức khác nhau

- Thanh ghi dữ liệu (16 bit): `T = TEMP_DATA / 128 + 25`.
- FIFO (1 byte, datasheet 17.14): `T = FIFO_TEMP_DATA / 2 + 25`.

Đường FIFO dùng công thức thứ hai.

### Timestamp

- Chỉ khác 0 khi `FIFO_TMST_FSYNC_EN` (FIFO_CONFIG4) và `TMST_EN` được bật. Driver TDK tự bật cả hai khi `accel_en && gyro_en` và mode khác BYPASS.
- Độ phân giải mặc định 1 µs (`TMST_RESOL` = 0; 1 = 16 µs). Driver không ghi bit này. Với 16 bit, timestamp tràn sau khoảng 65,5 ms.
- Đồng hồ của timestamp là đồng hồ nội của IMU, sai lệch vài % so với MCU. Chỉ dùng **hiệu hai timestamp liên tiếp**, đừng so với `Micros()`.

---

## 4. Pipeline thời gian thực

```
IMU INT1 (xung cạnh lên khi FIFO ≥ 4 frame)
   │
   ▼
EXTI0 ISR → HAL_GPIO_EXTI_Rising_Callback → icm45686_on_drdy
   │   kiểm tra streaming; nếu đang bận → stat_overrun++ và thoát
   │   ghi drdy_ts_us, đặt dma_busy, state = READ_COUNT
   ▼
DMA #1  (3 byte: 0x12|READ, 0, 0)  → đọc FIFO_COUNT lần 1
   ▼   on_dma_complete
DMA #2  (3 byte, cùng tx)          → đọc FIFO_COUNT lần 2 (errata)
   ▼   n = số frame; n==0 → idle; n>8 → stat_fifo_overflow++, kẹp còn 8
DMA #3  (1 + n×16 byte: 0x14|READ + 0...)  → đọc FIFO_DATA
   ▼   on_dma_complete → parse từng frame
parse: kiểm tra header → parse accel/gyro → temp (byte 13) → timestamp (byte 14–15)
   │   dt_us = (uint16_t)(ts − ts_trước)
   ▼
fill_sample (xoay theo align, đổi sang dps/g, trừ bias gyro)
   ├─ buf[2] + seqlock  → icm45686_get_latest()  (chỉ mẫu mới nhất)
   └─ ring[64] SPSC     → icm45686_pop()          (đủ mọi mẫu)
   ▼
main: pop → biquad 40 Hz → Mahony (dt = dt_us) → attitude
```

Tại sao 3 giao dịch DMA nối tiếp: mỗi giao dịch SPI full-duplex phải giữ CS thấp trọn vẹn, nên `on_dma_complete` kéo CS lên rồi mới khởi động giao dịch kế tiếp. Số byte của bước 3 phụ thuộc vào `n` đọc ở bước 2.

State machine (`fifo_state_t`): `FIFO_IDLE → READ_COUNT → READ_COUNT2 → READ_DATA → IDLE`. Lỗi SPI (`on_dma_error`) kéo CS lên, tăng `stat_dma_error`, về IDLE.

Ngân sách thời gian (ước tính): 3+3+65 byte ở ~15,6 MHz ≈ 36 µs dữ liệu cộng overhead HAL/DMA mỗi giao dịch; mỗi ngắt cách nhau ~2,5 ms (1600 Hz / 4 frame). Còn rất nhiều dư địa. Nếu muốn giảm ngắt, tăng `fifo_wm_th` (đồng thời tăng `FIFO_MAX_FRAMES` và kích thước `dma_tx/dma_rx` cho đủ `1 + n×16`).

### Hai cách lấy dữ liệu

| API | Dùng cho | Đặc điểm |
|---|---|---|
| `icm45686_pop()` | Bộ lọc, Mahony, logging | Nhận đủ mọi mẫu (1600/s). Ring 64 mẫu ≈ 40 ms dự phòng. `stat_ring_drop` tăng nếu main chậm |
| `icm45686_get_latest()` | Vòng PID chạy chậm hơn | Seqlock, không chặn, chỉ mẫu mới nhất. Trả `true` nếu có mẫu mới từ lần đọc trước |

Ring là SPSC (ISR chỉ ghi `ring_head`, main chỉ ghi `ring_tail`) nên không cần khóa, chỉ cần `__DMB()` giữa ghi dữ liệu và công bố con trỏ. `IMU_RING_SIZE` phải là lũy thừa của 2.

---

## 5. Cấu hình ngắt và ưu tiên

| Ngắt | Ưu tiên |
|---|---|
| SPI1 | 2 |
| EXTI0 (INT1) | 1 |
| GPDMA1 kênh 2/3 (SPI) | 5 |
| I2C1, USART3, DMA I2C/UART | 5 |

Lưu ý:

- SysTick của HAL hiện dùng ở mức mặc định. Khi thêm FreeRTOS (SysTick dành cho RTOS), phải chuyển time base của HAL sang một timer khác (`HAL_InitTick` override). TIM5 đang là bộ đếm micro tự do, **không** nên dùng làm tick của HAL.
- Callback `HAL_SPI_TxRxCpltCallback` được gọi từ ngữ cảnh ngắt DMA/SPI; không làm việc nặng trong đó.
- Khi dùng FreeRTOS, ISR gọi API RTOS phải có ưu tiên ≥ `configMAX_SYSCALL_INTERRUPT_PRIORITY`.

---

## 6. Calibration gyro

`icm45686_calibrate_gyro(imu, samples, max_spread_dps)`:

- Đọc `samples` mẫu qua `icm45686_read_blocking`, cách nhau một chu kỳ ODR (`icm_port_sleep_us`).
- Tính trung bình làm bias; nếu (max − min) trên trục nào lớn hơn `max_spread_dps` thì trả `ICM45686_ERR_MOVING` và **giữ bias cũ**.
- Chỉ chạy trước `icm45686_start`.
- 500 mẫu ở 1600 Hz ≈ 0,3 s: đủ để loại rung thô, chưa đủ để khử nhiễu ngẫu nhiên tốt. Muốn bias chính xác hơn thì tăng `samples` (ví dụ 2000–4000).
- Sau `start`, `discard` bỏ khoảng 10 ms đầu (gyro mới bật chưa ổn định).

---

## 7. Tính dt cho bộ lọc / Mahony

Thứ tự ưu tiên:

1. `s.dt_us` (hiệu timestamp FIFO, ≈ 625 µs ở 1600 Hz) → `dt = dt_us × 1e-6`.
2. Nếu `dt_us == 0` (mẫu đầu sau `start`, hoặc timestamp chưa bật) hoặc `dt > 4/IMU_FS_HZ` (mất frame) → dùng `1/IMU_FS_HZ`.

Không dùng `Micros()` tại lúc xử lý cho từng mẫu: một cụm 4 mẫu được xử lý gần như cùng lúc nên dt sẽ ra gần 0.

`IMU_FS_HZ` trong `main.c` phải khớp `cfg.gyro_odr` trong `board.c`. Đổi ODR là đổi cả hai chỗ, cộng cả tần số cắt biquad.

Đơn vị Mahony: gyro đưa vào là **rad/s** (đã nhân `DEG2RAD`). Mình chưa đọc `quaternion.c` để khẳng định; nếu `Quaternion_Integrate` nhận độ/giây thì bỏ phép nhân này.

---

## 8. Những điều cần lưu ý (bẫy đã gặp)

1. **Thứ tự byte của `FIFO_COUNT`:** `FIFO_COUNT_0` (0x12) là byte **cao**. Dùng `FORMAT_16_BITS_DATA(endianness_data, ...)` như driver, đừng tự ghép `rx[1] | rx[2]<<8`.
2. **Temp trong FIFO chỉ 1 byte.** `parse_burst` đọc 7 từ 16 bit nên từ thứ 7 trộn byte temp với byte timestamp; code ghi đè `d.temp_data` từ `f[13]`. Đừng dùng giá trị thô từ `parse_burst`.
3. **Hai công thức nhiệt độ** (mục 3). Đừng dùng `/128` cho dữ liệu FIFO.
4. **tx phải dài bằng rx.** SPI full-duplex cần đẩy đủ byte mới có xung clock để nhận; byte `tx[1…]` đặt 0.
5. **Buffer DMA đủ lớn:** `dma_tx/dma_rx` = 160 byte, đủ cho `1 + 8×16 = 129`. Đổi `FIFO_MAX_FRAMES` thì phải kiểm tra lại (có `aligned(32)` cho trường hợp bật cache).
6. **Ring phải lũy thừa của 2**, và `stat_ring_drop` là chỉ báo main không kịp.
7. **`FIFO_DEPTH_MAX` (0x1E) chưa rõ nghĩa.** Driver TDK ghi chú liên quan Errata AN-000364 nhưng mình chưa đọc tài liệu đó. Datasheet đề nghị 0x07 (2 KB) làm mặc định; 8 KB là 0x1F khi APEX tắt. Nếu không có lý do cụ thể, đổi sang 0x07. Với watermark 4 frame thì độ sâu không phải điểm nghẽn.
8. **Cạnh ngắt.** INT1 cấu hình xung (pulse), active high, push-pull; MCU bắt cạnh lên. Nếu `n > FIFO_MAX_FRAMES` thì phần còn lại **không** tự phát ngắt mới (không có cạnh mới nếu FIFO còn ≥ ngưỡng), nên `stat_fifo_overflow` tăng là dấu hiệu cần tăng `FIFO_MAX_FRAMES` hoặc giảm watermark.
9. **`on_drdy` bỏ qua ngắt khi đang bận** (`stat_overrun++`). Nếu số này tăng, chuỗi DMA dài hơn chu kỳ ngắt.
10. **Không gọi SPI blocking khi DMA có thể đang chạy** (`icm45686_read_blocking`, đọc/ghi thanh ghi thủ công…). Chỉ dùng trước `start`, hoặc sau khi dừng streaming và chờ `dma_busy == false`..
12. **Link libm:** dùng Mahony kéo theo `sqrt`/`asin` của newlib; `CMakeLists.txt` đã thêm `m` vào `target_link_libraries`. Nếu thiếu sẽ gặp `undefined reference to __errno`.
13. **`USE_MAG` / `USE_GPS`** đang tắt trong `config.h`; bật lại phải build và kiểm tra riêng.
14. **Nguồn nhiễu rung:** calib dừng ở `Error_Handler` nếu board rung, kể cả động cơ/quạt ở gần. Thiết kế cho phép retry (hiện chưa có) nếu cần.

---

## 9. Thống kê chẩn đoán

| Biến | Ý nghĩa | Kỳ vọng |
|---|---|---|
| `g_imu_hz` | Số mẫu main `pop` được mỗi giây | ≈ 1600 |
| `imu.stat_frames` | Tổng frame hợp lệ đã nhận | tăng ≈ 1600/s |
| `imu.stat_invalid` | Frame sai header hoặc mẫu 0x8000 | 0 |
| `imu.stat_overrun` | INT tới khi DMA trước chưa xong | 0 |
| `imu.stat_dma_error` | Lỗi SPI/DMA | 0 |
| `imu.stat_fifo_overflow` | `FIFO_COUNT` > `FIFO_MAX_FRAMES` | 0 |
| `imu.stat_ring_drop` | Ring đầy, main xử lý chậm | 0 |
| `g_dt` | dt của mẫu vừa xử lý | ≈ 625 µs |
| `g_imu_rc`, `g_calib_rc`, `g_imu_whoami` | Kết quả init, calib, WHO_AM_I | 0, 0, 0xE9 |

---

## 10. Cần đo khi chạy (chưa kiểm chứng)

1. `g_imu_hz` ≈ 1600 và các `stat_*` ở bảng trên đều 0 sau vài phút.
2. `fifo_n` mỗi ngắt khoảng 4 và header khoảng 0x68 (đặt breakpoint ở `READ_DATA`).
3. `s.dt_us` ≈ 625. Luôn bằng 0 nghĩa là timestamp chưa bật. Lệch 1–3 % so với 625 là sai số đồng hồ nội của IMU, không phải lỗi code.
4. `temp_c` hợp lý (nhiệt độ phòng cộng chút tự nóng, khoảng 25–45 °C).
5. Accel khi đặt phẳng: Z ≈ ±1 g; gyro sau calib ≈ 0 dps.
6. Chiều quay: xoay board quanh từng trục, xem dấu roll/pitch/yaw có đúng chiều (`align` trong cấu hình).
7. Đo SCK bằng oscilloscope để xác nhận tần số thực (kernel clock SPI1 có phải 250 MHz không).
8. Quaternion có đơn vị gyro đúng (rad/s) không (mục 7).

---

## 11. Việc còn lại / ý tưởng

- Quyết định `FIFO_DEPTH_MAX` hay 0x07 (mục 8.7).
- Sửa `gpio_it` (mục 8.11).
- Chuyển time base HAL khỏi SysTick khi thêm FreeRTOS.
- Cho phép retry calib thay vì dừng.
- Ghi bias vào flash để bỏ calib mỗi lần bật nguồn (cần kiểm tra drift theo nhiệt độ).
- Bật lại và test `USE_MAG`, `USE_GPS`.
